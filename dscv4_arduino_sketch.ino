#include <pb_encode.h> //
#include <pb_common.h>
#include <pb.h>
#include <pb_decode.h>

#include "Msg.pb.h"

// consider switching to https://github.com/rlogiacco/CircularBuffer
// MAKE SURE APPLY THIS FIX BEFORE USING: https://github.com/sdesalas/Arduino-Queue.h/issues/3
#include "Queue.h" // copied this file to arduino-1.8.9/libraries/Queue from https://github.com/sdesalas/Arduino-Queue.h/blob/51d2d0c69f5c6d88997b4a900637fcf35294317d/Queue.h

#include <time.h>
#include <sys/time.h>                   // struct timeval

#define TZ              1       // (utc+) TZ in hours
#define DST_MN          60      // use 60mn for summer time in some countries
#define TZ_MN           ((TZ)*60)
#define TZ_SEC          ((TZ)*3600)
#define DST_SEC         ((DST_MN)*60)

#include <SPI.h> // used for connection to LoRa radio
#include <LoRa.h>
#include "SSD1306.h"  
#include <TinyGPS++.h>     
#include "SPIFFS.h"

// Use Tools->Board->T-Beam
// Tested on TTGO T-Beam boards with silk-screened label: "T22_V07 20180711"


// Here's how to compile Msg.proto with nanopb on Ubuntu 18.04 following https://github.com/vladimirvivien/iot-dev/tree/master/esp8266/esp8266-dht11-temp#install-and-configure-nanopb
//  sudo apt-get install protobuf-compiler python-protobuf libprotobuf-dev
//  wget https://github.com/nanopb/nanopb/archive/0.3.9.3.zip
//  unzip
//  ~/src/nanopb-0.3.9.3/generator/proto$ make
//  protoc --plugin=protoc-gen-nanopb=/home/${USER}/src/nanopb-0.3.9.3/generator/protoc-gen-nanopb --nanopb_out=. --proto_path=/home/${USER}/src/nanopb-0.3.9.3/generator/proto/nanopb.proto --proto_path=/usr/include --proto_path=. Msg.proto
//  ~/src/nanopb-0.3.9.3$ cp pb.h pb_encode.* pb_decode.* pb_common.* /home/tz/Desktop/tbeam/arduino-1.8.9/libraries/Nanopb
//  cp nanopb.proto to sketch dir
//  restart arduino ide
//  Sketch > Include Library > Contributed Library, select Nanopb.

// Here's one way to connect to the board's UART:
//  python /usr/lib/python2.7/dist-packages/serial/tools/miniterm.py --eol LF /dev/ttyUSB0 115200



// LoRa globals --------------------------------------------------
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define LORA_BAND  915.05E6
int totalReceivedBytes = 0;
bool itsNotOurSlot = true;

// OLED globals --------------------------------------------------
const int OLED_I2C_ADDR = 0x3c;
const int OLED_I2C_SCA_PIN = 21;
const int OLED_I2C_SCL_PIN = 22;
SSD1306 display(OLED_I2C_ADDR, OLED_I2C_SCA_PIN, OLED_I2C_SCL_PIN); // https://techtutorialsx.com/2017/12/03/esp32-arduino-ssd1306-oled-redrawing-a-string/
const int OLED_RST_PIN_MAYBE = 16;
const int OLED_UPDATE_INTERVAL_MS = 500; // TODO: increase this as much as possible to conserve battery
int lastOLEDUpdateTime = 0;

// OLED default "to-display" globals
char millisecondsSinceBootOLEDText[32] = "DSCv4 #x xxxxs 12:12:12"; 
char batteryVoltageOLEDText[32] =        "Battery: xxxxx";
char gpsTimeOLEDText[32] =               "GPS time: xxxxxxxxxxxxxxX";
char evenMoreOLEDText[32] =              "=-=-=-=-=-=-=-=-=-=-=-=-="; /*__VERSION__  __DATE__ */
char messagingOLEDText[32] =             "Last msg: xxxxxxxxxxxxxxX";
char moreOLEDText[32] =                  "foo: xxxxxxxxxxxxxxxxxxxX";

// GPS globals --------------------------------------------------
TinyGPSPlus gps;                            
HardwareSerial GPSSerial1(1);    
const int GPS_BAUDRATE = 9600;
const int GPS_SERIAL_TX_PIN = 12;
const int GPS_SERIAL_RX_PIN = 15;

// Battery voltage ADC globals ----------------------------------
const byte ADCBOARDVOLTAGE_PIN = 35; // Prefer Use of ADC1 (8 channels, attached to GPIOs 32 - 39) . ADC2 (10 channels, attached to GPIOs 0, 2, 4, 12 - 15 and 25 - 27)
const byte ADC_BITS = 10; // 10 - 12 bits
int BATT_ADC_VAL_TO_PCT_REMAIN[50];

// Message queue globals ----------------------------------------
Queue<Msg> sentMsgs(30);     // assume we'll never have >N sent msgs that haven't been logged to flash
Queue<Msg> outboundMsgs(30); // assume we'll never have >N composed-here or to-repeat msgs that haven't been transmitted
Queue<Msg> receivedMsgs(30); // assume we'll never have >N received msgs that haven't been logged to flash 

// Misc globals -------------------------------------------------
const int SERIAL_BAUDRATE = 115200;
int nodeNum = -1;
unsigned int sentPktCounter = 0;
const int TIME_REQD_TO_SEND_LARGEST_PKT_SEC = 2; // TODO: measure w/ waterfall
bool firstGpsFix = true;
bool sentOnePktDuringOurSlot = false;
//const int LED_IO14_PIN_NUMBER = 2;
String usbSerialCommand = "";
const char MSG_LOG_FILENAME[] = "/msgLog.txt";
const char BATT_LOG_FILENAME[] = "/battLog.txt";
int lastFlashWriteTime = 0;
const int FLASH_WRITE_INTERVAL_MS = 5 * 60 * 1000;


void setup() {
  // Initialize UART ------------------------------------------------------------------------------
  Serial.begin(SERIAL_BAUDRATE);
  //while (!Serial); // TODO: blink led to indicate failure?
  Serial.println("BBQ Research DSCv4 firmware version " __VERSION__ " compiled on " __DATE__ " at " __TIME__); // TODO: use COMPILE_DATE
  
  // Initialize OLED ------------------------------------------------------------------------------
  pinMode(OLED_RST_PIN_MAYBE, OUTPUT);
  digitalWrite(OLED_RST_PIN_MAYBE, LOW);  // "set GPIO16 low to reset OLED"
  delay(50); 
  digitalWrite(OLED_RST_PIN_MAYBE, HIGH); // "while OLED is running, must set GPIO16 in high"
  delay(200);             // arbitrary
  display.init();
  display.flipScreenVertically();  
  display.setFont(ArialMT_Plain_10);  // TODO: fixed width
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  Serial.println(" OLED initialized.");  

  // Initialize GPS ------------------------------------------------------------------------------
  GPSSerial1.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_SERIAL_TX_PIN, GPS_SERIAL_RX_PIN);
  Serial.println(" GPS initialized.");

  // Initialize battery voltage ADC --------------------------------------------------------------
  analogReadResolution(ADC_BITS); // "Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution."
  analogSetAttenuation(ADC_11db); // "Default is 11db which is very noisy. Recommended to use 2.5 or 6. Options ADC_0db (1.1V), ADC_2_5db (1.5V), ADC_6db (2.2V), ADC_11db (3.9V but max VDD=3.3V)"
  pinMode(ADCBOARDVOLTAGE_PIN, INPUT);  
  BATT_ADC_VAL_TO_PCT_REMAIN[46] = 101;
  BATT_ADC_VAL_TO_PCT_REMAIN[45] = 100; // 45 is highest value we've seen
  BATT_ADC_VAL_TO_PCT_REMAIN[44] = 99;
  BATT_ADC_VAL_TO_PCT_REMAIN[43] = 98; // the batt LED is green at 43
  BATT_ADC_VAL_TO_PCT_REMAIN[42] = 93;
  BATT_ADC_VAL_TO_PCT_REMAIN[41] = 86;
  BATT_ADC_VAL_TO_PCT_REMAIN[40] = 79;
  BATT_ADC_VAL_TO_PCT_REMAIN[39] = 72;
  BATT_ADC_VAL_TO_PCT_REMAIN[38] = 65;
  BATT_ADC_VAL_TO_PCT_REMAIN[37] = 58;
  BATT_ADC_VAL_TO_PCT_REMAIN[36] = 51;
  BATT_ADC_VAL_TO_PCT_REMAIN[35] = 44;
  BATT_ADC_VAL_TO_PCT_REMAIN[34] = 37;
  BATT_ADC_VAL_TO_PCT_REMAIN[33] = 30;
  BATT_ADC_VAL_TO_PCT_REMAIN[32] = 23;
  BATT_ADC_VAL_TO_PCT_REMAIN[31] = 16;
  BATT_ADC_VAL_TO_PCT_REMAIN[30] = 9;
  BATT_ADC_VAL_TO_PCT_REMAIN[29] = 2;
  BATT_ADC_VAL_TO_PCT_REMAIN[28] = 0;// 28 is lowest val we've seen
  BATT_ADC_VAL_TO_PCT_REMAIN[27] = -1;   
  Serial.println(" Battery voltage monitor initialized.");


  // Initialize Node ID -------------------------------------------------------------------------------
  uint64_t chipid = ESP.getEfuseMac(); // The chip ID is essentially its MAC address (length: 6 bytes).

  switch((uint16_t)(chipid>>32)) {
    case 0xB04A:
      nodeNum = 0;
      break;
    case 0x084B:
      nodeNum = 1;
      break;
    case 0xE84A:
      nodeNum = 2;
      break;
    case 0x8C4A:
      nodeNum = 3;
      break;
    case 0x644A:
      nodeNum = 4;
      break;
    case 0xF44A:
      nodeNum = 5;
      break;    
    case 0xD803:
      nodeNum = 6;
      break;
    case 0x34A7:
      nodeNum = 7;
      break;     
         
    case 0x60A2:
      nodeNum = 10;
      break;
    case 0x648E:
      nodeNum = 11;
      break;                
    default : /* Optional */
      Serial.println(" we're running on an unexpected esp32 device!");
  }  
  Serial.printf(" Node ID (%d) (0x%04X) initialized.\n", nodeNum, (uint16_t)(chipid>>32));
  
  // Initialize LoRa -------------------------------------------------------------------------------
  LoRa.setPins(SS, RST, DI0);
  //LoRa.setFrequency(frequency);                               // "(433E6, 866E6, 915E6)" (we set the freq using LORA_BAND below)
  //LoRa.setSpreadingFactor(10);   <---- this may have been cause of modem flakiness   //TODO: use constants       // "Supported values are between 6 and 12. If a spreading factor of 6 is set, implicit header mode must be used to transmit and receive packets. 7 is default."
  //LoRa.setSignalBandwidth(125E3);  <---- this may have been cause of modem flakiness                             // "Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3 (default), and 250E3."
  //LoRa.setCodingRate4(5);   <---- this may have been cause of modem flakiness                                    // "Supported values are between 5 (default) and 8, these correspond to coding rates of 4/5 and 4/8. The coding rate numerator is fixed at 4."
  //LoRa.setPreambleLength(8);   <---- this may have been cause of modem flakiness                                 // "Supported values are between 6 and 65535 (8 is default)"
  //LoRa.setSyncWord(0x12);                                     // "byte value to use as the sync word, defaults to 0x12"
  //LoRa.enableCrc(); //LoRa.disableCrc();                      // "by default a CRC is not used."
  //LoRa.enableInvertIQ(); //LoRa.disableInvertIQ();            // "by default a invertIQ is not used."
  //LoRa.setTxPower(txPower);                                   // "TX power in dB, defaults to 17"
  if (!LoRa.begin(LORA_BAND)) {
    Serial.println(" Starting LoRa failed!");
    Serial.flush();
    while (1);
  }
  Serial.println(" LoRa intialized.");

  // Initialize SPIFFS -------------------------------------------------------------------------
  if (!SPIFFS.begin(true)) { // this can take 5+ sec on a new device
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }  
  Serial.println(" SPIFFS intialized.");

  // Initialize misc ---------------------------------------------------------------------------
  //pinMode(LED_IO14_PIN_NUMBER, OUTPUT); 

  Serial.println("");
  Serial.flush();
  delay(500); // maybe not necessary?
  
  Serial.printf("Adding two fake entries to receivedMsgs queue\n");
  Msg msg1;
  msg1.originatorNodeId = 99;
  Msg msg2;
  msg2.originatorNodeId = 999;
  receivedMsgs.push(msg1);
  receivedMsgs.push(msg2);
  
}

static void attemptGpsNmeaDecode(unsigned long ms) {   // TODO: how much time is required to read all avail data from gps             
  unsigned long start = millis();
  //do {
    while (GPSSerial1.available()) {
      gps.encode(GPSSerial1.read());
    }
  //} while (millis() - start < ms); // this sets sets an upper limit on time consumed
  
  if (millis() > 5000 && gps.charsProcessed() < 10) { // TODO: use constants
    Serial.println(F("\nwe've received <10 chars from GPS and we booted >5sec ago - check gps wiring!"));
    // dump the stream to Serial
    //Serial.println("GPS stream dump:");
    //while (true) { // infinite loop
    //  if (ss.available() > 0) // any data coming in?
    //    Serial.write(ss.read());   
    //  } 
  }   
}


void updateOLED() {
  sprintf(millisecondsSinceBootOLEDText, "DSCv4 #%d up %ds " __TIME__, nodeNum, millis()/1000);   
  display.clear();
  display.drawString(0, 0,  String(millisecondsSinceBootOLEDText)); // x, y, strToPrint    
  display.drawString(0, 10, String(evenMoreOLEDText));
  display.drawString(0, 20, String(batteryVoltageOLEDText));     
  display.drawString(0, 30, String(gpsTimeOLEDText));     
  display.drawString(0, 40, String(messagingOLEDText));     
  display.drawString(0, 50, String(moreOLEDText));     
  display.display(); // TODO: call this automagically so we don't spend 2+ hrs debugging in las vegas
}

int getOurTimeQuality() {
  return 0;  
}

void handleReceivedLoraPkt(int packetSize){
  uint8_t incomingPkt[256];// for the results of protobuf decoding

  // received a packet
  Serial.printf("Received LoRa packet of size %d\n", packetSize);

  // print the entire received packet 
  Serial.printf(" Packet bytes in hex: ");
  int i = 0;
  while (LoRa.available()) {
    char c = (char)LoRa.read();
    Serial.printf("%x", c);
    incomingPkt[i] = c;
    i += 1;
  }
  Serial.printf("\n");  
    
  Msg decodedMsg = Msg_init_zero;
  pb_istream_t istream = pb_istream_from_buffer(incomingPkt, packetSize);
  bool status = pb_decode(&istream, Msg_fields, &decodedMsg);
  if (!status) {
    printf("Decoding FAILED: %s\n\n", PB_GET_ERROR(&istream));
    return;
  }   
  
  totalReceivedBytes += packetSize;
  sprintf(messagingOLEDText, "Total RX bytes: %d", totalReceivedBytes);
  
  printf(" Decoded content: '%s'\n", (char*)decodedMsg.content);  
  printf(" Decoded originatorNodeId: %d\n", (int)decodedMsg.originatorNodeId);  
  printf(" Decoded originatorEpochTime: %d\n", (int)decodedMsg.originatorEpochTime);  
  printf(" Decoded originatorLat: %f\n", (float)decodedMsg.originatorLat);  
  printf(" Decoded originatorLng: %f\n", (float)decodedMsg.originatorLng);  
  printf(" Decoded originatorBattLevel: %d\n", (int)decodedMsg.originatorBattLevel);  
  printf(" Decoded senderLat: %f\n", (float)decodedMsg.senderLat);  
  printf(" Decoded senderLng: %f\n", (float)decodedMsg.senderLng);  
  printf(" Decoded senderEpochTime: %d\n", (int)decodedMsg.senderEpochTime);
  printf(" Decoded senderTimeQuality: %d\n", (int)decodedMsg.senderTimeQuality);

  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();
  long freqErr = LoRa.packetFrequencyError();
  Serial.printf(" RSSI: %d. SNR: %f. Frequency error: %d.\n", rssi, snr, freqErr);
  // add those "reception quality" values to the struct  
  decodedMsg.receiverRssi = rssi;
  decodedMsg.has_receiverRssi = true;
  decodedMsg.receiverSnr = snr;
  decodedMsg.has_receiverSnr = true;
  decodedMsg.receiverFreqErr = freqErr;
  decodedMsg.has_receiverFreqErr = true;  

  decodedMsg.receiverNodeId = nodeNum;

  decodedMsg.receiverLat = gps.location.lat();
  decodedMsg.receiverLng = gps.location.lng();
  decodedMsg.receiverEpochTime = getCurrentSystemTimeAsEpoch();

  Serial.printf("pushing decoded msg to receivedMsgs queue\n");
  //receivedMsgs.push(decodedMsg);

  // if the sender sent a time of higher quality than ours, set /our/ system clock with it
  if( (int)decodedMsg.senderTimeQuality > getOurTimeQuality()) {
    Serial.printf("setting our clock to sender's higher quality clock. sender's quality is %d. our quality is only %d.\n", (int)decodedMsg.senderTimeQuality, getOurTimeQuality());
    setSystemTimeByEpoch((int)decodedMsg.senderEpochTime);
  } else {
    Serial.printf("ignoring sender's time\n");
  }
  
  //demonstrate our ability to calculate distance and course
  double distanceKm = gps.distanceBetween(
    gps.location.lat(),
    gps.location.lng(),
    (float)decodedMsg.senderLat,
    (float)decodedMsg.senderLng) / 1000.0;
  double courseTo = gps.courseTo(
    gps.location.lat(),
    gps.location.lng(),
    (float)decodedMsg.senderLat,
    (float)decodedMsg.senderLng);    
  Serial.print(" Distance (km) from sender: ");
  Serial.println(distanceKm);
  //Serial.print(" Course to sender: ");
  //Serial.println(courseTo);
  Serial.print(" Cardinal direction to sender: ");
  Serial.println(gps.cardinal(courseTo));      
}

void printBatteryLog() {
  File fileToRead = SPIFFS.open(BATT_LOG_FILENAME);
 
  if(!fileToRead){
    Serial.println("Failed to open battery log for reading");
    return;
  }
 
  Serial.println("Battery log:");
  while(fileToRead.available()) {
    Serial.write(fileToRead.read());
  }
  fileToRead.close();
}

void printMsgLog() {
  File fileToRead = SPIFFS.open(MSG_LOG_FILENAME);
 
  if(!fileToRead){
    Serial.println("Failed to open msg log for reading");
    return;
  }
 
  Serial.println("Msg log:");
  while(fileToRead.available()) {
    Serial.write(fileToRead.read());
  }
  fileToRead.close();
}

// 3rd party code ----------------------------------------------
void setSystemTimeByEpoch(time_t epoch) {
  timeval tv = { epoch, 0 };
  timezone tz = { TZ_MN + DST_MN, 0 };
  settimeofday(&tv, &tz);
}

time_t getCurrentSystemTimeAsEpoch() {
  time_t epoch;
  time(&epoch);
  return epoch;
}

void printCurrentSystemTimeYearMonthDayHourMinuteSecond() {
  struct tm now;
  getLocalTime(&now,0);
  Serial.println(&now, "%B %d %Y %H:%M:%S (%A)");
}

static time_t yearMonthDayHourMinuteSecondToEpoch(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second) {
    struct tm t = {0};
    t.tm_year = year - 1900; // 2019 is 119
    t.tm_mon = month - 1;    // 0-indexed
    t.tm_mday = day;
    t.tm_hour = hour;
    t.tm_min = minute;
    t.tm_sec = second;
    return mktime(&t);  // https://stackoverflow.com/a/11979667
}
// end 3rd party code ----------------------------------------------


void queueNewOutboundMsg() {
  uint8_t buffer[252]; // to store the results of the encoding process
  // create struct, populate ---------------------------------------------
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));  
  Msg locallyComposedMsg = Msg_init_zero;
  char content[] = "my locally-composed content";
  strncpy((char*)locallyComposedMsg.content, content, strlen(content));
  locallyComposedMsg.has_content = true;
  
  locallyComposedMsg.originatorNodeId = nodeNum;
  locallyComposedMsg.has_originatorNodeId = true;
  
  locallyComposedMsg.originatorEpochTime = getCurrentSystemTimeAsEpoch(); // we got 943920000 once (Tue 30 Nov 1999 12:00:00 AM UTC)
  locallyComposedMsg.has_originatorEpochTime = true;
  
  locallyComposedMsg.originatorBattLevel = getBatteryVoltageADCVal();
  locallyComposedMsg.has_originatorBattLevel = true;
  locallyComposedMsg.originatorLat = gps.location.lat();
  locallyComposedMsg.has_originatorLat = true;
  locallyComposedMsg.originatorLng = gps.location.lng();
  locallyComposedMsg.has_originatorLng = true;
  
  locallyComposedMsg.senderLat = gps.location.lat();
  locallyComposedMsg.has_senderLat = true;
  locallyComposedMsg.senderLng = gps.location.lng();
  locallyComposedMsg.has_senderLng = true;  
  locallyComposedMsg.senderEpochTime = getCurrentSystemTimeAsEpoch();
  locallyComposedMsg.has_senderEpochTime = true;
  locallyComposedMsg.senderTimeQuality = 80;  // 80 is "t-beam w/ gps fix"
  locallyComposedMsg.has_senderTimeQuality = true;
  
  // push ------------------------------------------
  outboundMsgs.push(locallyComposedMsg); // TODO: report if full
  Serial.printf("queued new outbound msg\n");
}

bool sendAnyQueuedMessages() {
  uint8_t buffer[252]; // to store the results of the encoding process

  if(outboundMsgs.count() > 0) {
    Msg locallyComposedMsg = Msg_init_zero;
    
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));  
    
    // pop -------------------------------------------
    Msg outboundMsg = outboundMsgs.pop(); // TODO: catch err
    
    // encode ----------------------------------------
    bool status = pb_encode(&stream, Msg_fields, &outboundMsg);
    if (!status) {
      Serial.printf(" Encoding FAILED: %s\n\n", PB_GET_ERROR(&stream));
      
      return false;
    }
   
    Serial.print(" Length of encoded message: ");
    Serial.println(stream.bytes_written);
   
    Serial.print(" Message: ");
    for( int i = 0; i<stream.bytes_written; i++ ) {
      Serial.printf("%02X", buffer[i]);
    }
    Serial.println("");
      
    // transmit --------------------------------------    
    LoRa.beginPacket();
    LoRa.write(buffer, stream.bytes_written);
    //Serial.printf(" calling LoRa.endPacket()... (we've seen that function 'hang' before. likely due to unsupported lora rf params)\n");
    LoRa.endPacket();
    //Serial.printf(" LoRa.endPacket() returned!\n");
    return true;
  }
}

void queueTest() {
  Serial.printf("queueTest() sentMsgs.count(): %d\n", sentMsgs.count());
  Msg msgFromQueue = sentMsgs.pop();
  printf("popped originatorNodeId: %d\n", (int)msgFromQueue.originatorNodeId);      
  Serial.printf("            sentMsgs.count(): %d\n", sentMsgs.count());
}


byte getBatteryVoltageADCVal() {
  // this code adapted from http://werner.rothschopf.net/201905_ttgo_t_beam.htm
  // when battery is absent we see...
  //  "D022 ADC Measurement: 559 591 601 586 571 Avg=581"
  //  "D022 ADC Measurement: 597 589 580 563 546 Avg=575"
  //  "D022 ADC Measurement: 551 537 544 581 604 Avg=563"

 

  // multisample ADC
  const byte NO_OF_SAMPLES = 5;
  uint32_t adc_reading = 0;
  //Serial.print(F(" Battery voltage ADC measurements:"));
  analogRead(ADCBOARDVOLTAGE_PIN); // First measurement has the biggest difference on my board, this line just skips the first measurement
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    uint16_t thisReading = analogRead(ADCBOARDVOLTAGE_PIN);
    adc_reading += thisReading;
    //Serial.print(F(" ")); 
    //Serial.print(thisReading);
  }
  adc_reading /= NO_OF_SAMPLES;
  //Serial.print(F(" Avg=")); 
  //Serial.println(adc_reading);
  // Convert ADC reading to voltage in deciVolt, 1024/2048/4096 not hardcoded but calculated depending on the set ADC_BITS
  byte voltage = adc_reading * 39 * 2 / (1 << ADC_BITS); // 3.9V because of 11dB, 100K/100K Voltage Divider, maxResolution (1024/2048/4096) 
  return voltage; 
  //BATT_ADC_VAL_TO_PCT_REMAIN
}


bool saveMsgsAndBattPctRemainingToFlash() {
  Serial.printf(" saveMsgsAndBattPctRemainingToFlash()\n");
  // write all Msg structs in the receivedMsgs and sentMsgs queues to the corresponding SPIFFS file
  File msgFileToAppend = SPIFFS.open(MSG_LOG_FILENAME, FILE_APPEND);
 
  if(!msgFileToAppend){
    Serial.println("There was an error opening the msg file for appending");
    return false;
  }
  Msg cur;
  while( receivedMsgs.count() > 0 ) {
    cur = receivedMsgs.pop();
    Serial.printf(" originatorNodeId of Msg popped from receivedMsgs queue: %d\n", cur.originatorNodeId);
    // TODO: encode protobuf, and hex-encode it
    uint8_t buffer[252]; // to store the results of the encoding process
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));  

    // encode ----------------------------------------
    bool status = pb_encode(&stream, Msg_fields, &cur);
    if (!status) {
      Serial.printf("Decoding FAILED: %s\n\n", PB_GET_ERROR(&stream));
      return false;
    }

    char temp[5] = "\x00\x00\x00\x00";
    Serial.print(" Length of encoded message: ");
    Serial.println(stream.bytes_written);
    char serializedHexEncodedMsg[512] = "";
    
    Serial.print(" Message: ");
    for( int i = 0; i<stream.bytes_written; i++ ) {
      Serial.printf("%02X", buffer[i]);
      sprintf(temp, "%02X", buffer[i]);
      strcat(serializedHexEncodedMsg, temp);      
    }  
    strcat(serializedHexEncodedMsg, "\n"); 
    
    if(msgFileToAppend.printf("%d, Compiled "__DATE__" "__TIME__", %s\n", nodeNum, serializedHexEncodedMsg)){ // TODO: use last ADC reading instead of reading it now
      Serial.println("Msg log was appended");
    } else {
      Serial.println("Msg log append failed");
    }    
  } 

  msgFileToAppend.close(); 
  
  // now write battery level to flash -------------------------------------------------
  File fileToAppend = SPIFFS.open(BATT_LOG_FILENAME, FILE_APPEND);
 
  if(!fileToAppend){
    Serial.println("There was an error opening the battery file for appending");
    return false;
  }

  int epochTime = getCurrentSystemTimeAsEpoch();
  
  if(fileToAppend.printf("%d, Compiled "__DATE__" "__TIME__", %d, %d\n", nodeNum, epochTime, getBatteryVoltageADCVal() )){ // TODO: use last ADC reading instead of reading it now
      Serial.println("Battery log was appended");
  } else {
      Serial.println("Battery log append failed");
  }
  fileToAppend.close();    
}

void handleNewUSBSerialCommand(String command) {
  if(command.equals(String("send\n"))) {
    Serial.println("Got 'send' cmd.");
    queueNewOutboundMsg(); 
   } else if(command.equals(String("printmsglog\n"))) {
    printMsgLog();      
  } else if(command.equals(String("printbattlog\n"))) {
    printBatteryLog();      
  } else {
    Serial.printf("Got unknown command: %s", command.c_str());
  }
}

void loop() {
  //Serial.printf("loop() %d\n", millis());
  
  //digitalWrite(LED_IO14_PIN_NUMBER, HIGH);  // "set GPIO16 low to reset OLED"
  //delay(500);   
  //digitalWrite(LED_IO14_PIN_NUMBER, LOW);

  
  // 1) Output any available data to OLED -------------------------------------------------
  if (millis() - lastOLEDUpdateTime > OLED_UPDATE_INTERVAL_MS) {
    updateOLED();
    lastOLEDUpdateTime = millis();
  }
  
  
  // N) Input any available data from GPS and parse NMEA sentences ------------------------
  attemptGpsNmeaDecode(1000); 

  if( gps.location.lat() == 0 && gps.location.lng() == 0) { //t gps.isValid() != true ) {
    //Serial.println(" Received invalid (lat=0 lng=0) data from GPS module"); // TODO: display this to oled for new devices
    sprintf(gpsTimeOLEDText, "NOLatLng: %02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    //Serial.println(" Received valid (lat!=0 lng!=0) data from GPS module");   
    // "for performance reasons %f is not included in the Arduino's implementation of sprintf()"

    //Serial.print(String("  "));   
    //Serial.println(String(gpsTimeOLEDText)); 
    ///Serial.printf("  GPS lat: %f lon: %f\n", gps.location.lat(), gps.location.lng()); 
  
    //Serial.println(gps.satellites.value());
    //Serial.print(gps.altitude.feet() / 3.2808);
  
    sprintf(gpsTimeOLEDText, "GPS: %02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());       
    //Serial.println(sec);
    //if( sec == 0 ) {
    //    Serial.println("GPS MINUTE BEGINS (or it was first GPS read?)");      
    //}    
  }
  

  // N) Input any available data from LoRa radio -------------------------------------------------
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    handleReceivedLoraPkt(packetSize); // decode to new stack struct, populate rssi+snr+freq_err
  } else {
    //Serial.printf(" no lora pkt rx'd\n");
  }
  
  // N) Input battery voltage value -----------------------------------------------------------------
  byte adcVal = getBatteryVoltageADCVal();
  sprintf(batteryVoltageOLEDText, "Battery: %d%%", BATT_ADC_VAL_TO_PCT_REMAIN[adcVal]); 

  // N) Input any available USB serial command -------------------------------------------------
  while(Serial.available() > 0) {
    char newChar = Serial.read();
    usbSerialCommand.concat(newChar);
    Serial.printf("%c", newChar); // echo the character
    if( newChar == '\n' ) {
      handleNewUSBSerialCommand(usbSerialCommand);
      usbSerialCommand = "";
    }
  }

  // save logs to flash every N minutes
  if (millis() - lastFlashWriteTime > FLASH_WRITE_INTERVAL_MS) {
    Serial.printf("We've reached a N-minute interval. Saving msgs and batt level to flash.\n");
    saveMsgsAndBattPctRemainingToFlash();   
    lastFlashWriteTime = millis();
  }

  // THINKME: worth doing this less often?
  setSystemTimeByEpoch(yearMonthDayHourMinuteSecondToEpoch(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second()));
  //Serial.printf("we just set system time by epoch to %d\n", yearMonthDayHourMinuteSecondToEpoch(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second()));
  //printCurrentSystemTimeYearMonthDayHourMinuteSecond();
  
  // transmit if we should
  sendAnyQueuedMessages();
}
