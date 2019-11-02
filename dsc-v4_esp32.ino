#include <pb_encode.h>
#include <pb_common.h>
#include <pb.h>
#include <pb_decode.h>
#include "Msg.pb.h"
#include "build-time_source_code_git_version.h"
#include <CircularBuffer.h>
#include <SPI.h>                        // used for connection to LoRa radio
#include <LoRa.h>                       // THINKME: 0.6.1 req'd?
#include "Queue.h" // copied this file to arduino-1.8.9/libraries/Queue from https://github.com/sdesalas/Arduino-Queue.h/blob/51d2d0c69f5c6d88997b4a900637fcf35294317d/Queue.h
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <time.h>
#include <sys/time.h>                   // struct timeval
#include <TinyGPS++.h>     
#include "SPIFFS.h"
#include "Config.h"
#include "Utils.h"

// If you're using the Arduino IDE, use Tools->Board->T-Beam
// Tested on TTGO T-Beam boards with silk-screened label: "T22_V07 20180711"


// Here's how to compile Msg.proto with nanopb on Ubuntu 18.04 into sketch-compatible format by following the guide at... https://github.com/vladimirvivien/iot-dev/tree/master/esp8266/esp8266-dht11-temp#install-and-configure-nanopb
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
//  python /usr/lib/python2.7/dist-packages/serial/tools/miniterm.py -e --eol LF /dev/ttyUSB0 115200


/***********
 * OLED Stats Display
 *********/
void updateOLED(void) {
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  char* dst = (char*) malloc(128);
  sprintf(dst, "DSCv4 %s:", GIT_REV);
  display.print(dst);
  free(dst);
  
  display.println(janky_version);

  display.drawRect(0,58,128,2,0);// Clear Previous Bar
  display.drawRect(0,58,BATT_ADC_VAL_TO_PCT_REMAIN[getBatteryVoltageADCVal()]*oled_x_scale,2,1); // Draw a new one!

  display.setCursor(0,10);            // Start at top-left corner
  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(WHITE);
  display.print(F("DSC")); 
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.print(F(" ID"));
  display.setTextSize(2); 
  display.print(nodeNum);
  display.setTextSize(1); 
  display.print("Nodes");
  display.setTextSize(2); 
  display.println(activeNodes);
  display.setTextSize(1); 
  display.print(F("tx:"));
  display.print(String(kbSent));
  display.print(F("   rx:"));
  display.println(String(kbRecv));

  struct tm now;
  getLocalTime(&now,0);
  display.println(&now, "%m/%d/%Y %H:%M:%S");
  display.print("Loop(ms):");
  display.print(loopPeriod);
  display.print(" Max:");
  display.println(loopPeriodMax);
  display.display();

}


void setup() {
  // Initialize UART ------------------------------------------------------------------------------
  Serial.begin(SERIAL_BAUDRATE);
  while(Serial.available() > 0) {
    char t = Serial.read();
  }

  Serial.println("BBQ Research DSCv4 firmware version " __VERSION__ " compiled on " __DATE__ " at " __TIME__); // TODO: use COMPILE_DATE
  
  // Initialize OLED ------------------------------------------------------------------------------
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // TODO: use constant for this i2c address
    if (op_mode == MODE_COMMAND) {
      Serial.println(F("SSD1306 allocation failed"));
    }
  }

  pinMode(BLUE_LED, OUTPUT);

  // Initialize GPS ------------------------------------------------------------------------------
  GPSSerial1.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_SERIAL_TX_PIN, GPS_SERIAL_RX_PIN);
  if (op_mode == MODE_COMMAND) Serial.println(" GPS initialized.");

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
  if (op_mode == MODE_COMMAND) Serial.println(" Battery voltage monitor initialized.");


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
    case 0xF4E0:
      nodeNum = 12;
      break;        
    default : /* Optional */
      Serial.println(" we're running on an unexpected esp32 device!");
      Serial.print("Chip ID: ");
      Serial.println((uint16_t)(chipid>>32));
      delay(25000);
  }  
  if (op_mode == MODE_COMMAND) Serial.printf(" Node ID (%d) (0x%04X) initialized.\n", nodeNum, (uint16_t)(chipid>>32));
  
  // Initialize LoRa -------------------------------------------------------------------------------
  LoRa.setPins(SS, RST, DI0);
  //LoRa.setFrequency(frequency);                               // "(433E6, 866E6, 915E6)" (we set the freq using LORA_BAND below)
  LoRa.setSpreadingFactor(7); 
  LoRa.setSignalBandwidth(125E3); 
  LoRa.setCodingRate4(5);
  //LoRa.setPreambleLength(8); 
  //LoRa.setSyncWord(0x12);                                     // "byte value to use as the sync word, defaults to 0x12"
  //LoRa.enableCrc(); //LoRa.disableCrc();                      // "by default a CRC is not used."
  //LoRa.enableInvertIQ(); //LoRa.disableInvertIQ();            // "by default a invertIQ is not used."
  //LoRa.setTxPower(txPower);                                   // "TX power in dB, defaults to 17"

  if (!LoRa.begin(LORA_BAND)) {
    Serial.println(" Starting LoRa failed!");
    Serial.flush();
    while (1);
  }
  if (op_mode == MODE_COMMAND) Serial.println(" LoRa intialized.");


  // Initialize SPIFFS -------------------------------------------------------------------------
  if (!SPIFFS.begin(true)) { // this can take 5+ sec on a new device
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }  
  if (op_mode == MODE_COMMAND) Serial.println(" SPIFFS intialized.");

  delay(200);
  Serial.flush();
  
  //Initialize Loop measurement
  lastLoopTime = millis();

  oledBitmapTest();
}

void handleNewUSBSerialCommand(String command) {
  if(command.equals(String("/send"))) {
    Serial.println("Got 'send' cmd.");
    queueNewOutboundMsg("TEST"); 
  } else if(command.equals(String("/dump msglog"))) {
    printMsgLog();      
  } else if(command.equals(String("/dump battlog"))) {
    printBatteryLog();      
  } else if(command.equals(String("/removelogs"))) {
    removeLogs();     
  } else if(command.equals(String("/date"))) {
    printSystemTime();      
  } else if(command.startsWith(String("/lora freq "))) {
    //** TODO Calculate valid range based on bandwidth
    //** (902000000 + bw/2) through (928000000 - bw/2)
    int freq = command.substring(11).toInt();
    if (freq >= 902000000 && freq <= 928000000) {
      Serial.println("set frequency");
      LoRa.setFrequency(freq);
    }
    else {
      Serial.println("frequency out of range.");
    }
  } else if(command.startsWith(String("/lora bw "))) {
    int bw = command.substring(9).toInt();
    if (bw == 7800 || bw == 10400 || bw == 15600 || bw == 20800 || bw == 31250 
                   || bw == 41700 || bw == 62500 || bw == 125000 || bw == 250000) {
      Serial.println("set bandwidth");
      LoRa.setSignalBandwidth(bw);
    }
    else {
      Serial.println("invalid bandwidth.");
    }     
  } else if(command.startsWith(String("/lora cr "))) {
    int cr = command.substring(9).toInt();
    if (cr >= 5 && cr <= 8) {
      Serial.println("set coding rate");
      LoRa.setCodingRate4(cr);
    }
    else {
      Serial.println("invalid coding rate");
    }     
  } else if(command.startsWith(String("/lora sf "))) {
    int sf = command.substring(9).toInt();
    if (sf >=6 && sf <= 12) {
      Serial.println("set spread factor");
      LoRa.setSpreadingFactor(sf);
    }
    else {
      Serial.println("invalid coding rate.");
    }         
  } else if(command.equals(String("/testmode on"))) {
    transmit_loop = true;
    Serial.println("test mode enabled");    
  } else if(command.equals(String("/testmode off"))) {
    transmit_loop = false;
    Serial.println("test mode disabled");
  } else if(command.equals(String("/debug on"))) {
    debug_mode = true;
    Serial.println("debug mode enabled");
  } else if(command.equals(String("/debug off"))) {
    debug_mode = false;
    Serial.println("debug mode disabled");
  } else if(command.equals(String("/transparent on"))) {
    op_mode = MODE_PROMISCUOUS;
    Serial.println("transparent serial mode enabled");    
  } else if(command.equals(String("/help"))) {
    Serial.println("DSC Mesh Router Help");
    Serial.println("------ general            ----------------------------");
    Serial.println("/help                     help me!");
    Serial.println("/debug on                 enable debug output");
    Serial.println("/debug off                disable debug output");
    Serial.println("/dump msglog              show message logs");
    Serial.println("/dump battlog             show battery logs");
    Serial.println("/removelogs               remove logs from flash");        
    Serial.println("/date                     system datetime");
    Serial.println("/transparent on           enable transparent serial mode ( ~~~+++ to disable )");
    Serial.println("------ lora radio params  -----------------------------");
    Serial.println("/lora freq 9150000000     set lora frequency");
    Serial.println("                            valid (902000000 + bw/2) - (928000000 - bw/2) (U.S.)");
    Serial.println("/lora bw 250000           set lora bandwidth");
    Serial.println("                            valid value: 7800,10400,15600,20800,31250,41700,62500,125000,250000");
    Serial.println("/lora sf 7                set lora spreading factor");
    Serial.println("                            valid 6,7,8,9,10,11,12");
    Serial.println("/lora cr 8                set lora coding rate");
    Serial.println("                            valid 5,6,7,8  (4/5,4/6,4/7,4/8)");
    Serial.println("------ testing            -----------------------------");
    Serial.printf( "/testmode on              enable %d ms transmit loop (enabled by default)\n", TESTING_TRANSMIT_LOOP_INTERVAL_MS); 
    Serial.println("/testmode off             disable transmit loop"); 
  } else {
    Serial.printf("Got unknown command: %s\n", command.c_str());
  }
}

void handleReceivedLoraPkt(int packetSize){
  kbRecv += packetSize / 1000.0;    // unit kB
 
  if (debug_mode) {
    // received a packet
    Serial.printf("Received LoRa packet of size %d\n", packetSize);
  }

  if (op_mode == MODE_PROMISCUOUS) {
    String incoming = "";
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }
    Serial.print(incoming);
  } else {
    uint8_t incomingPkt[256];// for the results of protobuf decoding
    int i = 0;
    while (LoRa.available()) {
      char c = (char)LoRa.read();
      incomingPkt[i] = c;
      i += 1;
    }
   
    Msg decodedMsg = Msg_init_zero;
    pb_istream_t istream = pb_istream_from_buffer(incomingPkt, packetSize);
    bool status = pb_decode(&istream, Msg_fields, &decodedMsg);
    if (!status && debug_mode) {
      printf("Decoding FAILED: %s\n\n", PB_GET_ERROR(&istream));
      printf("Size: %d\n", i);
      printf("Data: %s\n", incomingPkt);
      return;
    }   
    
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();
    long freqErr = LoRa.packetFrequencyError();

    if (debug_mode) {
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

      Serial.printf(" RSSI: %d. SNR: %f. Frequency error: %d.\n", rssi, snr, freqErr);
    } 
    else {
      if (op_mode == MODE_COMMAND) {
        Serial.printf("Received a msg from node ID %d: '%s'\n", (int)decodedMsg.originatorNodeId, (char*)decodedMsg.content);
      } else if (op_mode == MODE_PROMISCUOUS) {
        Serial.print((char*)decodedMsg.content);
      }
    }
    
    nodeLastEpoch[(int)decodedMsg.originatorNodeId] = millis();

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

    //Serial.printf("pushing decoded msg to receivedMsgs queue\n");
    //receivedMsgs.push(decodedMsg);

    // if the sender sent a time of higher quality than ours, set /our/ system clock with it
    if( (int)decodedMsg.senderTimeQuality > quality_time) {
      if (debug_mode){
        quality_time = 50; // Arbitrary need to define quality conditions and possibly a decay function
        Serial.printf("setting our clock to sender's higher quality clock. sender's quality is %d. our quality is only %d.\n", (int)decodedMsg.senderTimeQuality, getOurTimeQuality());  
        Serial.println("Syncronizing Clock from Peer");
      }
      setSystemTimeByEpoch((int)decodedMsg.senderEpochTime);
    } else {
      if (debug_mode) {
        Serial.printf("ignoring sender's time\n");
      }
      
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
    if (debug_mode) {
      Serial.print(" Distance (km) from sender: ");
      Serial.println(distanceKm);
      //Serial.print(" Course to sender: ");
      //Serial.println(courseTo);
      Serial.print(" Cardinal direction to sender: ");
      Serial.println(gps.cardinal(courseTo));      
    }
  }
}

void queueNewOutboundMsg(char content[]) {
  uint8_t buffer[252]; // to store the results of the encoding process
  // create struct, populate ---------------------------------------------
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));  
  Msg locallyComposedMsg = Msg_init_zero;
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
  locallyComposedMsg.senderTimeQuality = quality_time;
  locallyComposedMsg.has_senderTimeQuality = true;
  
  // push ------------------------------------------
  outboundMsgs.push(locallyComposedMsg); // TODO: report if full
  if (debug_mode) {
    Serial.printf("queued new outbound msg\n");
  }
}

bool sendAnyQueuedMessages() {
  if (carrier_detect) {
    return false;
  }
  if(outboundMsgs.count() > 0) {
    uint8_t buffer[252]; // to store the results of the encoding process
    Msg outboundMsg = outboundMsgs.pop(); // TODO: catch err
    
    // encode ----------------------------------------
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));  
    bool status = pb_encode(&stream, Msg_fields, &outboundMsg);
    if (!status) {
      Serial.printf(" Encoding FAILED: %s\n\n", PB_GET_ERROR(&stream));
      
      return false;
    }
    
    if (debug_mode) {
      Serial.print(" Length of encoded message: ");
      Serial.println(stream.bytes_written);
     
      Serial.print(" Message: ");
      for( int i = 0; i<stream.bytes_written; i++ ) {
        Serial.printf("%02X", buffer[i]);
      }
      Serial.println("");
    }

    // transmit --------------------------------------    
    LoRa.beginPacket();
    LoRa.write(buffer, stream.bytes_written);
    LoRa.endPacket(true);         // ASYNC true (non-blocking)
    kbSent += stream.bytes_written / 1000.0;
    blue_led(true);
    return true;
  }
}



void loop() {

  /*
  * Calculate Loop Period Stats
  */
  loopPeriod = millis() - lastLoopTime;
  lastLoopTime = millis();
  if (loopPeriod > loopPeriodMax) {
    loopPeriodMax = loopPeriod;
  }


  /*
  ** Crude neighbor node awareness.
  */
  if (millis() - lastCheckActiveTime > checkActiveTime) {
    lastCheckActiveTime = millis();
    //Serial.println(lastCheckActiveTime);
    int totalActive = 0;
    for (int i=0; i<16;i++) { //Check node activity (max nodes 16 hardcoded)
      if (millis() - nodeLastEpoch[i] < timeNodeOutOfRange)
      {
        if (nodeLastEpoch[i] != 0) {
          totalActive++;
        }
      }
    }
    activeNodes = totalActive;
  }

  /*
  ** If enabled, periodically send a message (Generate traffic for testing)
  */
  if (transmit_loop && op_mode == MODE_COMMAND) {
    if (millis() - lastTransmitLoop > TESTING_TRANSMIT_LOOP_INTERVAL_MS) {
      lastTransmitLoop = millis();
      queueNewOutboundMsg("transmit loop test msg");
    }    
  }
  
  /*
  **  Update OLED Display
  */
  if (millis() - lastOLEDUpdateTime > OLED_UPDATE_INTERVAL_MS) {
    updateOLED();
    lastOLEDUpdateTime = millis();
  }
  
  /*
  **  Update Status LED
  */
  if (millis() - lastLEDUpdateTime > lastLEDFlashTime && isBlueLED) {
    blue_led(false);
  }
  
  /*
  ** Input any available data from GPS and parse NMEA sentences
  */
  updateGPS();

  /*
  ** Handle Inbound LoRa packets
  */ 
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    handleReceivedLoraPkt(packetSize);        // decode to new stack struct, populate rssi+snr+freq_err
    carrier_detect = true;
    carrier_timeout = 1000 + random(1000);    // CSMA (Helps Prevent Collisions)
    lastCarrierTime = millis();
    blue_led(true);
  } 

  /*
  ** Basic Carrier Detection Timeout (Have not seen a packet from others)
  */
  if (millis() - lastCarrierTime >  carrier_timeout) {
    carrier_detect = false;
  }

  /*
  ** Check incoming serial buffer for data to process
  */
  while (Serial.available() > 0) {
    char ch = Serial.read();

    if (op_mode == MODE_PROMISCUOUS) {
      serial_queue.push(ch);
      if (transparentEscape.isFull()) {
        transparentEscape.shift();
      }
      transparentEscape.push(ch);
      String checkEscape = "";
      for (int i=0; i<transparentEscape.size(); i++) {
        checkEscape += transparentEscape[i];
      }
      if (checkEscape.equals(escape_str)) {
        op_mode = MODE_COMMAND;
        transparentEscape.clear();
        Serial.println("transparent serial mode disabled");    
      }
    }
    else {
      if (ch == '\n' || ch == '\r') {
        if (serial_queue[0] == '/') {                       // Possible Command
          int ndx=0;
          while (!serial_queue.isEmpty()) {
            outboundSerial[ndx] += serial_queue.shift();    // Shift element from queue (FIFO)
            ndx++;
            if (ndx > outboundSerialMax-1) break;           // Leave room for the NULL termination!
          }
          handleNewUSBSerialCommand(outboundSerial);
          for (int i=0; i<outboundSerialMax; i++) {
             outboundSerial[i] = '\0';
          }
          serial_queue.clear();
        }
        else {
          int ndx=0;
          while (!serial_queue.isEmpty()) {
            outboundSerial[ndx] += serial_queue.shift();    // Shift element from queue (FIFO)
            ndx++;
            if (ndx > outboundSerialMax-1) break;           // Leave room for the NULL termination!
          }
          outboundSerial[ndx+1] = '\0';                     // NULL terminate outbound char array

          queueNewOutboundMsg(outboundSerial);              // ship it!
          
          for (int i=0; i<outboundSerialMax; i++) {
             outboundSerial[i] = '\0';
          }

          serial_queue.clear();
          break;
        }
        break;
      }
      else {
        serial_queue.push(ch);
      }
    }
  }

  /*
  **  Periodically save logs to flash
  */
  if (millis() - lastFlashWriteTime > FLASH_WRITE_INTERVAL_MS) {
    if (op_mode == MODE_COMMAND && debug_mode) {
      Serial.printf("We've reached a %d-minute interval. Saving received/sent msgs and batt level to flash.\n", FLASH_WRITE_INTERVAL_MS/1000/60);
    }
    saveMsgsAndBattPctRemainingToFlash();   
    lastFlashWriteTime = millis();
  }

  //Serial.printf("we just set system time by epoch to %d\n", yearMonthDayHourMinuteSecondToEpoch(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second()));
  //printCurrentSystemTimeYearMonthDayHourMinuteSecond();
  
  /*
  ** Transmit Queued data 
  */
  if (millis() - lastSendTime > TRANSMIT_INTERVAL_MS && !carrier_detect) {
    lastSendTime = millis();

    if (op_mode == MODE_PROMISCUOUS) {
      int ndx=0;
      if (!serial_queue.isEmpty()) {
        if (LoRa.beginPacket()) {
          while (!serial_queue.isEmpty()) {
            outboundTransparentSerial[ndx] = serial_queue.shift();  // Shift element from queue (FIFO)
            ndx++;
            if (ndx > outboundTransparentMax-1) break;               // Leave room for the NULL termination!
          }

          if (ndx != 0) {
            outboundTransparentSerial[ndx+1] = '\0';
            LoRa.print(outboundTransparentSerial);
            kbSent += ndx+1 / 1000.0;   //unit kB
            blue_led(true);
            
            TRANSMIT_INTERVAL_MS = 25 + symbolRate * (ndx+1);
            
            for (int i=0; i<outboundTransparentMax; i++) {
               outboundTransparentSerial[i] = '\0';
            }
          }
          LoRa.endPacket(true);         // ASYNC (non-blocking)
        }
        else {
          Serial.println("not ready to send");  // if get here our symbol rate is too small
        }
      }
    }
    else {
      TRANSMIT_INTERVAL_MS = 250;          // send interval in milliseconds
      sendAnyQueuedMessages();
    }
  }
}