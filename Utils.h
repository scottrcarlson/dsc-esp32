
static void attemptGpsNmeaDecode(unsigned long ms) {   // TODO: how much time is required to read all avail data from gps             
  //TODO Configure ublox chip to only send the sentences that we use, this will make this function execute less. 

  unsigned long start = millis();
  //do {
    while (GPSSerial1.available()) {
      gps.encode(GPSSerial1.read());
    }
  //} while (millis() - start < ms); // this sets sets an upper limit on time consumed
  
  if (millis() > 5000 && gps.charsProcessed() < 10) { // TODO: use constants
    //if (op_mode == MODE_COMMAND) Serial.println(F("\nwe've received <10 chars from GPS and we booted >5sec ago - check gps wiring!"));
  }   
}

int getOurTimeQuality() {
  return quality_time;  
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


void removeLogs() {
    Serial.printf("Removing log files...\n");      
    SPIFFS.remove(MSG_LOG_FILENAME);
    SPIFFS.remove(BATT_LOG_FILENAME);
    Serial.printf(" Log files removed.\n");    
}

// 3rd party code (eat it)----------------------------------------------
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


void printSystemTime() {
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


/*
 * Blue LED Control
 */
void blue_led(bool state) {
  if (state) {
    //Turn on LED
    digitalWrite(BLUE_LED, HIGH);
    isBlueLED = true;
    lastLEDUpdateTime = millis();
  }
  else {
    //Turn off LED
    isBlueLED = false;  
    digitalWrite(BLUE_LED, LOW);
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
  if (op_mode == MODE_COMMAND && debug_mode) {
    Serial.printf(" saveMsgsAndBattPctRemainingToFlash()\n");
  }

  if (op_mode == MODE_COMMAND) {
    // write all Msg structs in the receivedMsgs and sentMsgs queues to the corresponding SPIFFS files
    File msgFileToAppend = SPIFFS.open(MSG_LOG_FILENAME, FILE_APPEND);
   
    if(!msgFileToAppend){
      if (op_mode == MODE_COMMAND) {
        Serial.println("There was an error opening the msg file for appending");
      }
      return false;
    }
    Msg cur;
    while( receivedMsgs.count() > 0 ) {
      cur = receivedMsgs.pop();
      if (op_mode == MODE_COMMAND && debug_mode) Serial.printf(" originatorNodeId of Msg popped from receivedMsgs queue: %d\n", cur.originatorNodeId);
      // encode protobuf, then hex-encode that to save to the flash file
      uint8_t buffer[1024]; // to store the results of the encoding process
      pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));  

      // encode ----------------------------------------
      bool status = pb_encode(&stream, Msg_fields, &cur);
      if (!status) {
        if (op_mode == MODE_COMMAND) Serial.printf("Encoding FAILED: %s\n\n", PB_GET_ERROR(&stream));
        return false;
      }

      char temp[5] = "\x00\x00\x00\x00";
      if (debug_mode) {
        Serial.print(" Length of encoded message: ");
        Serial.println(stream.bytes_written);
      }
      char serializedHexEncodedMsg[512] = "";
      
      //Serial.print(" Message: ");
      for( int i = 0; i<stream.bytes_written; i++ ) {
        //Serial.printf("%02X", buffer[i]);
        sprintf(temp, "%02X", buffer[i]);
        strcat(serializedHexEncodedMsg, temp);      
      }  
      if (debug_mode) {
        Serial.println("");
      }
      strcat(serializedHexEncodedMsg, "\n"); 
      
      if(msgFileToAppend.printf("%d, Compiled "__DATE__" "__TIME__", %s\n", nodeNum, serializedHexEncodedMsg)){
        if (debug_mode) Serial.println("Msg log was appended");
      } else {
        if (op_mode == MODE_COMMAND) Serial.println("Msg log append failed");
      }    
    } 
    // TODO: save /sent/ messages to flash as well

    msgFileToAppend.close(); 
  }  
  // now write battery level to flash -------------------------------------------------
  File battFileToAppend = SPIFFS.open(BATT_LOG_FILENAME, FILE_APPEND);
 
  if(!battFileToAppend){
    if (op_mode == MODE_COMMAND) Serial.println("There was an error opening the battery file for appending");
    return false;
  }

  int epochTime = getCurrentSystemTimeAsEpoch();
  if(battFileToAppend.printf("%d, Compiled "__DATE__" "__TIME__", %d, %d\n", nodeNum, epochTime, getBatteryVoltageADCVal() )){ // TODO: use last ADC reading instead of reading it now
      if (debug_mode) {
        Serial.println("Battery log was appended");
      }
  } else {
      if (op_mode == MODE_COMMAND) {
        Serial.println("Battery log append failed");
      }
  }
  battFileToAppend.close();  
}


static void updateGPS() { // Call this frequently.      
    while (GPSSerial1.available()) {
      gps.encode(GPSSerial1.read());

    }
   if (gps.time.isUpdated() && gps.date.isUpdated()) {
      uint8_t hour = gps.time.hour();
      uint8_t minute = gps.time.minute();
      uint8_t second = gps.time.second();
      uint16_t year = gps.date.year();
      uint8_t month = gps.date.month();
      uint8_t day = gps.date.day();

      if (month != 0 && second != last_gps_second) {
        last_gps_second = second;
        time_t epoch = yearMonthDayHourMinuteSecondToEpoch(year, month, day, hour, minute, second);
        setSystemTimeByEpoch(epoch);
        gps_valid = true;
        quality_time = 100;
      }
      else {
        if (gps_valid) {
          quality_time = 90;
        }
        gps_valid = false;
      }
    }
  attemptGpsNmeaDecode(1000); 
}


void oledBitmapTest(){
  display.clearDisplay();
  display.drawBitmap(0, 0, logo, 128, 64, 1);
  display.display();
  delay(2000);

}