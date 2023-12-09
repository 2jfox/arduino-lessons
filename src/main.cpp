#include <Arduino.h>
#include "header.h"

void setup() 
{
#ifdef DEBUG
  debug_init();	// initialize the debugger  
#else
  Serial.begin(115200);
  while (!Serial);// wait for serial port to connect. Needed for native USB
  // Just to know which program is running on my Arduino
  Serial.println(F("START " __FILE__ " from " __DATE__));
#endif

  setupL04();
  setupL05();
  setupL06();
  setupL07();
  setupL08();
  setupL09();
  setupL10();
  setupL11();
  setupL12();
  setupL13();
  setupL14();
  setupL15();
  setupL16();
  setupL17();
  setupL18();
  setupL19();
  setupL20();
  setupL21();
  setupL22();

  setupL34();
}

void loop() 
{  
  loopL04();
  loopL05();
  loopL06();
  loopL07();
  loopL08();
  loopL09();
  loopL10();
  loopL11();
  loopL12();
  loopL13();
  loopL14();
  loopL15();
  loopL16();
  loopL17();
  loopL18();
  loopL19();
  loopL20();
  loopL21();
  loopL22();

  loopL34();
}

/******************************************** SETUP *****************************************/

/*RGB LED */
void setupL04(){
#ifdef L04
    //DDRH = 0b00111000; //set 6,7,8 OUTPUT
    pinMode(rLedPin, OUTPUT);
    pinMode(gLedPin, OUTPUT);
    pinMode(bLedPin, OUTPUT);
    
    //PORTH = 1<<3;//0b00001000; //set RED HIGHT
    digitalWrite(rLedPin, HIGH);
    digitalWrite(gLedPin, LOW);
    digitalWrite(bLedPin, LOW); 
#endif
}

/*Buttons */
void setupL05(){
#ifdef L05
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  btn1Status = !digitalRead(BTN1_PIN); //HIGH by default because INPUT_PULLUP
  btn2Status = !digitalRead(BTN2_PIN); //HIGH by default because INPUT_PULLUP
  digitalWrite(LED_BUILTIN, btn1Status);
  digitalWrite(LED_PIN, btn1Status);
#endif
}

/*Buzzer Active */
void setupL06(){
#ifdef L06 
  pinMode(BZR_PIN, OUTPUT);
#endif
}

/*Buzzer Passive */
void setupL07(){
#ifdef L07 
//do nothing. all menagement on the 'tone' side 
#endif
}

/*Tilt Ball Switch */
void setupL08(){
#ifdef L08 
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TBS_PIN, INPUT);
  digitalWrite(TBS_PIN, HIGH);
#endif
}

/*SG90 (Servo) */
void setupL09(){
#ifdef L09 
  mySrv.attach(SRV_PIN);
  mySrv.write(srvPos);
#endif
}

/*SR04 (Ultrasonic Sensor) */
void setupL10(){
#ifdef L10 
  HCSR04.begin(TRIG_PIN, ECHO_PIN);
#endif
}

/*Membrane Switch Module */
void setupL11(){
#ifdef L11 
#ifndef DEBUG
  Serial.println(F("initialize 4 digits code"));        
#endif
  while(codeCorrect[0] == '\0' || codeCorrect[1] == '\0' || codeCorrect[2] == '\0' || codeCorrect[3] == '\0'){      
    char userKey = myKeypad.getKey();
    if(!userKey) continue;
    codeCorrect[codeIndex++] = userKey;
    Serial.print(F("*"));           
  }
  codeIndex = 0;
#ifndef DEBUG
  Serial.println();
  Serial.print(F("code initialized: "));
  Serial.println(codeCorrect);
#endif
#endif
}

/*DHT11 Temperature and Humidity Sensor) */
void setupL12(){
#ifdef L12 
  // Initialize device.
  dht.begin();  
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
#ifndef DEBUG
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
#endif
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
#endif
}

/*Analog Joystick Module */
void setupL13(){
#ifdef L13 
  pinMode(AJM_S_PIN, INPUT);
  digitalWrite(AJM_S_PIN, HIGH);
#endif
}

/*IR Receiver Module */
void setupL14(){
#ifdef L14 
  //Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));
  Serial.println(F("Using library version " VERSION_IRREMOTE));
  // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);

  Serial.print(F("Ready to receive IR signals of protocols: "));
  printActiveIRProtocols(&Serial);
  Serial.println();
  //Serial.println(F("at pin " STR(IR_PIN)));
#endif
}

/* SETUP L15 (MAX7219 LED Dot Matrix Module)*/
void setupL15(){
#ifdef L15 
  u8g2.begin();
  u8g2.setFont(u8g2_font_amstrad_cpc_extended_8r);	// https://github.com/olikraus/u8g2/wiki/u8g2reference#setfont
#ifndef DEBUG
  Serial.print(F("Display height: ")); Serial.println(u8g2.getDisplayHeight());
  Serial.print(F("Display width: ")); Serial.println(u8g2.getDisplayWidth());
  Serial.print(F("Max char height: ")); Serial.println(u8g2.getMaxCharHeight());
  Serial.print(F("Max char width: ")); Serial.println(u8g2.getMaxCharHeight());
  Serial.print(F("Height: ")); Serial.println(u8g2.getHeight());
  Serial.print(F("Width: ")); Serial.println(u8g2.getWidth());
  Serial.print(F("Rows: ")); Serial.println(u8g2.getRows());
  Serial.print(F("Cols: ")); Serial.println(u8g2.getCols());
#endif
#endif
}

/* SETUP L16 (MPU-6050 (GY-521 Module))*/
void setupL16(){
#ifdef L16 
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    /* initialize serial communication
    (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    it's really up to you depending on your project)*/
    //Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    mpu6050.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu6050.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */

    // configure Arduino LED pin for output
    pinMode(LED_BUILTIN, OUTPUT);
#endif
}

/*HC-SR501 PIR Sensor*/
void setupL17(){
#ifdef L17 
  pinMode(RIP_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
#endif
}

/*Water Level Detection Sensor Module*/
void setupL18(){
#ifdef L18
  //nothing to setup
#endif  
}

/*DS1307 RTC (Real Time Clock Module)*/
void setupL19(void){
#ifdef L19 
#ifndef ESP8266
  while (!Serial); // wait for serial port to connect. Needed for native USB
#endif
  
  //rtc.adjust(DateTime(2023, 11, 10, 11, 0, 0));

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
#endif
}

/*DS1307 RTC (Real Time Clock Module)*/
void setupL20(void){
#ifdef L20
  pinMode(SSM_AO_PIN, INPUT);
  pinMode(SSM_DO_PIN, INPUT);
  digitalWrite(SSM_DO_PIN, LOW);
#endif  
}

/*RC522 RFID Module*/
void setupL21(void){
#ifdef L21
	while (!Serial);		// Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
	SPI.begin();			// Init SPI bus
	mfrc522.PCD_Init();		// Init MFRC522
	delay(4);				// Optional delay. Some board do need more time after init to be ready
	mfrc522.PCD_DumpVersionToSerial();	// Show details of PCD - MFRC522 Card Reader details	
#endif  
}

/*LCD1602 (LCD Display)*/
void setupL22(void){
#ifdef L22
	
#endif  
}

/*7941E 3Pin 125KHz (TZT RFID UART Reader Wireless Module) */
void setupL34(void){
#ifdef L34
#ifndef RFID2
  gwiot7941e.begin(RFID_READER_RX_PIN);
#else
  //pinMode(RFID_READER_RX_PIN, INPUT);
  if (!stream_) {
    stream_ = softwareSerial_ = new SoftwareSerial(RFID_READER_RX_PIN, RFID_READER_TX_PIN);
    softwareSerial_->begin(GWIOT_7941E_BAUDRATE);
    }
  softwareSerial_->begin(GWIOT_7941E_BAUDRATE);
  stream_->setTimeout(GWIOT_7941E_READ_TIMEOUT);
#endif
#endif
}

/******************************************** LOOP *****************************************/

/*RGB LED */
void loopL04(){
#ifdef L04
  for(int i = 0; i < 255; i++)
  {         
    analogWrite(rLedPin, 255-i);
    analogWrite(gLedPin, i);
    //analogWrite(bLedPin, 255-i);        
    delay(10);
  }        
        
  for(int i = 0; i < 255; i++)
  {         
    //analogWrite(rLedPin, 255-i);
    analogWrite(gLedPin, 255-i);
    analogWrite(bLedPin, i);        
    delay(10);
  } 
    
  for(int i = 0; i < 255; i++)
  {         
    analogWrite(rLedPin, i);
    //analogWrite(gLedPin, 255-i);
    analogWrite(bLedPin, 255-i);        
    delay(10);
  } 
#endif
}

/*Buttons */
void loopL05(){
#ifdef L05
  if(!digitalRead(BTN1_PIN)) digitalWrite(LED_BUILTIN, btn1Status = !btn1Status);
  if(!digitalRead(BTN2_PIN)) digitalWrite(LED_BUILTIN, btn2Status = !btn2Status);
  delay(150); 
#endif
}

/*Buzzer Active */
void loopL06(){
#ifdef L06
  if(btn1Status){
    if (!anyrtttl::nonblocking::isPlaying()){
      if (songIndex == 0)
       anyrtttl::nonblocking::beginProgMem(BZR_PIN, tetris);
      else if (songIndex == 1)
        anyrtttl::nonblocking::begin_P(BZR_PIN, arkanoid);
      else if (songIndex == 2)          
        anyrtttl::nonblocking::begin_P(BZR_PIN, mario);//anyrtttl::nonblocking::begin(bzrPin, FPSTR(mario));
      else if (songIndex == 3)
        anyrtttl::nonblocking::begin(BZR_PIN, F("Bond:d=4,o=5,b=80:32p,16c#6,32d#6,32d#6,16d#6,8d#6,16c#6,16c#6,16c#6,16c#6,32e6,32e6,16e6,8e6,16d#6,16d#6,16d#6,16c#6,32d#6,32d#6,16d#6,8d#6,16c#6,16c#6,16c#6,16c#6,32e6,32e6,16e6,8e6,16d#6,16d6,16c#6,16c#7,c.7,16g#6,16f#6,g#.6"));      
      songIndex++;
    }
    else{
      anyrtttl::nonblocking::play();
      //anyrtttl::blocking::play(bzrPin, tetris);
    }    
  }
  else if(btn2Status){
    unsigned char i;
    for(int j = 0; j < 3; j++){
      //output an frequency
      for(i=0;i<80;i++){
        digitalWrite(BZR_PIN,HIGH);
        delay(1);//wait for 1ms
        digitalWrite(BZR_PIN,LOW);
        delay(1);//wait for 1ms
      }
      //output another frequency
      for(i=0;i<100;i++){
        digitalWrite(BZR_PIN,HIGH);
        delay(2);//wait for 2ms
        digitalWrite(BZR_PIN,LOW);
        delay(2);//wait for 2ms
      }
      if(!btn2Status) break;
    }
  }
#endif
}

/*Buzzer Passive */
void loopL07(){
#ifdef L07
  if(!btn1Status) return;
  for (int thisNote = 0; thisNote < 8; thisNote++) {
    tone(BZR_PIN, bzrPassMelody[thisNote], 200);    
    delay(300);
    if(!btn1Status) break;
  }  
#endif
}

/*Tilt Ball Switch */
void loopL08(){
#ifdef L08
  digitalWrite(LED_BUILTIN, !digitalRead(TBS_PIN));
  delay(1000);
#endif
}

/*Servo SG90 */
void loopL09(){
#ifdef L09
  if(btn1Status) return; 
  if(srvPos >= 180){
    srvPos = 0;
    mySrv.write(srvPos);
    delay(500);
  }
  mySrv.write(srvPos += 10);
  delay(250);  
#endif
}

/*Ultrasonic Sensor (SR04) */
void loopL10(){
#ifdef L10
  if(btn1Status) return;
  double* distances = HCSR04.measureDistanceCm();
#ifndef DEBUG
  Serial.println(distances[0]);//sm   
#endif
  delay(500);  
#endif
}

/*Membrane Switch Module */
void loopL11(){
#ifdef L11
  char userKey = myKeypad.getKey();    
  if (userKey){
#ifdef DEBUG
    breakpoint();
#endif
#ifndef DEBUG
    Serial.print(userKey);
#endif

    codeUser[codeIndex] = userKey;
    codeIndex++;    

    if(codeIndex == 4){
      //check code      
      if (strcmp(codeCorrect, codeUser) == 0){
#ifndef DEBUG        
        Serial.println(F(" unlocked"));
#endif
      }
      else{
#ifndef DEBUG
        Serial.println(F(" incorrect code. system locked"));
#endif
      }
      codeIndex = 0;
    }
  }
#endif
}

/*DHT11 Temperature and HumiditySensor */
void loopL12(){
#ifdef L12
  if(!btn1Status) return;

  // Delay between measurements.
  delay(delayMS);
#ifdef DEBUG
  breakpoint();
#endif
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
#ifndef DEBUG
    Serial.println(F("Error reading temperature!"));
#endif
  }
  else {
#ifndef DEBUG
    Serial.print(F("(°C): "));
    Serial.println(event.temperature);
#endif
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
#ifndef DEBUG
    Serial.println(F("Error reading humidity!"));
#endif
  }
  else {
#ifndef DEBUG
    Serial.print(F("( %): "));
    Serial.println(event.relative_humidity);
#endif
  }  
#endif
}

/*Analog Joystick Module */
void loopL13(){
#ifdef L13
  int valS = digitalRead(AJM_S_PIN);
  int valX = digitalRead(AJM_X_PIN);
  int valY = digitalRead(AJM_Y_PIN);
#ifndef DEBUG
  Serial.print(F("S: "));
  Serial.print(valS);
  Serial.print(F(", X: "));
  Serial.print(valX);
  Serial.print(F(", Y: "));
  Serial.println(valY);
#endif
  delay(500);
#endif
}

/*IR Receiver Module */
void loopL14(){
#ifdef L14
    /*
     * Check if received data is available and if yes, try to decode it.
     * Decoded result is in the IrReceiver.decodedIRData structure.
     *
     * E.g. command is in IrReceiver.decodedIRData.command
     * address is in command is in IrReceiver.decodedIRData.address
     * and up to 32 bit raw data in IrReceiver.decodedIRData.decodedRawData
     */
    if (IrReceiver.decode()) {
        // Print a short summary of received data        
        IrReceiver.printIRResultShort(&Serial);
        IrReceiver.printIRSendUsage(&Serial);
        if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
            Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
            // We have an unknown protocol here, print more info
            IrReceiver.printIRResultRawFormatted(&Serial, true);
        }        
        Serial.println();

        /*
         * !!!Important!!! Enable receiving of the next value,
         * since receiving has stopped after the end of the current received data packet.
         */
        IrReceiver.resume(); // Enable receiving of the next value

        irrcvTranslate(IrReceiver.decodedIRData.decodedRawData);
        //Finally, check the received data and perform actions according to the received command
        if (IrReceiver.decodedIRData.command == 0x10) {
            // do something
        } else if (IrReceiver.decodedIRData.command == 0x11) {
            // do something else
        }
    }
#endif
}

/*Parse raw signal of IR Receiver Module to user compartible data*/
void irrcvTranslate(uint32_t raw){
#ifdef L14
  Serial.println((raw&0x00FF0000)>>16, HEX);//get the command byte
  Serial.println((raw&0x0000FFFF), HEX);//get the address bytes

  switch(raw)
  {
  case 0xBA45FF00: Serial.println("POWER"); break;
  case 0xB847FF00: Serial.println("FUNC/STOP"); break;
  case 0xB946FF00: Serial.println("VOL+"); break;
  case 0xBB44FF00: Serial.println("FAST BACK");    break;
  case 0xBF40FF00: Serial.println("PAUSE");    break;
  case 0xBC43FF00: Serial.println("FAST FORWARD");   break;
  case 0xF807FF00: Serial.println("DOWN");    break;
  case 0xEA15FF00: Serial.println("VOL-");    break;
  case 0xF609FF00: Serial.println("UP");    break;
  case 0xE619FF00: Serial.println("EQ");    break;
  case 0xF20DFF00: Serial.println("ST/REPT");    break;
  case 0xE916FF00: Serial.println("0");    break;
  case 0xF30CFF00: Serial.println("1");    break;
  case 0xE718FF00: Serial.println("2");    break;
  case 0xA15EFF00: Serial.println("3");    break;
  case 0xF708FF00: Serial.println("4");    break;
  case 0xE31CFF00: Serial.println("5");    break;
  case 0xA55AFF00: Serial.println("6");    break;
  case 0xBD42FF00: Serial.println("7");    break;
  case 0xAD52FF00: Serial.println("8");    break;
  case 0xB54AFF00: Serial.println("9");    break;
  case 0xFFFFFFFF: Serial.println("REPEAT F");break;  
  case 0x00000000: Serial.println("REPEAT 0");break; 

  default:
    Serial.print(F("undefined/unexpected command: "));
    Serial.println(raw, HEX);
  }
  delay(500); // Do not get immediate repeat
#endif
}

/*MAX7219 LED Dot Matrix Module*/
void loopL15(void) {
#ifdef L15
  u8g2.clearBuffer();					// clear the internal memory    
  for(byte i = 0; i < text.length(); i++){    
    u8g2.drawStr(0, 8, &text[i]);	// write something to the internal memory
    u8g2.sendBuffer();					// transfer internal memory to the display    
    delay(1000);    
    u8g2.clear();//u8g2.clearBuffer();//u8g2.clearDisplay();
  }
#endif
}

/*MPU-6050 (GY-521 Module)*/
void loopL16(){
#ifdef L16
  if(digitalRead(BTN1_PIN)) return;
  // read raw accel/gyro measurements from device
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  int16_t temp_raw = mpu6050.getTemperature();

  // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);
#ifndef DEBUG
  #ifdef OUTPUT_READABLE_ACCELGYRO    
    // display tab-separated accel/gyro x/y/z values
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\t");
    Serial.println(temp_raw/340.00 + 36.53);//looks like error in datasheet :(
  #endif
#endif

  #ifdef OUTPUT_BINARY_ACCELGYRO
    Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
    Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
    Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
    Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
    Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
    Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
  #endif
        
  digitalWrite(LED_BUILTIN, blinkState = !blinkState);
  delay(500);
#endif
}

/*HC-SR501 PIR Sensor*/
void loopL17(void){  
#ifdef L17
  if(!btn1Status) return;
#ifndef DEBUG  
  Serial.println(digitalRead(RIP_PIN));  
#endif
  digitalWrite(LED_PIN, digitalRead(RIP_PIN));
#endif
}

/*Water Level Detection Sensor Module*/
void loopL18(void){
#ifdef L18
  if(!btn1Status) return;
  int value = analogRead(ADC_PIN); // get adc value

  if(
    ((HistoryValue>=value) && ((HistoryValue - value) > 10)) || 
    ((HistoryValue<value) && ((value - HistoryValue) > 10))){
      sprintf(printBuffer, "ADC%d level is %d\n", ADC_PIN, value);
      Serial.print(printBuffer);
      HistoryValue = value;
  }
#endif  
}

/*DS1307 RTC (Real Time Clock Module)*/
void loopL19(void){
#ifdef L19
  if(!btn1Status) return;
#ifndef DEBUG
  DateTime now = rtc.now();
  
  sprintf(printBuffer, "%u/%u/%u (%s) %u:%u:%u\n", 
    now.year(), now.month(), now.day(), daysOfTheWeek[now.dayOfTheWeek()], 
    now.hour(), now.minute(), now.second());
  Serial.print(printBuffer);

  sprintf(printBuffer, "Since midnight 1/1/1970: %u seconds (%i days)\n", 
    now.unixtime(), (now.unixtime()/86400L));
  Serial.print(printBuffer);
#endif
#endif  
}

/*KY-037 (Sound Sensor Module)*/
void loopL20(void){
#ifdef L20
  if(!btn1Status){
    digitalWrite(LED_PIN, LOW);
    return;
  }
#ifndef DEBUG
  sprintf(printBuffer, "D: %u, A: %i\n", digitalRead(SSM_DO_PIN), analogRead(SSM_AO_PIN));
  Serial.print(printBuffer);  
#endif
  digitalWrite(LED_PIN, digitalRead(SSM_DO_PIN));
#endif  
}

/*
    RC522 (RFID Module)
    https://www.electronicshub.org/arduino-rc522-rfid-module/
*/
void loopL21(void){
#ifdef L21
  if(!btn1Status){
    digitalWrite(LED_PIN, LOW);
    return;
  }
#ifndef DEBUG
  //sprintf(printBuffer, "D: %u, A: %i\n", digitalRead(SSM_DO_PIN), analogRead(SSM_AO_PIN));
  //Serial.print(printBuffer);
	
	if (!mfrc522.PICC_IsNewCardPresent()) return;	
	if (!mfrc522.PICC_ReadCardSerial()) return;
  
  //init key '46 D7 97 C8 24 21'
  MFRC522::MIFARE_Key key;
  key.keyByte[0] = 0x46; key.keyByte[1] = 0xD7; key.keyByte[2] = 0x97;
  key.keyByte[3] = 0xC8; key.keyByte[4] = 0x24; key.keyByte[5] = 0x21;
	
  // Dump debug info about the card; PICC_HaltA() is automatically called  
  digitalWrite(LED_PIN, HIGH);
  //mfrc522.PCD_Authenticate();
  mfrc522.PICC_DumpToSerial(&(mfrc522.uid));
  //mfrc522.dum
  digitalWrite(LED_PIN, LOW);  
#endif  
#endif  
}

/*LCD1602 (LCD Display)*/
void loopL22(void){
#ifdef L22
#ifndef DEBUG
  //sprintf(printBuffer, "D: %u, A: %i\n", digitalRead(SSM_DO_PIN), analogRead(SSM_AO_PIN));
  //Serial.print(printBuffer);
#endif  
#endif  
}

/*7941E 3Pin 125KHz (TZT RFID UART Reader Wireless Module) */
void loopL34(void){
#ifdef L34
#ifndef RFID2
  if (gwiot7941e.update()) {    
    Serial.println(gwiot7941e.getLastTagId(), HEX);
  }
  else Serial.print("*");
#else
    char buff[100];

    while(Serial.available()){
      Serial.print(" ");
      Serial.print(Serial.read(), HEX);
    }
    
    if (!stream_) return;// false;

    //set 115200 (3F3F3F3F3F 40)
    byte test1[]{0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x31, 0x31, 0x35, 0x32, 0x30, 0x30, 0x40};
    stream_->write(test1, 12);
    byte test2[]{0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x40};
    stream_->write(test2, 6);
    
    char test[] = "Delete all cards @";//"Query all cards @" "query status" "Set baud rate 115200@"
    stream_->write(test);

    if (!stream_->available()){
      Serial.print("*");
      delay(1000);
      return;// false;
    }

    // if a packet doesn't begin with the right byte, remove that byte
    if (stream_->peek() != GWIOT_7941E_PACKET_BEGIN/* && stream_->read()*/) {
      Serial.println(stream_->read());
      return;// false;
    }

    // if read a packet with the wrong size, drop it
    if (GWIOT_7941E_PACKET_SIZE != stream_->readBytes(buff, GWIOT_7941E_PACKET_SIZE)) {
      Serial.println("wrong length");
      return;// false;
    }
return;
    // if a packet doesn't end with the right byte, drop it
    //if (buff[9] != GWIOT_7941E_PACKET_END) return false;

    // calculate checksum (excluding start and end bytes)
    uint8_t checksum = 0;
    for (uint8_t i = 1; i <= 8; ++i) {
        checksum ^= buff[i];
    }
    //if (checksum) return false;

    // extract tag id from message
    lastTagId_ = 0;
    for (uint8_t i = 0; i <= 3; ++i) {
        uint32_t val = (uint8_t) buff[i+4];
        lastTagId_ |= val << (8 * (3 - i));
    }
#endif
  delay(1000);
#endif
}