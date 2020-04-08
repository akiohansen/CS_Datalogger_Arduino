/*
 Hamburg Urban Soil Observatory (HUSCO) NET Arduino Network Connector
 - Further information about HUSCONET: www.wetterstationen.uni-hamburg.de
 - Reads Campbell Scientific CS1000 Logger, stores data on microSD card
    and uploads it as CSV files to predefined FTP server with
    1) WiFi module on Arduino MKR WiFi 1010 or
    2) GPRS/3G module of Arduino MKR GSM 1400

 Acknowledgements: Arduino plattform and following libraries:
 - FTP Client support from project https://playground.arduino.cc/Code/FTP/
 - WLAN features implemented by: https://www.arduino.cc/en/Reference/WiFiNINA
 - SD Card features realised by: https://github.com/greiman/SdFat
 - RealTimeClock (RTC) via NTP-server by: https://www.arduino.cc/en/Tutorial/WiFiRTC
    based on RTC Library for Arduino: https://github.com/arduino-libraries/RTCZero
 - GSM connectivity based on: https://www.arduino.cc/en/Reference/MKRGSM
    example used "TestGPRS", Created 18 Jun 2012 by David del Peral
 - "wiring_private.h": Copyright (c) 2005-2006 David A. Mellis, GNU Lesser General Public
     License

 Authors: Akio Hansen, Katharina Holube
 Last update: 08/04/2020
 Version History: v5

 v1: Version with loop down and upload of one single file
 v2: Download index file from server with files to upload, every 5 min
     Upload files of new index file
 v3: With datalog feature, read analog sensor inputs and RTC time feature,
     changed to SDFat Library for long file names and more efficient processing,
     added two options for upload files (1: append to proc file, 2: upload newest data)
 v4: Cleaned code, moved code to functions, prepared for Github publication
 v5: Included Campbell Scientific Datalogger Communication Code from Katharina Holube
      Sourcefile: Stand_2020_01_31.ino, Added #include <Arduino.h> for "wiring_private.h"

 GPRS version: GSM GPRS connectivity version, use celluar network for
         data transmission instead of WiFi
 
 */
#include <SPI.h>
//#include <SD.h>    // Old library, supports only short filenames with 12 characters
//#include <WiFiNINA.h> // WiFi version
#include <MKRGSM.h>     // GSM version

//#include <WiFiUdp.h> // required for NTP Time with WiFi module
#include <RTCZero.h> // required for NTP Time

// Code to use SD.h functions with newest SdFat library
#include <SdFat.h>
SdFat SD;

// Include all private user data, defined in arduino_secrets.h
#include "arduino_secrets.h"

#include <Arduino.h>   // required before wiring_private.h
// Files required for right communication with Campbell Scientific logger
#include "wiring_private.h" // pinPeripheral() function
//D0-TX, D1-RX
Uart mySerial(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// initialize time objects and variables
RTCZero rtc;
const int GMT = 0; //change this to adapt it to your time zone

// Define global file handler
File fh;
File fhftp;

// Specify FTP server (IP address or DNS based - requires larger sketch size)
// if you don't want to use DNaS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
//IPAddress server(192,168,178,1);
char server[] = ftpserver;

// Initialize the WiFi client:
//WiFiClient client;
//WiFiClient dclient;

// Initialize the GSM client and set required information
// Please enter your sensitive data in the Secret tab or arduino_secrets.h
// PIN Number
//const char PINNUMBER[] = SECRET_PINNUMBER;

// initialize the library instance
GSM gsmAccess;      // GSM access: include a 'true' parameter for debug enabled
GPRS gprsAccess;    // GPRS access
GSMClient client;   // Client service for TCP connection
GSMClient dclient;  // Client service for TCP connection

GSMScanner scannerNetworks; // Optional Debug GSM object for current network carrier

// messages for serial monitor response
String oktext = "OK";
String errortext = "ERROR";


// Initialize Campbell Scientific Logger Communication variables
int answerlen = 85;      // Length of the answer in 16-bit integers, needs to be uneven
int output[85];          // Size is the length of the answer in byte, double of answerlen
float floats[20];        // Maximum number of floats that need to be saved at a given time: soil data   has 20 values
byte message[8];         // All Modbus Requests are 8 bytes long

// Output file for datalogger
File Textdatei;

//variables for epoch conversion
int yearCounter;
int monthCounter;
int dayCounter;
int numberOfDaysInYear = 365;

String currentStringForFloat = "";
const int chipSelect = 4;


// Initialize variables for reading SD card content
char outBuf[128];
char outCount;

// Set filename for Server file containing requested datalog files
char fileNameProc[14] = "FILESUP.TXT";
// Initialize dynamic filename and loop count for datalogger
char fileNameData[30];

// WiFi module part
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
// char ssid[] = SECRET_SSID;        // Network SSID (name)
// char pass[] = SECRET_PASS;        // Network password (use for WPA, or use as key for WEP)
// int status = WL_IDLE_STATUS;      // WiFi radio status

// Initilisation program code - processed only once at startup
void setup() {

  // Initialize object for serial communication with Campbell logger
  mySerial.begin(19200);
  
  pinPeripheral(0, PIO_SERCOM);
  pinPeripheral(1, PIO_SERCOM);
  
  //Initialize serial, SD-Card and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect.
  }

  digitalWrite(10,HIGH);

  // Start open SD card object
  Serial.print("Initializing SD card...");
  if(SD.begin(4) == 0)
  {
    Serial.println(F("SD init fail"));          
  }

  // Setup GSM GPRS connection with error handling for data transmission
  // start GSM shield
  // if your SIM has PIN, pass it as a parameter of begin() in quotes
  Serial.print("Connecting GSM network...");
  if (gsmAccess.begin() != GSM_READY) {
    Serial.println(errortext);
    while (true);
  }
  Serial.println(oktext);

  // Start of GSM network information
  // currently connected carrier
  Serial.print("Current carrier: ");
  Serial.println(scannerNetworks.getCurrentCarrier());

  // returns strength and ber
  // signal strength in 0-31 scale. 31 means power > 51dBm
  // BER is the Bit Error Rate. 0-7 scale. 99=not detectable
  Serial.print("Signal Strength: ");
  Serial.print(scannerNetworks.getSignalStrength());
  Serial.println(" [0-31]");
  // End of GSM network information

  // Set network provider information
  char apn[50] = gsmapn;
  // Read APN login introduced by user
  char login[50] = gsmlogin; 
  // read APN password introduced by user
  char password[20] = gsmpasswd;   

  // attach GPRS
  Serial.println("Attaching to GPRS with your APN...");
  if (gprsAccess.attachGPRS(apn, login, password) != GPRS_READY) {
    Serial.println(errortext);
  } else {

    Serial.println(oktext);
    
    // Connection with FTP is ready
    Serial.print("Connecting and sending FTP data to server...");

  }

  
  //Serial.print("Waiting for FTP Connection.");
  delay(1000);

  // Set NTP RealTimeClock, if fails set clock to 1990
  // start synchronizing clock with NTP server
  rtc.begin();

  unsigned long epoch;
  int numberOfTries = 0, maxTries = 10;
  do {
    //epoch = WiFi.getTime();      // WiFi version
    epoch = gsmAccess.getTime();   // GSM version
    numberOfTries++;
  }
  while ((epoch == 0) && (numberOfTries < maxTries));

  if (numberOfTries == maxTries) {
    Serial.println("NTP unreachable!!");
    Serial.println("Set epoch to 1990");
    rtc.setEpoch(631152000);
    //while (1);
  }
  else {
    Serial.print("Epoch received: ");
    Serial.println(epoch);
    rtc.setEpoch(epoch);

    Serial.println();
  }

  delay(2000);

  // Option: Manual Down-/Upload of files by defined FTP functions
  // .        please uncomment functions (Download: doFTPDownload, Upload: doFTPUpload)
  //  Serial.println("Start downloading process file...");
  //  if(doFTPUpload(fileNameProc)) Serial.println(F("FTP OK"));
  //  else Serial.println(F("FTP FAIL"));
}

// Loop function, processed continously after setup function
void loop() {
  // Define functions and code to run at main program

  // Start download server file by FTP to check for requested, additional files
  Serial.println("Start downloading process file...");
  if(doFTPDownload(fileNameProc)) Serial.println(F("FTP OK"));
  else Serial.println(F("FTP FAIL"));

  // Insert delay - 5 sec
  delay(5000);

  // Generate well-formatted datestring for new file to open
  snprintf(fileNameData, 64, "HUSCO01_%04d%02d%02d_%02d%02d.csv",
    rtc.getYear()+2000, rtc.getMonth(), rtc.getDay(),
    rtc.getHours(), rtc.getMinutes());

  // Simple filename formatting - Debug option
  //snprintf(fileNameData, 64, "%02d%02d.csv",
  //  rtc.getHours(), rtc.getMinutes());
  Serial.println("Files will be written to: ");
  Serial.print(fileNameData);

  // Option 1: Synthetic data logger
  //  - Generate 10 datalogger entries by collecting data from analog ports
  //    Has to be replaced by: Download new data from Datalogger
  dataLog(fileNameData);
  delay(5000);

  // Option 2: Real Campbell Scientific logger
  //  - Code based on Katharina Holube work -- !!! NOT TESTED YET !!!
//  int start = 0;
//  for (int i=0; i<50; i++){
//    if (i==1){
//      start = start+86;  //soil and status data are between minute 0 and minute 1    
//    }
//    if (i==31){
//      start = start+80;  //soil data are between minute 0 and minute 31
//    }
//
//  designModbusRequest(message, start, ((answerlen-5)/2));
//  delay(2);
//  mySerial.write(message, sizeof(message)); 
//  delay(2);
//  readLogger(answerlen,output); //read answer
//  fourIntToFloat(output,floats,20); //convert four ints of the answer to a float
//  long seconds = loggerFloatsToSeconds(floats);
//  SDwrite(0,seconds);
//  start=start+34;
//  Serial.println("Finished CS-reading, wait for next bunch of data.");
//  delay(3000);

  // Option 1: Append file name to upload full file list
  //fnameAppend(fileNameProc, fileNameData);
  //delay(1000);

  // Option 2: Upload only newest dataLog file and
  //             upload afterwards requested Server files,
  //             if upload fails add to server list
  Serial.println("Start uploading new datalogger file...");
  if(doFTPUpload(fileNameData)){
    Serial.println(F("FTP OK"));
  }
  else{ 
    Serial.println(F("FTP FAIL")); 
    fnameAppend(fileNameProc, fileNameData);
  }

  // Processing of requested files, upload them to FTP server
  Serial.println("Processing file and upload files...");
  readSD(fileNameProc);

  // Sleep for 10 minutes to start next loop
  delay(600000);
  
}


// FTP Function for download
byte doFTPDownload(char *fileNameDown)
{
// First download FTP file
  SD.remove(fileNameDown);
  fh = SD.open(fileNameDown,FILE_WRITE);

  if(!fh)
  {
    Serial.println(F("SD open fail"));
    return 0;    
  }

  if(!fh.seek(0))
  {
    Serial.println(F("Rewind fail"));
    fh.close();
    return 0;    
  }

  Serial.println(F("SD opened"));

  if (client.connect(server,21)) {
    Serial.println(F("Command connected"));
  }
  else {
    fh.close();
    Serial.println(F("Command connection failed"));
    return 0;
  }

  if(!eRcv()) return 0;

  client.println(F(ftpuser));

  if(!eRcv()) return 0;

  client.println(F(ftppasswd));

  if(!eRcv()) return 0;

  client.println(F("SYST"));

  if(!eRcv()) return 0;

  client.println(F("Type I"));

  if(!eRcv()) return 0;

  client.println(F("PASV"));

  if(!eRcv()) return 0;

  char *tStr = strtok(outBuf,"(,");
  int array_pasv[6];
  for ( int i = 0; i < 6; i++) {
    tStr = strtok(NULL,"(,");
    array_pasv[i] = atoi(tStr);
    if(tStr == NULL)
    {
      Serial.println(F("Bad PASV Answer"));    

    }
  }

  unsigned int hiPort,loPort;

  hiPort = array_pasv[4] << 8;
  loPort = array_pasv[5] & 255;

  Serial.print(F("Data port: "));
  hiPort = hiPort | loPort;
  Serial.println(hiPort);

  if (dclient.connect(server,hiPort)) {
    Serial.println(F("Data connected"));
  }
  else {
    Serial.println(F("Data connection failed"));
    client.stop();
    fh.close();
    return 0;
  }

  client.print(F("RETR "));
  client.println(fileNameDown);

  if(!eRcv())
  {
    dclient.stop();
    return 0;
  }

  while(dclient.connected())
  {
    while(dclient.available())
    {
      char c = dclient.read();
      fh.write(c);      
      Serial.write(c);
    }
  }

  dclient.stop();
  Serial.println(F("Data disconnected"));

  if(!eRcv()) return 0;

  client.println(F("QUIT"));

  if(!eRcv()) return 0;

  client.stop();
  Serial.println(F("Command disconnected"));

  fh.close();
  Serial.println(F("SD closed"));
  return 1;
}

// FTP Function for upload
byte doFTPUpload(char *fileNameUp)
{  
// Second upload FTP file again
  Serial.println("Start uploading file");
  Serial.println(fileNameUp);

  fhftp = SD.open(fileNameUp,FILE_READ);

  if(!fhftp)
  {
    Serial.println(F("SD open fail"));
    return 0;    
  }

  Serial.println(F("SD opened"));

  if (client.connect(server,21)) {
    Serial.println(F("Command connected"));
  }
  else {
    fhftp.close();
    Serial.println(F("Command connection failed"));
    return 0;
  }

  if(!eRcv()) return 0;

  client.println(F(ftpuser));

  if(!eRcv()) return 0;

  client.println(F(ftppasswd));

  if(!eRcv()) return 0;

  client.println(F("SYST"));

  if(!eRcv()) return 0;

  client.println(F("Type I"));

  if(!eRcv()) return 0;

  client.println(F("PASV"));

  if(!eRcv()) return 0;

  char *tStr = strtok(outBuf,"(,");
  int array_pasv[6];
  for ( int i = 0; i < 6; i++) {
    tStr = strtok(NULL,"(,");
    array_pasv[i] = atoi(tStr);
    if(tStr == NULL)
    {
      Serial.println(F("Bad PASV Answer"));    

    }
  }

  unsigned int hiPort,loPort;

  hiPort = array_pasv[4] << 8;
  loPort = array_pasv[5] & 255;

  Serial.print(F("Data port: "));
  hiPort = hiPort | loPort;
  Serial.println(hiPort);

  if (dclient.connect(server,hiPort)) {
    Serial.println(F("Data connected"));
  }
  else {
    Serial.println(F("Data connection failed"));
    client.stop();
    fhftp.close();
    return 0;
  }

  client.print(F("STOR "));
  client.println(fileNameUp);

  if(!eRcv())
  {
    dclient.stop();
    return 0;
  }

  Serial.println(F("Writing"));

  byte clientBuf[64];
  int clientCount = 0;

  // dclient.write(clientBuffer,8); - Write Test string to file

  while(fhftp.available())
  {
    clientBuf[clientCount] = fhftp.read();
    clientCount++;

    if(clientCount > 63)
    {
      dclient.write(clientBuf,64);
      clientCount = 0;
    }
  }

  if(clientCount > 0) dclient.write(clientBuf,clientCount);

  dclient.stop();
  Serial.println(F("Data disconnected"));

  if(!eRcv()) return 0;

  client.println(F("QUIT"));

  if(!eRcv()) return 0;

  client.stop();
  Serial.println(F("Command disconnected"));

  fhftp.close();
  Serial.println(F("SD closed"));
  return 1;
  
}


byte eRcv()
{
  byte respCode;
  byte thisByte;

  while(!client.available()) delay(1);

  respCode = client.peek();

  outCount = 0;

  while(client.available())
  {  
    thisByte = client.read();    
    Serial.write(thisByte);

    if(outCount < 127)
    {
      outBuf[outCount] = thisByte;
      outCount++;      
      outBuf[outCount] = 0;
    }
  }

  if(respCode >= '4')
  {
    efail();
    return 0;  
  }

  return 1;
}


void efail()
{
  byte thisByte = 0;

  client.println(F("QUIT"));

  while(!client.available()) delay(1);

  while(client.available())
  {  
    thisByte = client.read();    
    Serial.write(thisByte);
  }

  client.stop();
  Serial.println(F("Command disconnected"));
  fh.close();
  Serial.println(F("SD closed"));
}

void readSD(char *fileNameProc)
{
  // Try to open SD card content
  fh = SD.open(fileNameProc,FILE_READ);

  if(!fh)
  {
    Serial.println(F("SD open fail"));
    return;    
  }

  Serial.println("Start reading...");

  char buffersd[30];
  memset(buffersd, 0, sizeof(buffersd));
  //uint8_t i = 0;
  while (fh.available())
  {
    fh.readBytesUntil('\n', buffersd, 30);

    Serial.print("Start FTP file:");
    Serial.print(buffersd);

    if(buffersd[0] == '\0')
    {
      Serial.println("Empty line, nothing to do.");
      memset(buffersd, 0, sizeof(buffersd));
    }
    else {
      Serial.println("Start upload file: ");
      Serial.print(buffersd);

      if (doFTPUpload(buffersd) == 1)
      {
        Serial.println(F("FTP OK"));
      }
      else {
        Serial.println(F("FTP FAIL"));
      }
      
      //Serial.println(sizeof(buffersd));
      Serial.println("Next line");
      memset(buffersd, 0, sizeof(buffersd));
     }
  }

  fh.close();

}

// Datalog function to create data from analog sensors for FTP test case
void dataLog(char *fileNameData)
{
  uint8_t t = 0;

  Serial.println("Data should be written to:");
  Serial.println(fileNameData);
  
  while (t < 11)
  {
    
    // make a string for assembling the data to log:
    String dataString = "";
  
    // read three sensors and append to the string:
    for (int analogPin = 0; analogPin < 3; analogPin++) {
      int sensor = analogRead(analogPin);
      dataString += String(sensor);
      if (analogPin < 2) {
        dataString += ",";
      }
    }

    //Serial.println(fileNameData);
    //Serial.println(dataString);
  
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File dataFile = SD.open(fileNameData, FILE_WRITE);
  
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
      // print to the serial port too: (Debug option)
      // Serial.println(dataString);
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening ");
      Serial.print(fileNameData);
    }
    t++;
    dataFile.close();
    
  }
  //return 1;
}

void fnameAppend(char *fileNameProc, char *fileNameData)
{
  Serial.println("Append filename to file: ");
  Serial.println(fileNameProc);
  // Serial.println(fileNameData);

  // Open the file.
  File dataFileNames = SD.open(fileNameProc, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFileNames) {
    dataFileNames.print('\n');
    dataFileNames.print(fileNameData);
    //dataFileNames.print('\n');
    //dataFileNames.println('\n');
    dataFileNames.close();
    // print to the serial port too:
    //Serial.println(fileNameData);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening FileNames text file");
  }
  
  Serial.println("Successfully appended filename.");

  dataFileNames.close();
}

// Campbell Scientific Datalogger functions - based on work of Katharina Holube
// 
void designModbusRequest(byte messageArray[], int startingRegister, int numberOfRegisters){
  messageArray[0] = 0x01;        //slave adress                               
  messageArray[1] = 0x03;        //function number: 03 read holding registers
  messageArray[2] = highByte(startingRegister);    //split the 16 bit integers into two bytes
  messageArray[3] = lowByte(startingRegister);
  messageArray[4] = highByte(numberOfRegisters);
  messageArray[5] = lowByte(numberOfRegisters);
  
  uint16_t crc = 0xFFFF;                  //calculate Cyclic Redundancy Check
  for (int pos = 0; pos < 6; pos++) {
    crc ^= (uint16_t)messageArray[pos];   // XOR byte into least sig. byte of crc
    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  
  messageArray[6] = lowByte(crc);    //switch byte order and append to messageArray
  messageArray[7] = highByte(crc);
  
  
}

// Call this function to store the logger´s answer in an array
void readLogger(int answerlenlocal, int outputArray[]){
  delay(100); // wait to fill buffer
  for(int i=0; i<answerlenlocal; i++){
    //while(mySerial.available()); Ohne diese Zeile funktioniert es besser
    outputArray[i] = mySerial.read();
  }
}

// Call this function to convert four ints to a float and store all floats in an array
void fourIntToFloat(int outputArray[], float floatArray[], int numberOfFloats){         
  for(int j=0; j<(numberOfFloats); j++){                                  
    long part1 = outputArray[4*j+5]; // has to be long, so the bit shifts are possible later
    long part2 = outputArray[4*j+6];
    long part3 = outputArray[4*j+3];
    long part4 = outputArray[4*j+4];
    union{                                                          // long and float share the same storage space
      long c;
      float f;
    }modbusfloat;
    modbusfloat.c = ((part1<<24)+(part2<<16)+(part3<<8)+(part4));    //all four ints behind one another
    floatArray[j] = modbusfloat.f;
    //Serial.println(floatArray[j]);
  }
}

long loggerFloatsToSeconds(float floatArray[]){
  long hByte = (long)floatArray[0];
  long lByte = (long)floatArray[1];
  long result = (hByte << 16)+lByte;
  return result;
}

void SERCOM3_Handler()
{
  mySerial.IrqHandler();
}

//TODO: save some storage space here, if the unnecessary zeros are deleted, table = 0 for minutes, 1 for soil, 2 for status
void SDwrite (int table, long secondsSince1990){
  String fileName = "";
  String writeLine = "";
  String writeLine2 = "";
  long daysSince1990 = secondsSince1990/86400;
  int fourYearSets = daysSince1990/1461;
  yearCounter = 1990+fourYearSets*4;
  int lastDays = daysSince1990-1461*fourYearSets;
  if(lastDays >= 365){ //subtract normal year when possible
    lastDays = lastDays-365;
    yearCounter = yearCounter+1;
  }
  if(lastDays >= 365){ //subtract another normal year when possible
    lastDays = lastDays-365;
    yearCounter = yearCounter+1;
    numberOfDaysInYear = 366;
  }
  if(lastDays >= 366){ //subtract leap year when possible
    lastDays = lastDays-366;
    yearCounter = yearCounter+1;
    numberOfDaysInYear = 365;
  }
  //Now we are in the current year
  if (lastDays > numberOfDaysInYear-32){ //December
    monthCounter = 12;
    dayCounter = lastDays-(numberOfDaysInYear-32);
  }
  else if(lastDays > numberOfDaysInYear-62){ //November
    monthCounter = 11;
    dayCounter = lastDays-(numberOfDaysInYear-62);
  }
  else if(lastDays > numberOfDaysInYear-93){ //October
    monthCounter = 10;
    dayCounter = lastDays-(numberOfDaysInYear-93);
  }
  else if(lastDays > numberOfDaysInYear-123){ //September
    monthCounter = 9;
    dayCounter = lastDays-(numberOfDaysInYear-123);
  }
  else if(lastDays > numberOfDaysInYear-154){ //August
    monthCounter = 8;
    dayCounter = lastDays-(numberOfDaysInYear-154);
  }
  else if(lastDays > numberOfDaysInYear-185){ //Juli
    monthCounter = 7;
    dayCounter = lastDays-(numberOfDaysInYear-185);
  }
  else if(lastDays > numberOfDaysInYear-215){ //Juni
    monthCounter = 6;
    dayCounter = lastDays-(numberOfDaysInYear-215);
  }
  else if(lastDays > numberOfDaysInYear-246){ //Mai
    monthCounter = 5;
    dayCounter = lastDays-(numberOfDaysInYear-246);
  }
  else if(lastDays > numberOfDaysInYear-276){ //April
    monthCounter = 4;
    dayCounter = lastDays-(numberOfDaysInYear-276);
  }
  else if(lastDays > numberOfDaysInYear-307){ //March
    monthCounter = 3;
    dayCounter = lastDays-(numberOfDaysInYear-307);
  }
  else if(lastDays <= 31){ //January
    monthCounter = 1;
    dayCounter = lastDays+1;
  }
  else{
    monthCounter = 2;
    dayCounter = lastDays-30;
  }

  long hoursMinutes = secondsSince1990-daysSince1990*86400;
  int hours = hoursMinutes/3600;
  int minutes = hoursMinutes%3600/60;
  // create file on SD-Card with month and day in the name
  if (monthCounter < 10){
    fileName += "0";
  }
  fileName += (String)monthCounter;
  if (dayCounter < 10){
    fileName += "0";
  }
  fileName += (String)dayCounter;
  if (table == 0){
    fileName += ".wgm"; // end of file name for minute data
  }
  if (table == 1){
    fileName += ".wgs"; // end of file name for soil data
  }
  if (table == 2){
    fileName += ".wga"; // end of file name for status data
  } 
  if (dayCounter < 10){ // Zeros if necessary
    writeLine += "0";
  }
  writeLine += (String)dayCounter;
  if (monthCounter < 10){ //Zeros if necessary
    writeLine += ".0";
  }
  else{
    writeLine += ".";
  }
  writeLine += (String)monthCounter;
  writeLine += ".";
  writeLine += (String)yearCounter;
  if (hours<10){ //Zeros if necessary
    writeLine += ";0";
  }
  else {
    writeLine += ";";
  }
  writeLine += (String)hours;
  if (minutes<10){ //Zeros if necessary
    writeLine += ":0";
  }
  else{
    writeLine += ":";
  }
  writeLine += (String)minutes;
  writeLine += ":00;UTC+1;HUSCOGREP-1;;;;;;";
  if (table == 0){   //Minutendaten 
    //Temperature, Humidity, Tipping Bucket
    for (int i=2; i<5; i++){
      if(!isnan(floats[i])){
        writeLine += (String)floats[i];
      }
      writeLine += ";";
    }
  }
    //Rain and Wind (WXT)
    for(int i=5; i<7; i++){
      if(!isnan(floats[i])){
        floatToString(floats[i],3); // 3 decimal digits
        writeLine += (String)currentStringForFloat;
        currentStringForFloat = "";
      }
      writeLine += ";";
    }
    //mean_wind_speed, mean_wind_direction, std_wind_dir, max_wind_speed, min_wind_speed, BP_mbar_Avg; AVG_Surface_Temperature_C, SlrW, SlrkJ, Soil_HF_Wpm2 (total 10 values)
    for(int i=7; i<17; i++){
      if(!isnan(floats[i])){
        writeLine += (String)floats[i];
      }
      if(i != 16){
        writeLine += ";";
      }
     }
  /*if (table == 1){ // TODO: Bodendaten und Statusdaten hinzufügen!!
    
  }*/
  if(!SD.exists(fileName.c_str())){ // if the file is not already there, print the key
      Textdatei = SD.open(fileName, FILE_WRITE);
      Textdatei.println("DATE;TIME;TIMEZONE;VERSION;LAT;LON;ALT;SN;RECORD;AirTC_Avg;RH_Avg;Raing_mm_Tot;Rain_mm;Hail_hits;mean_wind_speed;mean_wind_direction;std_wind_dir;max_wind_speed;min_wind_speed;BP_mbar_Avg;AVG_Surface_Temperature_C;SlrW;SlrkJ;Soil_HF_Wpm2");
    }
    else{
      Textdatei = SD.open(fileName, FILE_WRITE);
    }
    Textdatei.println(writeLine);
    Textdatei.close();
    Serial.println(writeLine);
    Serial.println("abgeschlossen");
}

//Function to convert float to String
// Source: Practical Arduino Engineering by Harold Timmis, page 118
void floatToString(float number, int digits){
  //Handle negative numbers
  if (number<0.0){
    currentStringForFloat += "-";
    number = -number;
  }
  //Round corectly so that print(1.999,2) becomes "2,00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /=10.0;
  number += rounding;

  //Extract the integer part of the number and append it to the string
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  currentStringForFloat += (String)int_part;

  //Append the decimal point, but only if there are digits beyond
  if (digits > 0)
    currentStringForFloat += ".";

  //Extract digits from the remainder one at a time
  while (digits-- >0){
    remainder *= 10.0;
    int toPrint = int(remainder);
    currentStringForFloat += (String)toPrint;
    remainder -= toPrint;
  }
}

// End Campbell Scientific Datalogger functions
