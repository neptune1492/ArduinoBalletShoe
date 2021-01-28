include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include "utility/debug.h"
#include<stdlib.h>
#include <Time.h>
#include <TimeLib.h>
#include <WiFiUdp.h>
// Define CC3000 chip pins
#define ADAFRUIT_CC3000_IRQ 3//MUST be an interrupt pin
#define ADAFRUIT_CC3000_VBAT 5
#define ADAFRUIT_CC3000_CS 10

// these constants describe the pins. They won't change:
const int groundpin = 18;             // analog input pin 4 -- ground
const int powerpin = 19;              // analog input pin 5 -- voltage
const int xpin = A3;                  // x-axis of the accelerometer
const int ypin = A2;                  // y-axis
const int zpin = A1;                  // z-axis (only on 3-axis models)

static const char ntpServerName[] = "us.pool.ntp.org"; 
const int timeZone = 0;  // UTC

String timestamp;

// Create CC3000 instances
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIV2); // you can change this clock speed
                               
// WLAN parameters
//HOME:
//#define WLAN_SSID "RADIOSHACK"
//#define WLAN_PASS "A2M3D4G5RAD"
//Phone:
#define WLAN_SSID "LG Escape2_775d" //slu. CASE SENSITIVE!
#define WLAN_PASS "34d0cecc06ec" //not needed but defined anyway.
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY WLAN_SEC_WPA2 //Home
  Adafruit_CC3000_Client client;
//#define WLAN_SECURITY WLAN_SEC_UNSEC //unsecured
// Carriots parameters
//define WEBSITE "10.0.0.11" //HOME
#define WEBSITE "192.168.43.143" //Phone
#define API_KEY "42c3132cb37de19a8cdd505e431e411a4b8fc1fda02866fdd9580a853e412a68"
#define DEVICE "LaundryBug@LaundryBugDevice.LaundryBugDevice"

uint32_t ip;

const unsigned long
  connectTimeout  = 15L * 1000L, // Max time to wait for server connection
  responseTimeout = 15L * 1000L; // Max time to wait for data from server
int
  countdown       = 0;  // loop() iterations until next time server query
unsigned long
  lastPolledTime  = 0L, // Last value retrieved from time server
  sketchTime      = 0L; // CPU milliseconds since last server query



void setup(void)
{
  // Initialize
  Serial.begin(115200);

  displayDriverMode();
 
  Serial.println(F("\nInitializing..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while(1);
  }
 
 
  uint16_t firmware = checkFirmwareVersion();
  if ((firmware != 0x113) && (firmware != 0x118)) {
    Serial.println(F("Wrong firmware version!"));
    for(;;);
  }
 
    displayMACAddress();
 
   // Provide ground and power by using the analog inputs as normal digital pins.
  // This makes it possible to directly connect the breakout board to the
  // Arduino. If you use the normal 5V and GND pins on the Arduino,
  // you can remove these lines.
  pinMode(groundpin, OUTPUT);
  pinMode(powerpin, OUTPUT);
  digitalWrite(groundpin, LOW);
  digitalWrite(powerpin, HIGH);
 
 
  // Connect to WiFi network
  cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY);//home
  //cc3000.con  nectToAP(WLAN_SSID,WLAN_PASS, WLAN_SECURITY);//slu
  Serial.println(F("Connected!"));
 
  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(100);
  }
 
 /* Display the IP address DNS, Gateway, etc. */ 
  while (!displayConnectionDetails()) {
     (1000);
  }
 
  // Get the website IP & print it
  ip = 0;
  Serial.print(WEBSITE); Serial.print(F(" -> "));
  while (ip == 0) {
    if (! cc3000.getHostByName(WEBSITE, &ip)) {
      Serial.println(F("Couldn't resolve!"));
    }
    delay(500);
  }
  cc3000.printIPdotsRev(ip);
 

 
}
String z_accel;
String x_accel;
String y_accel;
void loop(void)
{
 
  Serial.println("shoot.");
  if(countdown == 0) {            // Time's up?
    unsigned long t  = getTime(); // Query time server
    if(t) {                       // Success?
      lastPolledTime = t;         // Save time
      sketchTime     = millis();  // Save sketch time of last valid time query
      countdown      = 24*60*4-1; // Reset counter: 24 hours * 15-second intervals
    }
  } else {
    countdown--;                  // Don't poll; use math to figure current time
  }

  unsigned long currentTime = lastPolledTime + (millis() - sketchTime) / 1000;

  //Serial.print(F("Current UNIX time: "));
  //Serial.print(currentTime);
  //Serial.println(F(" (seconds since 1/1/1970 UTC)"));

 
 
  // Prepare JSON for Carriots & get length
  int length = 0;
  int state = 1;
  String x_accel = doubleToString(analogRead(xpin),1);
  //Serial.print("X: " +x_accel);
  String y_accel = doubleToString(analogRead(ypin),1);
  //Serial.print("Y: " +y_accel);
  String z_accel = doubleToString(analogRead(zpin),1);
  //Serial.print("Z: " +z_accel);
 String currentDate = "2018-04-30";
 
//String data = "{\"x\":" + x_accel +",\"y\":"+ y_accel +",\"z\":"+ z_accel +",\"@timestamp\":\"2019-11-11 13:06:11\"}";
  String data = "{\"x\":" + x_accel +",\"y\":"+ y_accel +",\"z\":"+ z_accel +",\"timestamp\":\""+currentTime + "\"}";
 
  // This is the index of the Elasticsearch document we're creating
//Example:  String url = "/weather/reading";
  String url = "/accelerometer_data_sec2/_doc";
  length = data.length();
 
  // Send request
  Adafruit_CC3000_Client client = cc3000.connectTCP(ip, 9200);
  if (client.connected()) {
    Serial.println("Connected!");
    client.println("POST " + url + " HTTP/1.1");
   // client.println("Host: api.carriots.com");
    client.println("Accept: application/json");
    client.println("Content-Type: application/json");
    client.println("Content-Length: " + String(length));
    client.println("Connection: close");
    client.println();
   
    client.println(data);
   
  } else {
    Serial.println(F("Connection failed"));
    return;
  }
 
  Serial.println(F("-------------------------------------"));
  while (client.connected()) {
    while (client.available()) {
      char c = client.read();
      Serial.print(c);
    }
  }
  client.close();
  
}

// Convert double to string
String doubleToString(float input,int decimalPlaces){
  if(decimalPlaces!=0){
    String string = String((int)(input*pow(10,decimalPlaces)));
      if(abs(input)<1){
        if(input>0)
          string = "0"+string;
        else if(input<0)
          string = string.substring(0,1)+"0"+string.substring(1);
      }
      return string.substring(0,string.length()-decimalPlaces)+"."+string.substring(string.length()-decimalPlaces);
    }
  else {
    return String((int)input);
  }
}



/**************************************************************************/
/*!
    @brief  Displays the driver mode (tiny of normal), and the buffer
            size if tiny mode is not being used

    @note   The buffer size and driver mode are defined in cc3000_common.h
*/
/**************************************************************************/
void displayDriverMode(void)
{
  #ifdef CC3000_TINY_DRIVER
    Serial.println(F("CC3000 is configure in 'Tiny' mode"));
  #else
    Serial.print(F("RX Buffer : "));
    Serial.print(CC3000_RX_BUFFER_SIZE);
    Serial.println(F(" bytes"));
    Serial.print(F("TX Buffer : "));
    Serial.print(CC3000_TX_BUFFER_SIZE);
    Serial.println(F(" bytes"));
  #endif
}

/**************************************************************************/
/*!
    @brief  Tries to read the CC3000's internal firmware patch ID
*/
/**************************************************************************/
uint16_t checkFirmwareVersion(void)
{
  uint8_t major, minor;
  uint16_t version;
 
#ifndef CC3000_TINY_DRIVER 
  if(!cc3000.getFirmwareVersion(&major, &minor))
  {
    Serial.println(F("Unable to retrieve the firmware version!\r\n"));
    version = 0;
  }
  else
  {
    Serial.print(F("Firmware V. : "));
    Serial.print(major); Serial.print(F(".")); Serial.println(minor);
    version = major; version <<= 8; version |= minor;
  }
#endif
  return version;
}

/**************************************************************************/
/*!
    @brief  Tries to read the 6-byte MAC address of the CC3000 module
*/
/**************************************************************************/
void displayMACAddress(void)
{
  uint8_t macAddress[6];
 
  if(!cc3000.getMacAddress(macAddress))
  {
    Serial.println(F("Unable to retrieve MAC Address!\r\n"));
  }
  else
  {
    Serial.print(F("MAC Address : "));
    cc3000.printHex((byte*)&macAddress, 6);
  }
}


/**************************************************************************/
/*!
    @brief  Tries to read the IP address and other connection details
*/
/**************************************************************************/
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
 
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}

// Minimalist time server query; adapted from Adafruit Gutenbird sketch,
// which in turn has roots in Arduino UdpNTPClient tutorial.
unsigned long getTime(void) {

  uint8_t       buf[48];
  unsigned long ip, startTime, t = 0L;

  Serial.print(F("Locating time server..."));

  // Hostname to IP lookup; use NTP pool (rotates through servers)
  if(cc3000.getHostByName("pool.ntp.org", &ip)) {
    static const char PROGMEM
      timeReqA[] = { 227,  0,  6, 236 },
      timeReqB[] = {  49, 78, 49,  52 };

    Serial.println(F("\r\nAttempting connection..."));
    startTime = millis();
    do {
      client = cc3000.connectUDP(ip, 123);
    } while((!client.connected()) &&
            ((millis() - startTime) < connectTimeout));

    if(client.connected()) {
      Serial.print(F("connected!\r\nIssuing request..."));

      // Assemble and issue request packet
      memset(buf, 0, sizeof(buf));
      memcpy_P( buf    , timeReqA, sizeof(timeReqA));
      memcpy_P(&buf[12], timeReqB, sizeof(timeReqB));
      client.write(buf, sizeof(buf));

      Serial.print(F("\r\nAwaiting response..."));
      memset(buf, 0, sizeof(buf));
      startTime = millis();
      while((!client.available()) &&
            ((millis() - startTime) < responseTimeout));
      if(client.available()) {
        client.read(buf, sizeof(buf));
        t = (((unsigned long)buf[40] << 24) |
             ((unsigned long)buf[41] << 16) |
             ((unsigned long)buf[42] <<  8) |
              (unsigned long)buf[43]) - 2208988800UL;
        Serial.print(F("OK\r\n"));
      }
      client.close();
    }
  }
  if(!t) Serial.println(F("error"));
  return t;
}

