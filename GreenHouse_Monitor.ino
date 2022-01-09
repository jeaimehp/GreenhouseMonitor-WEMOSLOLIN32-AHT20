/**********************************************
 * Program Name: GreenHouse Monitor
 * Input: AHT20 
 * Output: TM1637
 * Microcontroller: WEMOS LOLIN32
 */
#include <Adafruit_AHTX0.h>
#include <Arduino.h>
#include <TM1637Display.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// Sensor Name
String sensor_name = "Greenhouse"; //Sensor Assigned Name


// Module connection pins (GPIO/Digital Pins)
#define CLK 16
#define DIO 17

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 5 // on pin 5 (a 4.7K resistor is necessary)

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
 
// arrays to hold device address
DeviceAddress insideThermometer;

TM1637Display display(CLK, DIO);

Adafruit_AHTX0 aht;

float tempC;

// WIFI
// Replace with your network credentials
const char* ssid     = "ReadyPlayer2";
const char* password = "Itiswhatitis!";
String ipaddress;


// MQTT
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "Jeaimehp"
#define AIO_KEY         "aio_LDJG993NtS1vEGchpaP5K0SfvbK0"

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Setup a feed called 'balcony' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish greenhouse_temp1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/greenhouse_temperature1");
Adafruit_MQTT_Publish greenhouse_temp2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/greenhouse_tempurature2");
Adafruit_MQTT_Publish greenhouse_humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/greenhouse_humidty");

// Setup a feed called 'sensorsitrep' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish sensorsitrep = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sensorsitrep");

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();

// Timming
const unsigned long mqtt_time = 120000;

unsigned long current_time = 0;
unsigned long previous_time = 0;

//Prototypes

void sitrep();
void senddata();


void setup() {
  Serial.begin(115200);
  Serial.println("Adafruit AHT10/AHT20 demo!");

sensors.begin();
if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
  if (! aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
  }
  Serial.println("AHT10 or AHT20 found");

  
   // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 9);
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC); 
  Serial.println();

   // WIFI
  Serial.println("Wifi Connecting");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Wifi Connected");
  // MQTT
  MQTT_connect();

    //Sitrep
  sitrep();
  delay (1500);
  
  //Intial Data
  senddata();
  
}

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  // method 1 - slower
  //Serial.print("Temp C: ");
  //Serial.print(sensors.getTempC(deviceAddress));
  //Serial.print(" Temp F: ");
  //Serial.print(sensors.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit

  // method 2 - faster
  tempC = sensors.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.println(" ");
  //Serial.print(" Temp F: ");
  //Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit


}


void loop() {
  int k; //for Display
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
   Serial.println(" ");
  Serial.println("Requesting temperatures from AHT20 .... Done");
  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
  Serial.println(" ");
  Serial.print("Temp Differential: "); Serial.print(temp.temperature - tempC); Serial.println(" degrees C");
  Serial.println(" ");
  display.setBrightness(0x0f);
  uint8_t data[] = { 0xff, 0xff, 0xff, 0xff };
  uint8_t blank[] = { 0x00, 0x00, 0x00, 0x00 };
  //display.setSegments(data);
  display.setSegments(blank);
  const uint8_t celsiusUnit[] = {
  SEG_A | SEG_B | SEG_F | SEG_G,  // Circle
  SEG_A | SEG_D | SEG_E | SEG_F   // C 
  };
  const uint8_t fahrenheitUnit[] = {
  SEG_A | SEG_G | SEG_E | SEG_F   // F 
  };
   const uint8_t humidityUnit[] = {
  SEG_B | SEG_C | SEG_G | SEG_E | SEG_F   // H 
  };
  
  //delay(500);
  display.showNumberDec((temp.temperature*1.8)+32, false, 3, 0); 
  display.setSegments(fahrenheitUnit, 1, 3);
  delay(3000);
  display.showNumberDec(humidity.relative_humidity, false, 3, 0); 
  display.setSegments(humidityUnit, 1, 3);
  delay(3000);
  
  Serial.println(" ");
  Serial.print("Requesting temperatures from DS18B20...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  
  // It responds almost immediately. Let's print out the data
  printTemperature(insideThermometer); // Use a simple function to print out the data

  current_time = millis();


  // Sends MQTT data and Inverts Screen to prevent burnin
  
  if (current_time - previous_time >= mqtt_time ) {
    // MQTT data update
    Serial.println("Sending MQTT");
    senddata();
    previous_time = current_time;
    
  }
  
  
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void sitrep() {
    MQTT_connect();
    long day = 86400000; // 86400000 milliseconds in a day
    long hour = 3600000; // 3600000 milliseconds in an hour
    long minute = 60000; // 60000 milliseconds in a minute
    //long second =  1000; // 1000 milliseconds in a second
    long timeNow = millis();
    
    int days = timeNow / day ;                                //number of days
    int hours = (timeNow % day) / hour;                       //the remainder from days division (in milliseconds) divided by hours, this gives the full hours
    //int minutes = ((timeNow % day) % hour) / minute ;         //and so on...
    //int seconds = (((timeNow % day) % hour) % minute) / second;
    String payload = sensor_name + "uptime" + days + "days " + hour + ":" + minute;
    sensorsitrep.publish((char*) payload.c_str());    
   
}

void senddata() {
    //MQTT_connect();
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
    
    //MQTT_connect();
    //String data = "{\"temp\":" + String(temp.temperature) + ",\"humidity\":" + String(humidity.relative_humidity) + "}"; 
    //balcony.publish((char*) data.c_str());
     greenhouse_temp1.publish(temp.temperature);
     greenhouse_temp2.publish(tempC);
     greenhouse_humidity.publish(humidity.relative_humidity);
    
}


// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}
