
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MAX30105.h"
#include "heartRate.h"
#define outPin 8
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 8
#define DHTTYPE    DHT11
DHT_Unified dht(DHTPIN, DHTTYPE);
MAX30105 particleSensor;
LiquidCrystal_I2C lcd(0x27,16,2);
int RXPin = 3;
int TXPin = 2;
int GPSBaud = 9600;
float tempthresholdH = 28.00;
float tempthresholdL = 25.00;
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
SoftwareSerial gpsSerial(RXPin, TXPin);
SoftwareSerial espSerial(10, 11);
String str;
TinyGPSPlus gps;


void setup() {
  // put your setup code here, to run once:
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Initialising");
  delay(500);
  lcd.clear();
  Serial.begin(115200);
  Serial.println("Initialising");
  espSerial.begin(115200);
	// Initialize sensor
	if (particleSensor.begin(Wire, I2C_SPEED_FAST) == false) {
		Serial.println("MAX30102 was not found. Please check wiring/power.");
		// while (1);
	}
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  // particleSensor.enableDIETEMPRDY();
  gpsSerial.begin(GPSBaud);
  dht.begin();
}

void loop() {
  
  // put your main code here, to run repeatedly:
  long irValue = particleSensor.getIR();
  if (checkForBeat(irValue) == true) {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  str = str+ beatAvg;

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
  Serial.println();
  if (irValue < 50000)
  {
    lcd.setCursor(0, 0);
    lcd.print("No finger?      ");
  }
  else
  {

    lcd.setCursor(0, 0);
    lcd.print("BPM=");
    lcd.print(beatAvg);
    
    if(beatAvg < 10){
      lcd.setCursor(5,0);
      lcd.print("            ");
    }
    else if (beatAvg < 100){
      lcd.setCursor(6,0);
      lcd.print("          ");
    }
    else{
      lcd.setCursor(7,0);
      lcd.print("         ");
      
    }
  }
   
  // float temperature = particleSensor.readTemperature();
  // Serial.print("temperatureC=");
  // Serial.print(temperature, 4);
  
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    // Serial.print(F("Temperature: "));
    // Serial.print(event.temperature);
    // Serial.println(F("°C"));
    lcd.setCursor(0, 1);
  lcd.print("Temp : ");
  lcd.print(event.temperature);
  lcd.print(" C");


  
  if(event.temperature>tempthresholdH){
    digitalWrite(6, LOW);
    digitalWrite(5, HIGH);
  }
  else{
    digitalWrite(6, HIGH);
    digitalWrite(5, LOW);
  }
  // else if(event.temperature>tempthresholdH){
  //   digitalWrite(5, HIGH);
  //   digitalWrite(6, LOW);
  // } 
  }
  // Get humidity event and print its value.
  
  // Serial.println();

  str = str+ " 72.91242 82.89293" ;
  
  // Serial.println();
  // espSerial.println(str);
  if(gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read()))
      // displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected");
    // while(true);
  }
}

void displayInfo()
{
  if (gps.location.isValid())
  {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());
  }
  else
  {
    Serial.println("Location: Not Available");
  }
  
  Serial.print("Date: ");
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.println(gps.date.year());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.print("Time: ");
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(".");
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.println(gps.time.centisecond());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.println();
  Serial.println();
  // delay(1000);
}