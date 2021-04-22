// ----------------------- Library -------------------------------
// Sensors
#include "CO2Sensor.h"
#include "PMS.h"
#include "SoftwareSerial.h"
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
 
#define SEALEVELPRESSURE_HPA (1013.25)
 
// Blynk 
#include <SPI.h>
#include <WiFiNINA.h>
#include <BlynkSimpleWiFiNINA.h>

#define BME_SCK 5
#define BME_MISO 4
#define BME_MOSI 3
#define BME_CS 2

// ------------------- Global Variable ---------------------------
// Sensor Instance
CO2Sensor co2Sensor(A0,0.99,100);
Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);
PMS pms(Serial1);
PMS::DATA data;

int loop_count = 0;


// WiFi
#define SECRET_SSID "Sanbundit_2.4G"
#define SECRET_PASS "foilfame"
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

// Blynk
#define BLYNK_PRINT Serial
// Local cloud
char auth[] = "Q_aLdAyInRqUQsEdxjme1l6xbwwLx_29";
int ReCnctFlag;
int ReCnctCount = 0;   
BlynkTimer timer;

String str = "";

// ----------------------------- Utils function -----------------------------
// BME680

double dewPointFast(double celsius, double humidity)
{
  double a = 17.271;
  double b = 237.7;
  double temp = (a * celsius) / (b + celsius) + log(humidity * 0.01);
  double Td = (b * temp) / (a - temp);
  return Td;
}

// -------------------------Timed Action------------------------------

int mq9_process(){
  
  int alarm = 0; 
  float sensor_volt; 
  float RS_gas; 
  float ratio; 
 // Replace the name "R0" with the value of R0 in the demo of First Test -/ 
 // float R0 = 0.73; 
  float R0 = 0.1;
 
  int sensorValue = analogRead(A0); 
  sensor_volt = ((float)sensorValue / 1024) * 5.0; 
  RS_gas = (5.0 - sensor_volt) / sensor_volt; // Depend on RL on yor module 
 
  ratio = RS_gas / R0; // ratio = RS/R0 

  //-------------Calculate CO in term of PPM-------------------/
  float base;
  float exponent;
  float ppm;
  
  base = 23.483/ratio;
  exponent = 1000.0/493.0;
  ppm = pow(base, exponent);

  Serial.println("[ MQ-9 ]");
  Serial.print("sensor_volt = "); 
  Serial.println(sensor_volt); 
  Serial.print("RS_ratio = "); 
  Serial.println(RS_gas); 
  Serial.print("Rs/R0 = "); 
  Serial.println(ratio);
  Serial.print("CO = ");
  Serial.print(ppm);
  Serial.println(" ppm");
  Serial.print("\n"); 
  str = str+ppm+",";
  
}

void bme_process(){

// Tell BME680 to begin measurement.
//  Serial.println("[ BME680 ]");
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }
  
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }
//  Serial.print(F("Reading completed at "));
//  Serial.println(millis());
 
  float temperature = bme.temperature;
  float pressure = bme.pressure / 100.0;
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float humidity = bme.humidity;
  float gas = bme.gas_resistance / 1000.0;
  double dewPoint = dewPointFast(temperature, humidity);
  
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" *C");
 
  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" hPa");
 
  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");
 
  Serial.print("Dew Point = ");
  Serial.print(dewPoint);
  Serial.println(" *C");
 
  Serial.print("Approx. Altitude = ");
  Serial.print(altitude);
  Serial.println(" m");
 
  Serial.print("Gas = ");
  Serial.print(gas);
  Serial.println(" KOhms");
 
  Serial.println();
  
//  -------------

  str = str+temperature+",";
  str = str+humidity+",";
  str = str+pressure+",";
  str = str+gas+",";
  delay(2000);
}

void pms_process(){
  pms.requestRead();
  if (pms.readUntil(data))
  {
    Serial.println("[ PMS7003 ]");
    Serial.print("PM 1.0 (ug/m3): ");
    Serial.println(data.PM_AE_UG_1_0);

    Serial.print("PM 2.5 (ug/m3): ");
    Serial.println(data.PM_AE_UG_2_5);

    Serial.print("PM 10.0 (ug/m3): ");
    Serial.println(data.PM_AE_UG_10_0);
  }
  else
  {
    Serial1.println("No data.");
  }
// Blynk
  str = str+data.PM_AE_UG_1_0+","+data.PM_AE_UG_2_5+","+data.PM_AE_UG_10_0+",";
}


// ----------------------------------------------------------------
// ----------------------------- Main -----------------------------
// ----------------------------------------------------------------

void setup() {
//  Set baud rate 
  Serial.begin(9600);
  while(!Serial) {delay(10);} // Wait
  
  Serial.println("------------ Set up--------------------");
// --------- MQ-9 ----------
  pinMode(8, INPUT); 
// -------- BME680 ----------
  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1);
  }
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

//  -------- PMS7003 ----------
  Serial1.begin(9600);
  
// --------------- WiFi + Blynk --------------
  Blynk.begin(auth, ssid, pass, "139.59.126.32", 8080);
  Serial.println("-------------------------------------");

   
}

// -------------------------------------------------------------------


void loop() {

  timer.run();
  
  if (Blynk.connected()) {  // If connected run as normal
    Blynk.run();
    Serial.print("--------------- ");
    Serial.print(loop_count++);
    Serial.println(" ---------------");
  
  
    // --------  [ MG811 : A0 Pin : C02 sensor ] --------
    mq9_process();

    // -------- [ BME680 : A4,A5 Pin : Temperature, Humidity, Pressure, Gas ] ---------
    bme_process();

    // -------- [ PMS7003 : TX,RX Pin : PM 1.0, 2.5, 10.0 ] ---------
    pms_process();

    str = str+auth;
    Blynk.virtualWrite(V10, str);
    Serial.println(str);
    str="";

    delay(5000);
  
  } else if (ReCnctFlag == 0) {
      Serial.print("WiFi.status : ");
    //  3 = connected, 6 = disconnected
  Serial.println(WiFi.status());
    // If NOT connected and not already trying to reconnect, set timer to try to reconnect in 30 seconds
    ReCnctFlag = 1;  // Set reconnection Flag
    Serial.println("Starting reconnection timer in 40 seconds...");
    timer.setTimeout(40000L, []() {  // Lambda Reconnection Timer Function
      ReCnctFlag = 0;  // Reset reconnection Flag
      ReCnctCount++;  // Increment reconnection Counter
      Serial.print("Attempting reconnection #");
      Serial.println(ReCnctCount);
      WiFi.begin(ssid, pass);
      delay(10000);
      Blynk.connect();  // Try to reconnect to the server
    });  // END Timer Function
  }
  



}
