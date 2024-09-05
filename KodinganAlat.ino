#include <Arduino.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <DHT.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <TimeLib.h>
#include <stdlib.h>

// WiFi and Firebase configurations
#define WIFI_SSID2 "UNAND"
#define WIFI_PASSWORD2 "HardiknasDiAndalas"
#define WIFI_SSID "Andromax22"
#define WIFI_PASSWORD "testestes"
#define FIREBASE_PROJECT_ID "taranda-2f398"
#define API_KEY "AIzaSyCbo-Bo-IiYX7d3AeTEQOOZaMtxKGYPZuU"
#define USER_EMAIL "esp32@esp.com"
#define USER_PASSWORD "taranda123"

// NTP Server
#define NTP_SERVER "time.nist.gov"

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// NTP Client
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_SERVER);

// Sensor Class
#ifndef Sensor_H_
#define Sensor_H_

#include <MQUnifiedsensor.h>

#define INTERVAL 1000 // Delay between sensor readings in milliseconds
#define UPDATE_INTERVAL 2000 // Delay between data updates in milliseconds (10 seconds)
#define UPLOAD_INTERVAL 900000 // 15 minutes in milliseconds
#define HUMIDITY_UPDATE_INTERVAL 300000 // 5 minutes in milliseconds

class Sensor
{
public:
    Sensor(); // Constructor
    void init_MQ135();
    void init_MQ136();
    void init_MQ7();
    float get_MQ135();
    float get_MQ136();
    float get_MQ7();
    void uploadData(FirebaseData &fbdo, float nh3, float h2s, float co, int temp, int humid, bool exhaustfan, bool toilet);
    void updateData(FirebaseData &fbdo, float nh3, float h2s, float co, int temp, int humid, bool exhaustfan, bool toilet);

public:
    unsigned long lastUploadTime = 0;
    unsigned long lastUpdateTime = 0;
    unsigned long initialStartTime = 0;
    bool toiletState = false;
private:
    MQUnifiedsensor MQ135;
    MQUnifiedsensor MQ136;
    MQUnifiedsensor MQ7;
    DHT dht;
};

#endif // !Sensor_H_

#include "Sensor.h"

#define board "ESP32"
#define voltageresol 3.3
#define ADC_Bit_Resolution 12
#define MQ135_TYPE "MQ-135"
#define MQ136_TYPE "MQ-136"
#define MQ7_TYPE "MQ-7"
#define RatioMQ135CleanAir 3.6
#define RatioMQ136CleanAir 3.6
#define RatioMQ7CleanAir 27.5
#define PIN_MQ135 32
#define PIN_MQ136 35
#define PIN_MQ7 34
#define DHTTYPE DHT11
#define PINdht 26
#define RELAY_PIN 25

DHT dht(PINdht, DHTTYPE);
#define WARMUP_TIME 60000
Sensor::Sensor() : MQ135(board, voltageresol, ADC_Bit_Resolution, PIN_MQ135, MQ135_TYPE),
                   MQ136(board, voltageresol, ADC_Bit_Resolution, PIN_MQ136, MQ136_TYPE),
                   MQ7(board, voltageresol, ADC_Bit_Resolution, PIN_MQ7, MQ7_TYPE),
                   dht(PINdht, DHTTYPE)
{
}

void Sensor::init_MQ135()
{
    MQ135.setRegressionMethod(1); // Use least squares method to calculate slope and y-intercept.
    MQ135.setA(102.2); // Set equation to get NH3 concentration.
    MQ135.setB(-2.473); // Set equation to get NH3 concentration.

    MQ135.init(); // Initialize sensor.
    Serial.print("Calibrating MQ135, please wait.");
    float calcR0 = 0;
    for (int i = 1; i <= 10; i++)
    {
        MQ135.update(); // Update data
        calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    }
    MQ135.setR0(20);
    //MQ135.setR0(calcR0/10);
    Serial.println("  done!");

    if (isinf(calcR0)){
        Serial.println("Warning: Connection problem, R0 is infinity (Detected open circuit), please check your wiring and power supply.");
        while (1);
    }
    if (calcR0 == 0){
        Serial.println("Warning: Connection problem, R0 is zero (Analog pin connected to ground), please check your wiring and power supply.");
        while (1);
    }
    MQ135.serialDebug(true);
}

void Sensor::init_MQ136()
{
    MQ136.setRegressionMethod(1); //_PPM = a*ratio^b
    MQ136.setA(36.737); // Set equation to calculate H2S concentration.
    MQ136.setB(-3.536); // Set equation to calculate H2S concentration.

    MQ136.init(); // Initialize sensor.
    Serial.print("Calibrating MQ136, please wait.");
    float calcR0 = 0;
    for (int i = 1; i <= 10; i++) 
    {
        MQ136.update(); // Update data, Arduino will read voltage from analog pin
        calcR0 += MQ136.calibrate(RatioMQ136CleanAir);
    }
    MQ136.setR0(37.50);
    //MQ136.setR0(calcR0 / 10);
    Serial.println("  done!");

    if (isinf(calcR0)){
        Serial.println("Warning: Connection problem, R0 is infinity (Detected open circuit), please check your wiring and power supply.");
        while (1);
    }
    if (calcR0 == 0){
        Serial.println("Warning: Connection problem, R0 is zero (Analog pin connected to ground), please check your wiring and power supply.");
        while (1);
    }
    MQ136.serialDebug(true);
}

void Sensor::init_MQ7()
{
    MQ7.setRegressionMethod(1); // Use least squares method to calculate slope and y-intercept.
    MQ7.setA(99.042); // Set equation to get CO concentration.
    MQ7.setB(-1.518); // Set equation to get CO concentration.

    MQ7.init(); // Initialize sensor.

    Serial.print("Calibrating MQ7, please wait.");
    float calcR0 = 0;
    for (int i = 1; i <= 10; i++)
    {
        MQ7.update(); // Update data, Arduino will read voltage from analog pin
        calcR0 += MQ7.calibrate(RatioMQ7CleanAir);
    }
    MQ7.setR0(17.60);
    //MQ7.setR0(calcR0 / 10);
    Serial.println("  done!");

    if (isinf(calcR0)){
        Serial.println("Warning: Connection problem, R0 is infinity (Detected open circuit), please check your wiring and power supply.");
        while (1);
    }
    if (calcR0 == 0){
        Serial.println("Warning: Connection problem, R0 is zero (Analog pin connected to ground), please check your wiring and power supply.");
        while (1);
    }
    MQ7.serialDebug(true);
}

float Sensor::get_MQ135()
{
    MQ135.update();
    return MQ135.readSensor();
}

float Sensor::get_MQ136()
{
    MQ136.update();
    return MQ136.readSensor();
}

float Sensor::get_MQ7()
{
    MQ7.update();
    return MQ7.readSensor();
}

String getTanggalWaktu()
{
    timeClient.update();
    unsigned long rawTime = timeClient.getEpochTime();
    time_t t = rawTime + (7 * 3600);

    int jam = hour(t);
    String jamStr = jam < 10 ? "0" + String(jam) : String(jam);

    int menit = minute(t);
    String menitStr = menit < 10 ? "0" + String(menit) : String(menit);

    int detik = second(t);
    String detikStr = detik < 10 ? "0" + String(detik) : String(detik);

    int tgl = day(t);
    String tglStr = tgl < 10 ? "0" + String(tgl) : String(tgl);

    int bln = month(t);
    String blnStr = bln < 10 ? "0" + String(bln) : String(bln);

    int thn = year(t);
    String thnStr = String(thn);

    String tanggal = tglStr + "/" + blnStr + "/" + thnStr;
    String waktu = jamStr + ":" + menitStr + ":" + detikStr;

    return tanggal + " " + waktu;
}

void Sensor::updateData(FirebaseData &fbdo, float nh3, float h2s, float co, int temp, int humid, bool exhaustfan, bool toilet)
{
    if (WiFi.status() == WL_CONNECTED && Firebase.ready())
    {
        String tanggalWaktu = getTanggalWaktu();
        FirebaseJson data;

        String documentPath = "gedungh/monitoring";
        String mask = "Tanggal, Amonia, HidrogenSulfida, KarbonMonoksida, Suhu, Kelembapan, Exhaust, Toilet";

        data.set("fields/Tanggal/stringValue", tanggalWaktu.substring(0, 10));
        data.set("fields/Amonia/doubleValue", nh3);
        data.set("fields/HidrogenSulfida/doubleValue", h2s);
        data.set("fields/KarbonMonoksida/doubleValue", co);
        data.set("fields/Suhu/integerValue", temp);
        data.set("fields/Kelembapan/integerValue", humid);
        data.set("fields/Exhaust/booleanValue", exhaustfan);
        data.set("fields/Toilet/booleanValue", toilet);
        
        if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), data.raw(), mask.c_str()))
        {
            Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
        }
        else
        {
            Serial.println(fbdo.errorReason());
        }
    }
}

void Sensor::uploadData(FirebaseData &fbdo, float nh3, float h2s, float co, int temp, int humid, bool exhaustfan, bool toilet)
{
    if (WiFi.status() == WL_CONNECTED && Firebase.ready())
    {
        String tanggalWaktu = getTanggalWaktu(); // Get actual date and time

        FirebaseJson data;

        String documentPath = "gedungh/History/" + tanggalWaktu.substring(6, 10) + "-" + tanggalWaktu.substring(3, 5) + "-" + tanggalWaktu.substring(0, 2);

        data.set("fields/Tanggal/stringValue", tanggalWaktu.substring(0, 10));
        data.set("fields/Jam/stringValue", tanggalWaktu.substring(11));
        data.set("fields/Amonia/doubleValue", nh3);
        data.set("fields/HidrogenSulfida/doubleValue", h2s);
        data.set("fields/KarbonMonoksida/doubleValue", co);
        data.set("fields/Suhu/integerValue", temp);
        data.set("fields/Kelembapan/integerValue", humid);
        data.set("fields/Exhaustfan/booleanValue", exhaustfan);
        data.set("fields/Toilet/booleanValue", toilet);

        if (Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), data.raw()))
        {
            Serial.print("ok\n%s\n\n");
            Serial.println(fbdo.payload());
        }
        else
        {
            Serial.println("Failed to send data to Firestore");
            Serial.println(fbdo.errorReason());
        }
    }
}

Sensor sensor; // Create an instance of Sensor class

void connectToWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi ");
    Serial.println(WIFI_SSID);

    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 5000) { // Wait for 10 seconds
        Serial.print(".");
        delay(500);
    }

    if (WiFi.status() != WL_CONNECTED) {
        WiFi.begin(WIFI_SSID2, WIFI_PASSWORD2);
        startTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startTime < 5000) { // Wait for 10 seconds
            Serial.print(".");
            delay(500);
        }
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.print("Connected with IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("Failed to connect to WiFi");
    }
}

void setup() {
    Serial.begin(9600); // Initialize serial communication
    sensor.init_MQ135();
    sensor.init_MQ136();
    sensor.init_MQ7();
    dht.begin();
    pinMode(RELAY_PIN, OUTPUT);

    WiFi.mode(WIFI_STA);
    connectToWiFi();
    // Setup Firebase configuration
    config.api_key = API_KEY;
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;
    config.token_status_callback = tokenStatusCallback;

    Firebase.reconnectWiFi(true);
    fbdo.setResponseSize(4096);
    Firebase.begin(&config, &auth);
    // Initialize NTPClient to get time from NTP server
    timeClient.begin();
    sensor.initialStartTime = millis();
}

void loop() {
    unsigned long currentTime = millis();

    // Get current date and time
    String tanggalWaktu = getTanggalWaktu();
    int currentHour = tanggalWaktu.substring(11, 13).toInt(); // Extract hour from time string

    // Process sensor data only within the desired time range
    if (currentHour >= 8 && currentHour <= 17){
        float nh3 = sensor.get_MQ135();
        float h2s = sensor.get_MQ136();
        float co = sensor.get_MQ7();
        int temp = dht.readTemperature();
        int humid = dht.readHumidity(); 

        bool exhaustfan = false;
        bool toilet = false;
        
        
        if (co >= 2.5) {
            if (nh3 > 5) {
                exhaustfan = true;
                toilet = true;
            } else {
                exhaustfan = true;
            }
        } else if ((nh3 > 2.7 || h2s >= 2.5) && co < 1.5) {
            exhaustfan = true;
            toilet = true;
        } else if ((nh3 >= 2.2 || h2s > 2) && co < 1.5) {
            toilet = true;
        } else if (temp < 20 || temp > 30) {
            exhaustfan = true;
        } else if (humid < 39 || humid > 50) {
            exhaustfan = true;
        }

        if (exhaustfan) {
            digitalWrite(RELAY_PIN, LOW); // Turn on the exhaust fan
        } else {
            digitalWrite(RELAY_PIN, HIGH); // Turn off the exhaust fan
        }

        if (currentTime - sensor.initialStartTime >= WARMUP_TIME) {
            if (currentTime - sensor.lastUpdateTime >= UPDATE_INTERVAL) {
                sensor.updateData(fbdo, nh3, h2s, co, temp, humid, exhaustfan, toilet);
                sensor.lastUpdateTime = currentTime;
            }

            if (toilet != sensor.toiletState || currentTime - sensor.lastUploadTime >= UPLOAD_INTERVAL) {
                sensor.uploadData(fbdo, nh3, h2s, co, temp, humid, exhaustfan, toilet);
                sensor.toiletState = toilet;
                sensor.lastUploadTime = currentTime;
            }
        }
    } 

    if (WiFi.status() != WL_CONNECTED || !Firebase.ready()) {
        connectToWiFi();
    }
    
    delay(INTERVAL); // Delay between readings
}
