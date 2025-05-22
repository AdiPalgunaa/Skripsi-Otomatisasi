#include "env.h"
#include <WiFi.h>
#include <Firebase.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DS18B20.h"
#include <jsnsr04t.h>
#include <time.h>

// Sensor pins dan setup
#define LCD_ADDRESS 0x27
#define SDA_PIN 17
#define SCL_PIN 16
LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);
// Karakter kustom untuk simbol derajat
byte degreeChar[8] = {
  0b00110,
  0b01001,
  0b01001,
  0b00110,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

// TDS Sensor Configuration
#define TDS_PIN 36
#define VREF 3.3        // Menggunakan 3.3V karena ESP32
#define SCOUNT 30       // Jumlah sampel untuk averaging

// Temperature Sensor
#define ONE_WIRE_BUS 14
OneWire oneWire(ONE_WIRE_BUS);
DS18B20 sensor(&oneWire);

// Buzzer
#define BUZZER_PIN 33

// Ultrasonic Sensors
#define ECHO_PIN_1 18
#define TRIGGER_PIN_1 5
#define ECHO_PIN_2 21
#define TRIGGER_PIN_2 19
JsnSr04T ultrasonicSensor1(ECHO_PIN_1, TRIGGER_PIN_1, LOG_LEVEL_VERBOSE);
JsnSr04T ultrasonicSensor2(ECHO_PIN_2, TRIGGER_PIN_2, LOG_LEVEL_VERBOSE);

// Relay pins
const int relayA = 26;
const int relayB = 27;

// Variabel global
int analogBuffer[SCOUNT];
int analogBufferIndex = 0;
float averageVoltage = 0;
int tdsValue = 0;
float temperature;
int distance1 = 0, distance2 = 0;
String relayAStatus = "OFF";
String relayBStatus = "OFF";
const int NUM_READINGS = 20;

unsigned long lastRealtimeUpdate = 0;
unsigned long analogSampleTimepoint = 0;
unsigned long lastWifiCheckTime = 0;
const unsigned long WIFI_CHECK_INTERVAL = 10000; // Cek koneksi WiFi setiap 10 detik

// Variabel untuk standby mode relay
unsigned long lastRelayActivationTime = 0;
const unsigned long RELAY_COOLDOWN_PERIOD = 5 * 60 * 1000; // 5 menit dalam milidetik

Firebase Firebase(FIREBASE_HOST, FIREBASE_AUTH);

void setup() {
    Serial.begin(115200);
    
    Wire.begin(SDA_PIN, SCL_PIN);
    lcd.init();
    lcd.backlight();

    lcd.createChar(0, degreeChar);
    
    pinMode(relayA, OUTPUT);
    pinMode(relayB, OUTPUT);
    digitalWrite(relayA, LOW);
    digitalWrite(relayB, LOW);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    
    sensor.begin();
    sensor.setResolution(12);
    ultrasonicSensor1.begin(Serial);
    ultrasonicSensor2.begin(Serial);

    // Inisialisasi TDS sensor
    pinMode(TDS_PIN, INPUT);

    // Koneksi WiFi
    connectWifi();

    // Sinkronisasi waktu
    configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    Serial.println("Menunggu sinkronisasi waktu");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Syncing time...");
    time_t now;
    struct tm timeinfo;
    do {
        delay(500);
        Serial.print(".");
        now = time(nullptr);
        localtime_r(&now, &timeinfo);
    } while (now < 1680000000);
    Serial.println();
    Serial.print("Waktu terkini: ");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Waktu terkini:");
    char timeStr[20];
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
    Serial.println(timeStr);
    lcd.setCursor(0, 1);
    lcd.print(timeStr);
}

bool connectWifi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Connecting WiFi");
    
    unsigned long startAttemptTime = millis();
    
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 30000) {
        Serial.print(".");
        lcd.setCursor(0, 1);
        lcd.print("Trying: ");
        lcd.print((millis() - startAttemptTime) / 1000);
        lcd.print("s");
        delay(300);
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.print("Connected with IP: ");
        Serial.println(WiFi.localIP());
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("WiFi Connected!");
        lcd.setCursor(0, 1);
        lcd.print(WiFi.localIP());
        delay(1000);
        return true;
    } else {
        Serial.println();
        Serial.println("Connection failed");
        lcd.print("Connection Error");
        return false;
    }
}

bool checkWifiConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected. Attempting to reconnect...");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Connection Error");
        lcd.setCursor(0, 1);
        lcd.print("Reconnecting...");
        
        // Attempt to reconnect for 30 seconds
        bool reconnected = connectWifi();
        
        if (!reconnected) {
            // If still not connected after 30 seconds, display error and activate buzzer
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Connection error");
            lcd.setCursor(0, 1);
            lcd.print("Please Restart!");
            
            // Activate buzzer in a non-blocking way
            while (true) { 
                digitalWrite(BUZZER_PIN, HIGH);
                delay(500);
                digitalWrite(BUZZER_PIN, LOW);
                delay(500);
            }
            
            return false;
        }
        return true;
    }
    return true;
}

void processTDS() {
    // Baca TDS 10 kali dengan delay kecil antar pembacaan
    for(int i = 0; i < NUM_READINGS; i++) {
        // Baca analog value
        analogBuffer[0] = analogRead(TDS_PIN);
        
        // Hitung voltage
        averageVoltage = (analogBuffer[0] * VREF) / 4095.0;
        
        // Kompensasi suhu
        float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
        float compensationVoltage = averageVoltage / compensationCoefficient;

        // Konversi ke TDS value
        tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage 
                - 255.86 * compensationVoltage * compensationVoltage 
                + 857.39 * compensationVoltage) * 0.5;
        
        Serial.print("TDS Reading ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(tdsValue);
        Serial.println(" ppm");
        
        delay(100); // Delay 100ms antar pembacaan
    }
    
    Serial.print("Final TDS Value: ");
    Serial.print(tdsValue);
    Serial.println(" ppm");
}

void sendRealtimeData(float temperature, float tds, float dist1, float dist2) {
    // Check WiFi connection before sending data
    if (!checkWifiConnection()) {
        Serial.println("Cannot send data: WiFi not connected");
        return;
    }
    
    String realtimePath = "MonitoringNutrisi/realtime/";
    Firebase.setFloat(realtimePath + "temperaturetds", temperature);
    Firebase.setFloat(realtimePath + "tdsValue", tds);
    Firebase.setFloat(realtimePath + "distance1", dist1);
    Firebase.setFloat(realtimePath + "distance2", dist2);
    Firebase.setString(realtimePath + "relayA", relayAStatus);
    Firebase.setString(realtimePath + "relayB", relayBStatus);
    Serial.println("Data realtime terkirim ke Firebase");
}

void sendHistoryData(float temperature, float tds, float dist1, float dist2) {
    // Check WiFi connection before sending data
    if (!checkWifiConnection()) {
        Serial.println("Cannot send history data: WiFi not connected");
        return;
    }
    
    time_t now = time(nullptr);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    char dateString[11];
    char hourString[6];
    strftime(dateString, sizeof(dateString), "%Y-%m-%d", &timeinfo);
    strftime(hourString, sizeof(hourString), "%H:%M", &timeinfo);
    
    String historyPath = "MonitoringNutrisi/history/" + String(dateString) + "/" + String(hourString) + "/";
    Firebase.setFloat(historyPath + "temperaturetds", temperature);
    Firebase.setFloat(historyPath + "tdsValue", tds);
    Firebase.setFloat(historyPath + "distance1", dist1);
    Firebase.setFloat(historyPath + "distance2", dist2);
    Firebase.setString(historyPath + "relayA", relayAStatus);
    Firebase.setString(historyPath + "relayB", relayBStatus);
    Serial.println("Data history tersimpan");
}

void checkSensorError() {
    // Cek error sensor TDS
    if (tdsValue < 0) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("TDS Error!");
        lcd.setCursor(0, 1);
        lcd.print("Please restart!");
        // Infinite loop untuk buzzer hidup dan mati
        while (true) {
            digitalWrite(BUZZER_PIN, HIGH);  // Nyalakan buzzer
            delay(1000);                     // Buzzer menyala selama 1 detik
            digitalWrite(BUZZER_PIN, LOW);   // Matikan buzzer
            delay(1000);                     // Buzzer mati selama 1 detik
        }
    }

    // Cek error sensor suhu
    if (temperature < 0) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Temp Error!");
        lcd.setCursor(0, 1);
        lcd.print("Please restart!");
        // Infinite loop untuk buzzer hidup dan mati
        while (true) {
            digitalWrite(BUZZER_PIN, HIGH);  // Nyalakan buzzer
            delay(1000);                     // Buzzer menyala selama 1 detik
            digitalWrite(BUZZER_PIN, LOW);   // Matikan buzzer
            delay(1000);                     // Buzzer mati selama 1 detik
        }
    }

     if (distance1 < 0) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("TA Error!");
        lcd.setCursor(0, 1);
        lcd.print("Please restart!");
        // Infinite loop untuk buzzer hidup dan mati
        while (true) {
            digitalWrite(BUZZER_PIN, HIGH);  // Nyalakan buzzer
            delay(1000);                     // Buzzer menyala selama 1 detik
            digitalWrite(BUZZER_PIN, LOW);   // Matikan buzzer
            delay(1000);                     // Buzzer mati selama 1 detik
        }
    }
    
    // Cek error sensor ultrasonik JSN kedua
    if (distance2 < 0) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("TB Error!");
        lcd.setCursor(0, 1);
        lcd.print("Please restart!");
        // Infinite loop untuk buzzer hidup dan mati
        while (true) {
            digitalWrite(BUZZER_PIN, HIGH);  // Nyalakan buzzer
            delay(1000);                     // Buzzer menyala selama 1 detik
            digitalWrite(BUZZER_PIN, LOW);   // Matikan buzzer
            delay(1000);                     // Buzzer mati selama 1 detik
        }
    }
}

void checkControlIoT() {
    // Check WiFi connection first
    if (!checkWifiConnection()) {
        return;
    }
    
    String controlPath = "MonitoringNutrisi/controlIoT/";

    // Baca status pompa dari Firebase
    String relayACommand = Firebase.getString(controlPath + "relayA");
    String relayBCommand = Firebase.getString(controlPath + "relayB");

    // Kontrol pompa berdasarkan perintah dari Firebase
    if (relayACommand == "true" && relayBCommand == "true") {
      digitalWrite(relayA, HIGH);
      digitalWrite(relayB, HIGH);

      relayAStatus = "ON";
      relayBStatus = "ON";
      if (relayAStatus == "ON" || relayBStatus == "ON") {
          sendHistoryData(temperature, tdsValue, distance1, distance2);
      }
      
      delay(10000); // Hidup selama 10 detik

      relayAStatus = "OFF";
      relayBStatus = "OFF";
      
      digitalWrite(relayA, LOW);
      digitalWrite(relayB, LOW);
      
      Firebase.setBool(controlPath + "relayA", false); // Reset status di Firebase
      Firebase.setBool(controlPath + "relayB", false); // Reset status di Firebase

      // Update the last activation time untuk standby mode
      lastRelayActivationTime = millis();
    }
}

void restartESP() {
    // Check WiFi connection first
    if (!checkWifiConnection()) {
        return;
    }
    
    String controlPath = "MonitoringNutrisi/controlIoT/restart";
    
    // Baca status restart dari Firebase
    String restartCommand = Firebase.getString(controlPath);
    
    if (restartCommand == "true") {
        Serial.println("Restarting ESP...");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Restarting ESP...");
        

        Firebase.setBool(controlPath, false); // Reset status di Firebase agar tidak terus menerus restart

        delay(500); //Delay proses firebase

        delay(2000);  // Delay sebentar sebelum restart
        ESP.restart(); // Restart ESP32
    }
}

void loop() {
    // Check WiFi connection periodically
    if (millis() - lastWifiCheckTime > WIFI_CHECK_INTERVAL) {
        checkWifiConnection();
        lastWifiCheckTime = millis();
    }
    
    checkSensorError(); // Cek error sensor
    restartESP();
    checkControlIoT();  // Periksa kontrol IoT dari Firebase
    
    relayAStatus = "OFF";
    relayBStatus = "OFF";
    
    // Baca suhu
    sensor.requestTemperatures();
    temperature = sensor.getTempC();
    
    // Proses TDS
    processTDS();
    
    // Baca sensor ultrasonic
    distance1 = ultrasonicSensor1.readDistance();
    delay(50);
    distance2 = ultrasonicSensor2.readDistance();

    lcd.clear();
    lcd.setCursor(0, 0); // Baris pertama
    lcd.print(temperature, 1); // Tampilkan suhu
    lcd.write(byte(0));
    lcd.print("C");
    lcd.setCursor(8, 0); // Pindah ke kolom 13, baris pertama
    lcd.print("TA=");
    lcd.print(distance1, 1); // Tampilkan jarak ultrasonik 1
    lcd.print("CM");

    lcd.setCursor(0, 1); // Baris kedua
    lcd.print(tdsValue, 0); // Tampilkan nilai TDS
    lcd.print("ppm");
    lcd.setCursor(8, 1); // Pindah ke kolom 13, baris kedua
    lcd.print("TB=");
    lcd.print(distance2, 1); // Tampilkan jarak ultrasonik 2
    lcd.print("CM");

    // Kontrol relay berdasarkan TDS dengan standby mode
    if (tdsValue > 1 && tdsValue < 560) {
        // Check if cooldown period has passed
        if (millis() - lastRelayActivationTime > RELAY_COOLDOWN_PERIOD) {
            digitalWrite(relayA, HIGH);
            digitalWrite(relayB, HIGH);
            relayAStatus = "ON";
            relayBStatus = "ON";
            if (relayAStatus == "ON" || relayBStatus == "ON") {
                sendHistoryData(temperature, tdsValue, distance1, distance2);
            }
            delay(10000);
            relayAStatus = "OFF";
            relayBStatus = "OFF";
            digitalWrite(relayA, LOW);
            digitalWrite(relayB, LOW);
            
            // Update the last activation time
            lastRelayActivationTime = millis();
            
            // Log to serial
            Serial.println("Relays activated. Now entering 5-minute standby period.");
        } else {
            // Relays are in standby mode - only log to serial
            unsigned long remainingStandby = (RELAY_COOLDOWN_PERIOD - (millis() - lastRelayActivationTime)) / 1000;
            
            Serial.print("Relay in standby mode. ");
            Serial.print(remainingStandby);
            Serial.println(" seconds remaining before next activation allowed.");
        }
    }

    // Update data ke Firebase setiap 10 detik
    if (millis() - lastRealtimeUpdate > 10000) {
        sendRealtimeData(temperature, tdsValue, distance1, distance2);
        lastRealtimeUpdate = millis();
    }
}