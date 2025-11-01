#include <WiFi.h>
#include <HTTPClient.h>
#include <HardwareSerial.h>

HardwareSerial GPS(2); // UART2: RX=16, TX=17

// 🔧 Multiple WiFi Networks Configuration
const char* primarySSID = "YOUR_PRIMARY_SSID";
const char* primaryPassword = "YOUR_PRIMARY_PASSWORD";

const char* fallbackSSID = "YOUR_FALLBACK_SSID";
const char* fallbackPassword = "YOUR_FALLBACK_PASSWORD";

const char* firebaseHost = "YOUR_FIREBASE_DATABASE_URL";

// GSM Configuration
#define SerialAT Serial1
#define MODEM_TX 27
#define MODEM_RX 26
#define MODEM_RST 14
#define BUZZER_PIN 25
#define LED_PIN 13

const char* apn = "internet";  // Robi APN

// 🪑 Bus Seat Status Toggle
const int seatStatusPin = 4;
const int redLedPin = 13;
bool busSeatsAvailable = true;

// Connection status tracking
String currentConnection = "Not Connected";
String currentSSID = "Not Connected";
int connectionRetryCount = 0;
const int MAX_RETRIES = 3;
const unsigned long WIFI_TIMEOUT = 10000;
const unsigned long GSM_TIMEOUT = 30000;

// GSM status
bool simReady = false;
bool gprsReady = false;
bool gsmInitialized = false;

// GPS Data Variables
String nmeaSentence;
String latitude = "0", longitude = "0", utcTime = "000000", localTime = "00:00:00", fixStatus = "Void";
String speedKmh = "0.00", courseDegrees = "0.00", altitudeMeters = "0";
int satelliteCount = 0;
String satelliteStatus = "Inactive";
String espStatus = "Inactive";

// GPS status tracking
unsigned long lastValidGPS = 0;
const unsigned long GPS_TIMEOUT = 5000;

void setup() {
  Serial.begin(115200);
  GPS.begin(9600, SERIAL_8N1, 16, 17);

  pinMode(seatStatusPin, INPUT_PULLUP);
  pinMode(redLedPin, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);

  // Initialize GSM serial (but don't connect yet)
  SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);
  SerialAT.setTimeout(10000);

  // Try WiFi first, GSM will be fallback
  initializeWiFiWithFallback();
}

void loop() {
  // 🛰️ Read and parse GPS data
  bool gpsDataReceived = readAndParseGPS();
  
  // 🪑 Read Bus Seat Status and Control LED
  updateSeatStatusAndLED();
  
  // Check if GPS data is stale
  if (millis() - lastValidGPS > GPS_TIMEOUT) {
    fixStatus = "Void";
    satelliteStatus = "Inactive";
    satelliteCount = 0;
    Serial.println("⚠️ GPS data timeout - No valid GPS signal");
  }

  // 🔄 Check connection and reconnect if necessary
  if (!isConnected()) {
    Serial.println("❌ Connection lost! Attempting reconnection...");
    reconnectConnection();
  }

  // Upload every second if connected
  static unsigned long lastUpload = 0;
  if (millis() - lastUpload > 1000 && isConnected()) {
    uploadToFirebase();
    lastUpload = millis();
  }

  // Status LED blink when connected
  static unsigned long lastBlink = 0;
  if (isConnected() && millis() - lastBlink >= 1000) {
    lastBlink = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

void updateSeatStatusAndLED() {
  bool currentSeatStatus = digitalRead(seatStatusPin) == HIGH;
  
  static bool lastSeatStatus = busSeatsAvailable;
  if (lastSeatStatus != currentSeatStatus) {
    busSeatsAvailable = currentSeatStatus;
    Serial.println("🪑 Seat Status Changed: " + String(busSeatsAvailable ? "🟢 Available" : "🔴 Not Available"));
    lastSeatStatus = busSeatsAvailable;
    beep(1);
  }
  
  if (busSeatsAvailable) {
    digitalWrite(redLedPin, LOW);
  } else {
    digitalWrite(redLedPin, HIGH);
  }
}

bool isConnected() {
  if (currentConnection == "WiFi") {
    return WiFi.status() == WL_CONNECTED;
  } else if (currentConnection == "GSM") {
    return simReady && gprsReady;
  }
  return false;
}

void initializeWiFiWithFallback() {
  Serial.println("📡 Starting connection with WiFi primary, GSM fallback...");
  
  // Try WiFi first
  if (connectToWiFi(primarySSID, primaryPassword, "Primary WiFi")) {
    currentConnection = "WiFi";
    currentSSID = primarySSID;
    return;
  }
  
  if (connectToWiFi(fallbackSSID, fallbackPassword, "Fallback WiFi")) {
    currentConnection = "WiFi";
    currentSSID = fallbackSSID;
    return;
  }
  
  // If WiFi fails, try GSM
  Serial.println("❌ Both WiFi networks failed. Trying GSM...");
  if (initializeGSM()) {
    currentConnection = "GSM";
    currentSSID = "GSM Network";
  } else {
    currentConnection = "Not Connected";
    currentSSID = "Not Connected";
    Serial.println("❌ All connection methods failed.");
  }
}

bool connectToWiFi(const char* ssid, const char* password, const char* networkType) {
  Serial.print("🔌 Connecting to " + String(networkType) + ": " + String(ssid));
  
  WiFi.disconnect(true);
  delay(1000);
  WiFi.begin(ssid, password);
  
  unsigned long startAttemptTime = millis();
  
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Connected to " + String(networkType) + "!");
    Serial.println("📶 IP Address: " + WiFi.localIP().toString());
    connectionRetryCount = 0;
    return true;
  } else {
    Serial.println("\n❌ Failed to connect to " + String(networkType));
    WiFi.disconnect(true);
    return false;
  }
}

bool initializeGSM() {
  Serial.println("📡 Initializing GSM connection...");
  
  simReady = false;
  gprsReady = false;
  resetModem();

  // SIM Check
  for (int i = 0; i < 3; i++) {
    sendAT("AT+CPIN?");
    if (checkResponse("READY")) {
      Serial.println("✅ SIM Ready");
      beep(1);
      simReady = true;
      break;
    }
    Serial.println("🔁 SIM retry...");
    resetModem();
  }
  if (!simReady) return false;

  // GPRS Check
  for (int i = 0; i < 3; i++) {
    sendAT("AT+CGATT?");
    if (checkResponse("+CGATT: 1")) {
      Serial.println("✅ GPRS Attached");
      beep(2);
      gprsReady = true;
      break;
    }
    Serial.println("🔁 GPRS retry...");
    resetModem();
  }
  if (!gprsReady) return false;

  // GPRS Setup
  sendAT("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
  sendAT("AT+SAPBR=3,1,\"APN\",\"internet\"");
  sendAT("AT+SAPBR=1,1");
  
  gsmInitialized = true;
  return true;
}

void reconnectConnection() {
  connectionRetryCount++;
  Serial.println("🔄 Reconnection attempt #" + String(connectionRetryCount));
  
  if (connectionRetryCount > MAX_RETRIES) {
    Serial.println("🔄 Exceeded retry limit, switching connection method...");
    switchConnectionMethod();
    connectionRetryCount = 0;
  } else {
    // Try reconnecting with current method
    if (currentConnection == "WiFi") {
      if (currentSSID == primarySSID) {
        if (connectToWiFi(primarySSID, primaryPassword, "Primary WiFi (reconnect)")) return;
      } else {
        if (connectToWiFi(fallbackSSID, fallbackPassword, "Fallback WiFi (reconnect)")) return;
      }
      switchWiFiNetwork();
    } else if (currentConnection == "GSM") {
      if (initializeGSM()) return;
    }
    
    // If reconnection failed, switch method
    switchConnectionMethod();
  }
}

void switchConnectionMethod() {
  Serial.println("🔄 Switching connection method...");
  
  if (currentConnection == "WiFi") {
    // Switch from WiFi to GSM
    Serial.println("🔄 Switching from WiFi to GSM");
    WiFi.disconnect(true);
    if (initializeGSM()) {
      currentConnection = "GSM";
      currentSSID = "GSM Network";
    } else {
      currentConnection = "Not Connected";
    }
  } else {
    // Switch from GSM to WiFi
    Serial.println("🔄 Switching from GSM to WiFi");
    if (connectToWiFi(primarySSID, primaryPassword, "Primary WiFi (switch)")) {
      currentConnection = "WiFi";
      currentSSID = primarySSID;
    } else if (connectToWiFi(fallbackSSID, fallbackPassword, "Fallback WiFi (switch)")) {
      currentConnection = "WiFi";
      currentSSID = fallbackSSID;
    } else {
      currentConnection = "Not Connected";
    }
  }
}

void switchWiFiNetwork() {
  if (currentSSID == primarySSID) {
    if (connectToWiFi(fallbackSSID, fallbackPassword, "Fallback WiFi (switch)")) {
      currentSSID = fallbackSSID;
    }
  } else {
    if (connectToWiFi(primarySSID, primaryPassword, "Primary WiFi (switch)")) {
      currentSSID = primarySSID;
    }
  }
}

// GSM Helper Functions
void resetModem() {
  digitalWrite(MODEM_RST, LOW);
  delay(200);
  digitalWrite(MODEM_RST, HIGH);
  delay(3000);
}

void sendAT(String cmd) {
  SerialAT.println(cmd);
  delay(2000);
  printResponse();
}

void printResponse() {
  while (SerialAT.available()) {
    String r = SerialAT.readStringUntil('\n');
    r.trim();
    if (r.length() > 0) {
      Serial.println("📥 " + r);
    }
  }
}

bool checkResponse(String keyword) {
  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (SerialAT.available()) {
      String r = SerialAT.readStringUntil('\n');
      r.trim();
      if (r.indexOf(keyword) >= 0) return true;
    }
  }
  return false;
}

void beep(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}

bool readAndParseGPS() {
  bool dataReceived = false;
  
  while (GPS.available()) {
    nmeaSentence = GPS.readStringUntil('\n');
    dataReceived = true;

    if (nmeaSentence.startsWith("$GPRMC")) {
      parseGPRMC(nmeaSentence);
      localTime = convertUTCtoBST(utcTime);
      satelliteStatus = (fixStatus == "Active") ? "Active" : "Inactive";
      lastValidGPS = millis();
    } else if (nmeaSentence.startsWith("$GPGSV")) {
      parseGPGSV(nmeaSentence);
    } else if (nmeaSentence.startsWith("$GPGGA")) {
      parseGPGGA(nmeaSentence);
    }
  }
  
  return dataReceived;
}

void uploadToFirebase() {
  String formattedLat = (fixStatus == "Active") ? formatCoord(latitude, 'L') : "0°0.00'N";
  String formattedLon = (fixStatus == "Active") ? formatCoord(longitude, 'G') : "0°0.00'E";

  String payload = "{";
  payload += "\"lat\":\"" + formattedLat + "\",";
  payload += "\"lon\":\"" + formattedLon + "\",";
  payload += "\"time\":\"" + localTime + "\",";
  payload += "\"fix\":\"" + fixStatus + "\",";
  payload += "\"satellites\":\"" + String(satelliteCount) + "\",";
  payload += "\"satellite_status\":\"" + satelliteStatus + "\",";
  payload += "\"esp_status\":\"" + espStatus + "\",";
  payload += "\"speed_kmh\":\"" + speedKmh + "\",";
  payload += "\"course_deg\":\"" + courseDegrees + "\",";
  payload += "\"altitude_m\":\"" + altitudeMeters + "\",";
  payload += "\"bus_seat_status\":\"" + String(busSeatsAvailable ? "Available" : "Not Available") + "\",";
  payload += "\"connection_type\":\"" + currentConnection + "\",";
  payload += "\"network\":\"" + currentSSID + "\",";
  payload += "\"vehicle\":\"UIU-Tracker-01\"";
  payload += "}";

  if (currentConnection == "WiFi") {
    uploadViaWiFi(payload);
  } else if (currentConnection == "GSM") {
    uploadViaGSM(payload);
  }
}

void uploadViaWiFi(String payload) {
  HTTPClient http;
  String url = String(firebaseHost) + "/gps.json";
  
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  int httpResponseCode = http.PUT(payload);
  espStatus = (httpResponseCode == 200) ? "Active" : "Inactive";
  
  printStatusUpdate(httpResponseCode, String(WiFi.RSSI()));
  http.end();
}

void uploadViaGSM(String payload) {
  Serial.println("📤 Uploading via GSM...");

  // HTTP Setup for GSM
  sendAT("AT+HTTPINIT");
  sendAT("AT+HTTPPARA=\"CID\",1");
  
  String url = String(firebaseHost) + "/gps.json";
  String urlCmd = "AT+HTTPPARA=\"URL\",\"" + url + "\"";
  sendAT(urlCmd);
  
  sendAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"");

  // Send data
  SerialAT.println("AT+HTTPDATA=" + String(payload.length()) + ",10000");
  delay(1000);
  SerialAT.print(payload);
  delay(10000);

  Serial.println("🚀 Executing POST...");
  SerialAT.println("AT+HTTPACTION=1");
  delay(8000);
  
  if (checkResponse(",200,")) {
    espStatus = "Active";
    Serial.println("✅ GSM Upload Successful!");
    beep(1);
  } else {
    espStatus = "Inactive";
    Serial.println("❌ GSM Upload Failed");
  }

  // Cleanup
  sendAT("AT+HTTPREAD");
  sendAT("AT+HTTPTERM");
  
  printStatusUpdate(espStatus == "Active" ? 200 : 400, "GSM");
}

void printStatusUpdate(int responseCode, String signalInfo) {
  String formattedLat = (fixStatus == "Active") ? formatCoord(latitude, 'L') : "0°0.00'N";
  String formattedLon = (fixStatus == "Active") ? formatCoord(longitude, 'G') : "0°0.00'E";
  
  Serial.println("⏰ Time (BST): " + localTime);
  Serial.println(String("📶 ESP Status: ") + (espStatus == "Active" ? "✅ Active" : "❌ Inactive"));
  Serial.println(String("📡 Satellite Fix: ") + (fixStatus == "Active" ? "✅ Active" : "❌ Void"));
  Serial.println("🛰️ Satellite Count: " + String(satelliteCount));
  Serial.println("🧭 Latitude: " + formattedLat);
  Serial.println("🧭 Longitude: " + formattedLon);
  Serial.println("🗻 Altitude (m): " + altitudeMeters);
  Serial.println("🚀 Speed: " + speedKmh + " km/h");
  Serial.println("🧭 Course (°): " + courseDegrees);
  Serial.println("🪑 Bus Seat Status: " + String(busSeatsAvailable ? "🟢 Available" : "🔴 Not Available"));
  Serial.println("📶 Connection Type: " + currentConnection);
  Serial.println("🌐 Network: " + currentSSID);
  if (currentConnection == "WiFi") {
    Serial.println("📡 WiFi Signal: " + signalInfo + " dBm");
  }
  Serial.println("📤 Upload status: " + String(responseCode));
  Serial.println("──────────────────────────────");
}

// GPS parsing functions
void parseGPRMC(String sentence) {
  int comma1 = sentence.indexOf(',');
  int comma2 = sentence.indexOf(',', comma1 + 1);
  int comma3 = sentence.indexOf(',', comma2 + 1);
  int comma4 = sentence.indexOf(',', comma3 + 1);
  int comma5 = sentence.indexOf(',', comma4 + 1);
  int comma6 = sentence.indexOf(',', comma5 + 1);
  int comma7 = sentence.indexOf(',', comma6 + 1);
  int comma8 = sentence.indexOf(',', comma7 + 1);

  utcTime = sentence.substring(comma1 + 1, comma2);
  fixStatus = sentence.substring(comma2 + 1, comma3) == "A" ? "Active" : "Void";
  latitude = sentence.substring(comma3 + 1, comma4);
  longitude = sentence.substring(comma5 + 1, comma6);

  float knots = sentence.substring(comma6 + 1, comma7).toFloat();
  speedKmh = String(knots * 1.852, 2);

  courseDegrees = sentence.substring(comma7 + 1, comma8);
}

void parseGPGSV(String sentence) {
  int comma1 = sentence.indexOf(',');
  int comma2 = sentence.indexOf(',', comma1 + 1);
  int comma3 = sentence.indexOf(',', comma2 + 1);

  satelliteCount = sentence.substring(comma3 + 1, sentence.indexOf('*')).toInt();
}

void parseGPGGA(String sentence) {
  int commaCount = 0;
  int lastComma = -1;
  for (int i = 0; i < sentence.length(); i++) {
    if (sentence.charAt(i) == ',') {
      commaCount++;
      if (commaCount == 9) {
        lastComma = i;
        break;
      }
    }
  }
  int nextComma = sentence.indexOf(',', lastComma + 1);
  altitudeMeters = sentence.substring(lastComma + 1, nextComma);
}

String convertUTCtoBST(String utcTime) {
  if (utcTime.length() < 6) return "00:00:00";

  int hour = utcTime.substring(0, 2).toInt();
  int minute = utcTime.substring(2, 4).toInt();
  int second = utcTime.substring(4, 6).toInt();

  hour += 6;
  if (hour >= 24) hour -= 24;

  char buffer[9];
  sprintf(buffer, "%02d:%02d:%02d", hour, minute, second);
  return String(buffer);
}

String formatCoord(String rawCoord, char axis) {
  float raw = rawCoord.toFloat();
  int degrees = int(raw / 100);
  float minutes = raw - (degrees * 100);
  String hemi = (axis == 'L') ? "N" : "E";

  char buffer[20];
  sprintf(buffer, "%d°%.2f'%s", degrees, minutes, hemi.c_str());
  return String(buffer);
}
