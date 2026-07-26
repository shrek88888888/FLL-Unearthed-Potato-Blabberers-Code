#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>

WebServer server;

const char* ssid = "ssid";
const char* password = "password";

unsigned long prev = 0;
int16_t x, y, z;
float acc;

String msg = "Loading...";

void alert() {
  HTTPClient http;
  http.begin("token");
  http.GET();
  http.end();
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nConnected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  server.on("/", [&]() {
    String page = "<html><body>"
                  "<h1>Live Data</h1>"
                  "<div id='data'></div>"
                  "<script>"
                  "function update() {"
                  "  fetch('/data').then(r=>r.text()).then(t=>{document.getElementById('data').innerHTML=t});"
                  "  setTimeout(update, 100);"
                  "}"
                  "update();"
                  "</script>"
                  "</body></html>";
    server.send(200, "text/html", page);
  });
  server.on("/data", [&]() {
    server.send(200, "text/plain", msg);
  });

  server.begin();
  Wire.begin(8, 9);
  pinMode(48, OUTPUT);
  pinMode(5, INPUT);
  Wire.beginTransmission(0x53);
  Wire.write(0x2D);
  Wire.write(0x08);
  Wire.endTransmission();
}

void loop() {
  Wire.beginTransmission(0x53);
  Wire.write(0x32);
  Wire.endTransmission(false);
  Wire.requestFrom(0x53, 6, true);
  x = Wire.read() | (Wire.read() << 8);
  y = Wire.read() | (Wire.read() << 8);
  z = Wire.read() | (Wire.read() << 8);
  acc = sqrt(fabs((float)x * x + (float)y * y));
//  Serial.println(acc/256);
  int mstr = 103 - (analogRead(5) - 500) * 0.05;
//  Serial.println(mstr);
  msg = "Vibration: " + String(acc/256) + "      Moisture: " + String(mstr);
  if ((mstr > 70 || acc/256 > 0.11) && millis() - prev > 5000) {
    alert();
    prev = millis();
    digitalWrite(48, HIGH);
  } else {
    digitalWrite(48, LOW);
  }
  server.handleClient();
  delay(100);
}
