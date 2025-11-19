/*
  WiFi Web Server Robot Control (UNO R4 WiFi)

  Larry the Robot hosts a WiFi web server that provides remote control through any browser.
  The interface includes:
    • Up / Down / Left / Right directional control
    • Stop command
    • Adjustable motor speed slider (0–255)

  Additional Intelligent Behavior:
    • Ultrasonic distance sensing (HC-SR04)
    • Automatic obstacle avoidance:
         - Detects objects closer than 10 inches
         - Stops motors, shows LED warning, reverses, then performs a random turn
    • LED matrix displays a visual red warning indicator during avoidance

  System Workflow:
    • Browser sends HTTP GET requests (e.g., /UP, /DOWN, /STOP, /SPEED?speed=180)
    • Server parses requests and triggers movement functions
    • While moving forward, the robot continuously checks distance
    • If an obstacle is detected, forward motion is overridden for safety

  This file contains:
    - WiFi network setup
    - Web server loop and HTTP parsing
    - Motor driver control functions
    - Distance measurement and obstacle avoidance logic
    - Automatic generation of the web dashboard UI
*/

#include "WiFiS3.h"
#include "Arduino_LED_Matrix.h"

// LED Matrix object (UNO R4 WiFi technically doesn't include the matrix, but included for visual output)
ArduinoLEDMatrix matrix;


// ---------------------------------------------------------------------------
// WIFI CONFIGURATION
// ---------------------------------------------------------------------------

char ssid[] = "WIFI NAME";  // WiFi SSID
char pass[] = "PASSWORD";   // WiFi password
int keyIndex = 0;              // Only used for WEP networks

int status = WL_IDLE_STATUS;
WiFiServer server(80);         // Web server listens on port 80 (HTTP)


// ---------------------------------------------------------------------------
// LED MATRIX – Visual Obstacle Warning
// ---------------------------------------------------------------------------

// Displays a round red "warning" shape on the matrix when an obstacle is detected
void showRedWarningCircle() {
  uint8_t redCircle[8][12] = {
    { 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0 },
    { 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0 },
    { 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1 },
    { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
    { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
    { 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1 },
    { 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0 },
    { 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0 }
  };
  matrix.renderBitmap(redCircle, 12, 8);
}

// Clears the LED matrix display
void clearLED() {
  matrix.clear();
}


// ---------------------------------------------------------------------------
// MOTOR PIN DEFINITIONS
// ---------------------------------------------------------------------------

// Right motor (Motor A)
const int AIN1 = 13;
const int AIN2 = 12;
const int PWMA = 11;

// Left motor (Motor B)
const int PWMB = 10;
const int BIN2 = 9;
const int BIN1 = 8;

// Motor driver standby pin (must be HIGH for motors to operate)
const int STBY = 7;


// ---------------------------------------------------------------------------
// ULTRASONIC DISTANCE SENSOR
// ---------------------------------------------------------------------------

const int trigPin = 6;     // Sends ultrasonic pulse
const int echoPin = 5;     // Receives echo

float distanceInches = 0;  // Most recent distance measurement

// Durations for automatic reversal and turning when an obstacle is detected
const int backupTime = 500;  
const int turnTime = 500;


// ---------------------------------------------------------------------------
// SPEED CONTROL
// ---------------------------------------------------------------------------

int motorSpeed = 200;  // PWM motor speed (0–255)


// ---------------------------------------------------------------------------
// MOTOR CONTROL FUNCTIONS
// ---------------------------------------------------------------------------

// Stops both motors completely
void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  Serial.println("Command: STOP (motors off)");
}

// Drives robot forward
void moveUp() {
  Serial.print("Command: UP @ ");
  Serial.println(motorSpeed);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);  // right motor forward
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);   // left motor forward

  analogWrite(PWMA, motorSpeed);
  analogWrite(PWMB, motorSpeed);
}

// Drives robot backward
void moveDown() {
  Serial.print("Command: DOWN @ ");
  Serial.println(motorSpeed);

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);  // right motor backward
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH); // left motor backward

  analogWrite(PWMA, motorSpeed);
  analogWrite(PWMB, motorSpeed);
}

// Spins robot RIGHT (pivoting in place)
void moveRight() {
  Serial.print("Command: LEFT @ ");
  Serial.println(motorSpeed);

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);   // right motor backward
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);   // left motor forward

  analogWrite(PWMA, motorSpeed);
  analogWrite(PWMB, motorSpeed);
}

// Spins robot LEFT (pivoting in place)
void moveLeft() {
  Serial.print("Command: RIGHT @ ");
  Serial.println(motorSpeed);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);  // right motor forward
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);  // left motor backward

  analogWrite(PWMA, motorSpeed);
  analogWrite(PWMB, motorSpeed);
}


// ---------------------------------------------------------------------------
// HTTP REQUEST HANDLER
// ---------------------------------------------------------------------------

// Interprets commands from browser GET requests
void handleRequestLine(const String &requestLine) {

  // Handle speed slider: /SPEED?speed=###
  if (requestLine.indexOf("GET /SPEED") >= 0) {
    int pos = requestLine.indexOf("speed=");
    if (pos >= 0) {
      int value = requestLine.substring(pos + 6).toInt();
      motorSpeed = constrain(value, 0, 255);
      Serial.print("Speed set to: ");
      Serial.println(motorSpeed);
    }
  }

  // Movement commands
  if (requestLine.indexOf("GET /UP") >= 0) moveUp();
  if (requestLine.indexOf("GET /DOWN") >= 0) moveDown();
  if (requestLine.indexOf("GET /LEFT") >= 0) moveLeft();
  if (requestLine.indexOf("GET /RIGHT") >= 0) moveRight();
  if (requestLine.indexOf("GET /STOP") >= 0) stopMotors();
}


// ---------------------------------------------------------------------------
// DISTANCE / OBSTACLE AVOIDANCE LOGIC
// ---------------------------------------------------------------------------

// Returns true if the motor pins indicate forward motion
bool isMovingForward() {
  return digitalRead(AIN1) == LOW &&
         digitalRead(AIN2) == HIGH &&
         digitalRead(BIN1) == HIGH &&
         digitalRead(BIN2) == LOW;
}

// Reads distance from HC-SR04 ultrasonic sensor (in inches)
float getDistance() {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long echoTime = pulseIn(echoPin, HIGH);

  return echoTime / 148.0;  // Conversion constant (microseconds → inches)
}

// Runs automatically while robot is moving forward.
// Stops robot and performs reversal + turn if object detected < 10 inches.
void distanceAvoidance() {

  // Only run avoidance if robot is actively driving forward
  if (!isMovingForward()) return;

  distanceInches = getDistance();

  if (distanceInches > 0) {
    Serial.print("Distance: ");
    Serial.print(distanceInches);
    Serial.println(" in");
  }

  // Trigger avoidance when object is too close
  if (distanceInches > 0 && distanceInches < 10.0) {

    Serial.println("Obstacle detected! Initiating avoidance...");

    showRedWarningCircle();   // Turn on LED warning indicator

    stopMotors();
    delay(500);

    moveDown();               // Back up
    delay(backupTime);

    // Random left or right turn
    if (random(0, 2) == 0) {
      Serial.println("Turning LEFT...");
      moveLeft();
    } else {
      Serial.println("Turning RIGHT...");
      moveRight();
    }

    delay(turnTime);

    stopMotors();
    clearLED();               // Turn off matrix warning
  }
}


// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);

  matrix.begin();  // Initialize LED matrix (if available)

  // Motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);  // Enable motor driver

  // Ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);

  // Confirm WiFi module is detected
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFi Module Error!");
    while (true);
  }

  // Connect to WiFi
  while (status != WL_CONNECTED) {
    Serial.print("Connecting to: ");
    Serial.println(ssid);

    status = WiFi.begin(ssid, pass);
    delay(10000);
  }

  server.begin();
  printWifiStatus();
}


// ---------------------------------------------------------------------------
// MAIN LOOP – Handles web requests & obstacle checking
// ---------------------------------------------------------------------------

void loop() {

  WiFiClient client = server.available();

  if (client) {
    Serial.println("New client connected");

    String currentLine = "";
    String requestLine = "";

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);

        // Detect newline → end of HTTP header line
        if (c == '\n') {

          // Blank line = end of client request → time to respond
          if (currentLine.length() == 0) {

            handleRequestLine(requestLine);  // Execute movement/speed command

            //
            // Send the dynamically-generated HTML interface
            //
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/html");
            client.println("Connection: close");
            client.println();
            client.println("<!DOCTYPE html>");
            client.println("<html>");
            client.println("<head>");
            client.println("<meta name='viewport' content='width=device-width, initial-scale=1'>");
            client.println("<title>Larry Control Panel</title>");
            client.println("<style>");
            client.println("body {font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif;text-align:center;background-color:#f5f7fa;color:#333;margin:0;padding:2em 0;}");
            client.println("h1 {font-size:4vw;margin-bottom:1em;font-weight:600;}");
            client.println(".container {background-color:#ffffff;max-width:400px;margin:0 auto;padding:2em 2.5em;border-radius:12px;box-shadow:0 4px 12px rgba(0,0,0,0.1);}");
            client.println(".btn {font-size:1.5rem;padding:0.6em 1.8em;margin:0.5em;border:none;border-radius:8px;background-color:#3b82f6;color:white;cursor:pointer;transition:background-color 0.3s ease;user-select:none;}");
            client.println(".btn:hover {background-color:#2563eb;}");
            client.println(".btn-stop {background-color:#ef4444;}");
            client.println(".btn-stop:hover {background-color:#b91c1c;}");
            client.println(".row {margin:1.2em 0;}");
            client.println(".speed-label {font-size:1.25rem;display:block;margin-bottom:0.5em;font-weight:500;}");
            client.println(".speed-value {font-weight:700;margin-left:0.3em;color:#2563eb;}");
            client.println("input[type=range] {width:100%;-webkit-appearance:none;height:8px;border-radius:5px;background:#d1d5db;outline:none;cursor:pointer;transition:background 0.3s ease;}");
            client.println("input[type=range]::-webkit-slider-thumb {-webkit-appearance:none;appearance:none;width:22px;height:22px;border-radius:50%;background:#3b82f6;cursor:pointer;transition:background-color 0.3s ease;border:none;margin-top:-7px;}");
            client.println("input[type=range]:hover::-webkit-slider-thumb {background:#2563eb;}");
            client.println("input[type=range]::-moz-range-thumb {width:22px;height:22px;border-radius:50%;background:#3b82f6;cursor:pointer;border:none;transition:background-color 0.3s ease;}");
            client.println("input[type=range]:hover::-moz-range-thumb {background:#2563eb;}");
            client.println("</style>");
            client.println("</head>");
            client.println("<body>");
            client.println("<div class='container'>");
            client.println("<h1>Larry Control Panel</h1>");

            client.println("<div class='row'><a href='/UP'><button class='btn'>Up</button></a></div>");

            client.println("<div class='row'>");
            client.println("<a href='/LEFT'><button class='btn'>Left</button></a>");
            client.println("<a href='/RIGHT'><button class='btn'>Right</button></a>");
            client.println("</div>");

            client.println("<div class='row'><a href='/DOWN'><button class='btn'>Down</button></a></div>");

            client.println("<div class='row'><a href='/STOP'><button class='btn btn-stop'>Stop</button></a></div>");

            client.println("<div class='row'>");
            client.println("<form action='/SPEED' method='GET'>");
            client.println("<span class='speed-label'>Speed: <span id='speedValue' class='speed-value'>");
            client.print(motorSpeed);  // Show current speed
            client.println("</span></span>");

            client.print("<input type='range' id='speed' name='speed' min='0' max='255' value='");
            client.print(motorSpeed);  // Slider remembers last value
            client.println("' oninput=\"document.getElementById('speedValue').textContent=this.value\">");

            client.println("<br><br><button class='btn' type='submit'>Set Speed</button>");
            client.println("</form>");
            client.println("</div>");

            client.println("</div>");
            client.println("</body>");
            client.println("</html>");

            break;
          }

          else {
            if (currentLine.startsWith("GET ")) {
              requestLine = currentLine;  // Save full GET line for parsing
            }
            currentLine = "";
          }
        }

        // Build incoming HTTP line
        else if (c != '\r') {
          currentLine += c;
        }
      }
    }

    client.stop();
    Serial.println("Client disconnected");
  }

  // Continuously run obstacle detection while driving forward
  distanceAvoidance();
}


// ---------------------------------------------------------------------------
// PRINT WIFI STATUS TO SERIAL MONITOR
// ---------------------------------------------------------------------------

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  long rssi = WiFi.RSSI();
  Serial.print("Signal Strength (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");

  Serial.print("Open your browser at: http://");
  Serial.println(ip);
}