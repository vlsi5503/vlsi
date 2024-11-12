MQ3
const int mq3Pin = 8; int mq3Status = 0; void setup(){ Serial.begin(9600); pinMode(mq3Pin , INPUT); } void loop(){ mq3Status = digitalRead(mq3Pin); if(mq3Status == HIGH) { Serial.println("ALCOHOL DETECTED"); } else { Serial.println("ALCOHOL NOT DETECTED"); } delay(1000); }

LED
const int buttonPin = 2; const int ledPin = 13; int buttonState = 0; void setup() { pinMode(ledPin, OUTPUT); pinMode(buttonPin, INPUT_PULLUP); } void loop() { buttonState = digitalRead(buttonPin); if (buttonState == LOW) { digitalWrite(ledPin, HIGH); } else { digitalWrite(ledPin, LOW); } }

ULTRASONIC
const int trigPin = 9; const int echoPin = 10; void setup() { Serial.begin(9600); pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT); } void loop(){ digitalWrite(trigPin,LOW); delayMicroseconds(2); digitalWrite(trigPin, HIGH); delayMicroseconds(10); digitalWrite(trigPin, LOW); long duration = pulseIn(echoPin,HIGH); int distance = duration * 0.034 /2 ; Serial.print("Distance:\n"); Serial.print(distance); Serial.print("cm"); delay(1000); }

LM35
// Define the analog pin connected to LM35const int analogPin = A0;void setup() {  // Start the serial communication  Serial.begin(9600);}void loop() {  // Read the analog value from the LM35 sensor  int sensorValue = analogRead(analogPin);  // Convert the analog value to voltage (5V reference, 10-bit ADC)float voltage = sensorValue * (5.0 / 1023.0); // Convert the voltage to temperature in Celsius (10mV per °C) float temperatureC = voltage * 100.0;  // Print the temperature in Celsius to the serial monitor  Serial.print("Temperature: ");  Serial.print(temperatureC);  Serial.println(" °C");// Wait for 1 second before taking another readingdelay(1000);}

Motor
#define motorPin1 9   // Input 1 of L293D#define motorPin2 8   // Input 2 of L293D#define enablePin 10  // Enable pin of L293D (for speed control)void setup() {  pinMode(motorPin1, OUTPUT);   // Set motor control pins as output  pinMode(motorPin2, OUTPUT);  pinMode(enablePin, OUTPUT);   // Set enable pin as output // Set PWM value for motor speed (0-255) analogWrite(enablePin, 255); // Full speed}void loop() { // Motor clockwise directiondigitalWrite(motorPin1, HIGH);  // Motor turns clockwise  digitalWrite(motorPin2, LOW);   // Motor turns in one direction  delay(2000);  // Run motor for 2 seconds  // Motor stop  digitalWrite(motorPin1, LOW);   // Motor stops  digitalWrite(motorPin2, LOW);   // Motor stops delay(1000);  // Stop for 1 second  // Motor counterclockwise directiondigitalWrite(motorPin1, LOW);   // Motor turns counterclockwise digitalWrite(motorPin2, HIGH);  // Motor turns in the opposite direction  delay(2000);  // Run motor for 2 seconds  // Motor stop again  digitalWrite(motorPin1, LOW);   // Motor stopsdigitalWrite(motorPin2, LOW);   // Motor stopsdelay(1000);  // Stop for 1 second}
Arduino Pin 9  -------------> Input 1 (Pin 2) of L293DArduino Pin 8  -------------> Input 2 (Pin 7) of L293DArduino Pin 10 -------------> Enable Pin (Pin 1) of L293DArduino GND -------------> GND (Pin 4) of L293DArduino 5V -------------> VCC (Pin 16) and Motor Power Supply (Pin 8) of L293DMotor Terminal 1 -------------> Output 1 (Pin 3) of L293DMotor Terminal 2 -------------> Output 2 (Pin 6) of L293D

ZIGBEE
void setup() {  Serial.begin(9600);  // Initialize serial communication at 9600 baud}void loop() {  Serial.println("Hello from Arduino!");  // Send a message delay(1000);  // Wait for 1 second}

import serial# Open the serial port to the XBee moduleser = serial.Serial('/dev/ttyUSB0', 9600)while True:if ser.in_waiting > 0:  # Check if data is available to read   data = ser.readline().decode('utf-8').strip()  # Read data and decode it   print(f"Received: {data}")  # Print the received data

PISO SIPO
Clk,rst,s30,m,y30…signal t:std_logic_vector30..beg..pro..beg…if rst =1 then y<=0000 elsif clk’event and clk=’1’ then case m is when”0” => t(3) <= s(0);…..t(0) <= t(1);..y(0) <= t(0) ,,,,,,,t3<=s0..t2<=t3…….t0
