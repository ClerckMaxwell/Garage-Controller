#include <IRremote.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <Servo.h>

// Sensors
const int DHTPin = 7;
const int IRPin = 8;
const int servoPin = 9;
const int hallPin = 10;
const int buttonPin = 18;
const int buzzerPin = 46;
const int relayPin = 53;
const int LDRPin = A0;

const int LCDPin[6] = {12,11,5,4,3,2};
const int motorPin1 = 35;      //IN1 sul pin 8    
const int motorPin2 = 37;      //IN2 sul pin 9
const int motorPin3 = 39;     //IN3 sul pin 10
const int motorPin4 = 41;     //IN4 sul pin 11

// Constants
int LDRThreshold = 20;
// Garage timing
long int openingTime = 10;
long int closingTime;
long int lastObstacleTime = 10;
const long int idleTimeout = 10000;
const long int timeToClose = 27500;
const int motor_Speed = 3;
// Garage codes
int codeToOpen[4] = {1,2,3,4};    // code to insert to open garage
int insertedCode[4] = {0,0,0,0};  // curently inserted code
int digitToClose = 0;

// Boolean variables
bool hallSensorTriggered = 0;
bool lightDetected = 1;
bool windowState = 0; // 0: closed, 1:open
volatile bool buttonPressed = false;

// Variables
// volatile float h,t;

// Device instances
Servo myServo;
IRrecv irrecv(IRPin);
decode_results results;
LiquidCrystal lcd(LCDPin[0], LCDPin[1], LCDPin[2], LCDPin[3], LCDPin[4], LCDPin[5]);
// LCD RS pin to digital pin 12
// LCD Enable pin to digital pin 11
// LCD D4 pin to digital pin 5
// LCD D5 pin to digital pin 4
// LCD D6 pin to digital pin 3
// LCD D7 pin to digital pin 2
// LCD R/W pin to GND
// LCD VSS pin to GND
// LCD VCC pin to 5V
// LCD LED+ to 5V through a 220 ohm resistor
// LCD LED- to GND

// DHT dht(DHTPin, DHT11);

// Garage FSM
enum State {CLOSED, FIRST, SECOND, THIRD, OPEN};
String infoGarage[5] = {"Garage closed","Code: *", "Code: **", "Code: ***","Garage open"};
State currentState = CLOSED;

void setup() {

  Serial.begin(9600);
  
  // IR receiver
  // Timer2 is used in IRRemote library (cannot use twice)
  irrecv.enableIRIn();                                              // Start the IR receiver

  // Stepper initialization
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);

  // Hall sensor
  pinMode(hallPin, INPUT_PULLUP);          // Configure the Hall sensor pin as input with internal pull-up resistor
  PCMSK0 |= (1 << PCINT4);                 // Enable Pin Change Interrupt for the Hall sensor
  PCIFR |= (1 << PCIF0);                   // Clear the Pin Change Interrupt flag for PCINT7-0
  PCICR |= (1 << PCIE0);                   // Enable Pin Change Interrupt for PCINT7-0

//  // DHT sensor
//  dht.begin();

  // Servo motor
  myServo.attach(servoPin);
  myServo.write(2);

  // LCD screen
  lcd.begin(16,2);

  // Relay
  DDRB |= (1 << PB0);
  PORTB |= (1 << PB0);

  // Buzzer
  pinMode(buzzerPin,OUTPUT);

  // Button
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin),openWindow,FALLING);
//  PCICR |= (1 << PCIE1);
//  PCMSK1 |= (1 << PCINT9);

//  // Timer for DHT state monitoring
//  TCCR1A = 0; // initialize to 0
//  TCCR1B = 0; // same for TCCR1B
//  TCNT1  = 0; // counter initialized to 0
//
//  OCR1A = 62416;
//  TCCR1B |= (1 << WGM12); // CTC mode
//  TCCR1B |= (1 << CS12) | (1 << CS10); // prescaler = 1024
//  TIMSK1 |= (1 << OCIE1A); // abilita il relativo interrupt

  interrupts();
  
  printOnLCD(0,-1);
  printOnLCD(1,currentState);
  
  openGarage();
  closeGarage();
}

void loop() {
  
  if(millis() - openingTime >= idleTimeout && currentState == OPEN)
    closeGarage();
  
  if (irrecv.decode(&results))
    handleIR();

}

// Mealy Machine
void handleIR(){

    int decoded = translateIR(results.value);
    Serial.print(results.value, HEX);
    Serial.print(": ");
    Serial.println(decoded);
    if (decoded >= -1 && decoded <= 9){
      int digit = decoded;
      switch (currentState){
        case CLOSED:
          if (digit != -1){
            currentState = FIRST;
            insertedCode[0] = digit;
          }
          else
            currentState = CLOSED;
          break;
    
        case FIRST:
          if (digit != -1){
            currentState = SECOND;
            insertedCode[1] = digit;
          }
          else
            currentState = CLOSED;
          break;
    
        case SECOND:
          if (digit != -1){
            currentState = THIRD;
            insertedCode[2] = digit;
          }
          else
            currentState = FIRST;
          break;
    
        case THIRD:
          if (digit != -1){
            insertedCode[3] = digit;
            if(codeIsCorrect()){
              currentState = OPEN;
              openGarage();
            }
            else
              currentState = CLOSED;
          }
          else
            currentState = SECOND;
          break;
        
        case OPEN:
          if (digit == digitToClose){
              currentState = CLOSED;
              closeGarage();
          }
          break;
    
        default: break;
      }
    }
//    lcd.print(1,currentState);
    printOnLCD(1,currentState);
    Serial.println(infoGarage[currentState]);
    
    irrecv.resume();  // Receive the next IR codeToOpen

  results.value = 0;
}

bool codeIsCorrect(){
  for(int i = 0; i < 4; i++)
    if(insertedCode[i] != codeToOpen[i])
      return 0;

  return 1;
}

int translateIR(unsigned long value)
{
  // Takes command based on IR code received
  switch (value) {
    case 0xFF22DD:
      return -1;  // ("key: |<<");
    case 0xFF6897:
      return 0;
    case 0xFF30CF:
      return 1;
    case 0xFF18E7:
      return 2;
    case 0xFF7A85:
      return 3;
    case 0xFF10EF:
      return 4;
    case 0xFF38C7:
      return 5;
    case 0xFF5AA5:
      return 6;
    case 0xFF42BD:
      return 7;
    case 0xFF4AB5:
      return 8;
    case 0xFF52AD:
      return 9;
    default:
      Serial.println("Use IR remote to send numbers!");
      return -2;
  }
}

void openGarage(){
  currentState = OPEN;
  printOnLCD(1,currentState);
  
  while(!hallSensorTriggered)
    rotateLeft();

  // establish in setup position
  for(int i = 0; i < 20; i++)
    rotateRight();
  
  hallSensorTriggered = 0;
  openingTime = millis();  
}

void closeGarage(){
  currentState = CLOSED; 
  printOnLCD(1,currentState);

  closingTime = millis();
  int i = 0;
  while(millis() - closingTime <= timeToClose){

    if (i % 20 == 0){
      checkLDRState();
  
      if(lightDetected)
        rotateRight();
      else
        break;
    }
  }

  if(!lightDetected){
    digitalWrite(buzzerPin,HIGH);
    delay(1000);
    digitalWrite(buzzerPin,LOW);
    openGarage();
  }
}

void checkLDRState(){
  if (analogRead(LDRPin) <= LDRThreshold)
    lightDetected = 1;  // Set the flag to indicate light detection
  else{
    lightDetected = 0;  // Set the flag to indicate no light detection
    Serial.println("Obstacle detected!");
  }
}

void printOnLCD(int line, int index){
  // line = 0 is the first
  // line = 1 is the second
  
  // clear line before printing
  for(int i = 0; i < 16; i++) { // 16 indicates symbols in line
    lcd.setCursor(i,line);
    lcd.print(" ");
  }
  lcd.setCursor(0,line); // set cursor in the beginning of deleted line
  
  switch(line){
    case 0: if (windowState)
              lcd.print("Window open");
            else
              lcd.print("Window closed");
            break;

    case 1: if (index >= CLOSED && index <= OPEN)
              lcd.print(infoGarage[index]);
            else
              lcd.print("Number error!");
            break;
    default: Serial.println("LCD line setting error");
            break;
            
  }
}

void rotateLeft(){
  digitalWrite(motorPin4, HIGH);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin1, LOW);
  delay(motor_Speed);
  digitalWrite(motorPin4, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin1, LOW);
  delay(motor_Speed);
  digitalWrite(motorPin4, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin1, LOW);
  delay(motor_Speed);
  digitalWrite(motorPin4, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin1, HIGH);
  delay(motor_Speed);
}

void rotateRight(){
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
  delay(motor_Speed);
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
  delay(motor_Speed);
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
  delay(motor_Speed);
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH);
  delay(motor_Speed);
}

// Interrupt

ISR (PCINT0_vect){
  if(!digitalRead(hallPin)){
    hallSensorTriggered = 1;
  }
}

//ISR (PCINT1_vect) {
//  // Read the button pin state
//  bool buttonState = digitalRead(buttonPin);
//
//  // Check if the button is pressed (LOW)
//  if (buttonState == LOW) {
//    // invert vent and window state
//    digitalWrite(relayPin,!digitalRead(relayPin));
//    windowState = !windowState;
//
//    if(windowState)
//      myServo.write(45);
//    else
//      myServo.write(2);
//
//    printOnLCD(0,-1);
//  }
//}

void openWindow() {
  // invert vent and window state
  digitalWrite(relayPin,!digitalRead(relayPin));
  windowState = !windowState;

  if(windowState)
    myServo.write(45);
  else
    myServo.write(2);

  printOnLCD(0,-1);
}

//ISR (TIMER1_COMPA_vect){
//  // read humidity and temperature
//  h  = dht.readHumidity();
//  t = dht.readTemperature();
//
//  // check if any reads failed
//  if (isnan(h) || isnan(t)) {
//    Serial.println("Failed to read from DHT sensor!");
//  } 
//  else {
//    Serial.print("Humidity: ");
//    Serial.print(h);
//    Serial.print("%");
//    Serial.print("  |  "); 
//    Serial.print("Temperature: ");
//    Serial.print(t);
//    Serial.println("°C");
//  }
//
//  printOnLCD(0,-1);
//}

