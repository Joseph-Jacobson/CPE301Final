#include <LiquidCrystal.h>


#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_LiquidCrystal.h>

#include <Stepper.h>
#define DHTPIN 10
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
#define sensorPower 2
#define sensorPin 11

const float threshold = 80;

const int LED_PIN_1 = 9;
const int LED_PIN_2 = 8; //GREEN
const int LED_PIN_3 = 7;
const int LED_PIN_4 = 6;
int val = 0;
// Define button pins
const int BUTTON_PIN_1 = 5; //start
const int BUTTON_PIN_2 = 4; //stop
const int BUTTON_PIN_3 = 3; //reset
int motorPin = 13;
const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
const int rolePerMinute = 17;         // Adjustable range of 28BYJ-48 stepper is 0~17 rpm
const int rs = 23, en = 25, d4 = 27, d5 = 29, and  d6 = 31, d7 = 33;
// // //LiquidCrystal lcd(23, 25, 27, 29, 31, 33);
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 24, 30, 26, 32);



void setup() {
  // Set LED pins as outputs
  //pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  pinMode(LED_PIN_3, OUTPUT);
  pinMode(LED_PIN_4, OUTPUT);

  // Set button pins as inputs
  pinMode(BUTTON_PIN_1, INPUT_PULLUP);
  pinMode(BUTTON_PIN_2, INPUT_PULLUP);
  pinMode(BUTTON_PIN_3, INPUT_PULLUP);

    pinMode(motorPin, OUTPUT);
  Serial.begin(9600);
  while (! Serial);
  Serial.println("Speed 0 to 255");

    myStepper.setSpeed(rolePerMinute);
      dht.begin(); // initialize the sensor
  // initialize the serial port:
  	pinMode(sensorPower, OUTPUT);
	
	// Set to LOW so no power flows through the sensor
	digitalWrite(sensorPower, LOW);
  Serial.begin(9600);

  lcd.begin(16,2);
  lcd.print("dicks");
}


void loop() {

  // Read button states
  int button1State = digitalRead(BUTTON_PIN_1);
  int button2State = digitalRead(BUTTON_PIN_2);
  int button3State = digitalRead(BUTTON_PIN_3);

    Serial.println("Opening..");
    myStepper.step(stepsPerRevolution);
    delay(500);
    Serial.println("Open.");
    Serial.println("Closing..");
    myStepper.step(-stepsPerRevolution);
    delay(500);
    Serial.println("Closed.");
  // Turn on corresponding LED for each button that is pressed
  // if (button1State == LOW) {
  //   digitalWrite(LED_PIN_1, HIGH);
  // } else {
  //   digitalWrite(LED_PIN_1, LOW);
  // }

  // if (button2State == LOW) {
  //   digitalWrite(LED_PIN_2, HIGH);
  // } else {
  //   digitalWrite(LED_PIN_2, LOW);
  // }

  // if (button3State == LOW) {
  //   digitalWrite(LED_PIN_3, HIGH);
  //   digitalWrite(LED_PIN_4, HIGH);
  // } else {
  //   digitalWrite(LED_PIN_3, LOW);
  //   digitalWrite(LED_PIN_4, LOW);
  // }

    if (Serial.available())
  {
    int speed = Serial.parseInt();
    if (speed >= 0 && speed <= 255)
    {
      analogWrite(motorPin, speed);
    }
  }

	int level = readSensor();
	
	Serial.print("Water level: ");
	Serial.println(level);

float humi  = dht.readHumidity();
  // read temperature as Celsius
  float tempC = dht.readTemperature();
  // read temperature as Fahrenheit
  float tempF = dht.readTemperature(true);

  // check if any reads failed
  if (isnan(humi) || isnan(tempC) || isnan(tempF)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    Serial.print("Humidity: ");
    Serial.print(humi);
    Serial.print("%");

    Serial.print("  |  "); 

    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.print("°C ~ ");
    Serial.print(tempF);
    Serial.println("°F");
  }

  lcd.setCursor(0,1);
  lcd.print("balls");

/////////////////////////
if (tempF < threshold){
  //display temp and hum
  //fan off
  pinMode(LED_PIN_2, OUTPUT);
}
else if (BUTTON_PIN_2 == HIGH){
  pinMode(LED_PIN_4, OUTPUT);
  }
else if (BUTTON_PIN_1 == HIGH){
  //display temp and hum
  //fan off
  pinMode(LED_PIN_2, OUTPUT);
}
else{}

}

int readSensor() {
	digitalWrite(sensorPower, HIGH);	// Turn the sensor ON
	delay(10);							// wait 10 milliseconds
	val = analogRead(sensorPin);		// Read the analog value form sensor
	digitalWrite(sensorPower, LOW);		// Turn the sensor OFF
	return val;							// send current reading
}


//Stepper Motor:
// 24, 26, 28, 32
//DC Motor:
// 13
//Button:
// 3,4,5
//LEDS:
// 6yellow,7blue,8green,9red
//TEMP:
// 10
//Water Sensor:
//11



/*

if (temp < threshold){
  display temp and hum
  fan off
  green led on
}

else if (temp > threshold){
 display temp and hum
 fan motor on
 blue led on
}


else if (water < thres){
  red light on
  error message "water level is too low"
}

else if (water <= thres){
  red light on
  error message "water level is too low"
}

else if (reset == true){
  display temp and hum
  fan off
  green led on
}

else if (stop == true){
 Yellow light on
}
else if (start == true){
  display temp and hum
  fan off
  green led on
}
else{

}


*/