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
int t = 1;
const float threshold = 80;
int state = 0;
// const int LED_PIN_1 = 9;
// const int LED_PIN_2 = 8; //GREEN
// const int LED_PIN_3 = 7;
// const int LED_PIN_4 = 6;
int val = 0;
// Define button pins
// const int BUTTON_PIN_1 = 5; //start
// const int BUTTON_PIN_2 = 4; //stop
// const int BUTTON_PIN_3 = 3; //reset
int motorPin = 13;
const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
const int rolePerMinute = 17;         // Adjustable range of 28BYJ-48 stepper is 0~17 rpm
const int rs = A3, en = A5, d4 = A9, d5 = A10, d6 = A11, d7 = A12;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 24, 30, 26, 32);

//LIGHTS
volatile unsigned char* port_h= (unsigned char*) 0x102; 
volatile unsigned char* ddr_h = (unsigned char*) 0x101;
volatile unsigned char* pin_h = (unsigned char*) 0x100;

volatile unsigned char* port_c = (unsigned char*) 0x28; 
volatile unsigned char* ddr_c = (unsigned char*) 0x27;
volatile unsigned char* pin_c = (unsigned char*) 0x26;

///BUTTONS
volatile unsigned char* port_e= (unsigned char*) 0x2e; 
volatile unsigned char* ddr_e = (unsigned char*) 0x2d;
volatile unsigned char* pin_e = (unsigned char*) 0x2c;


volatile unsigned char* port_g= (unsigned char*) 0x14; 
volatile unsigned char* ddr_g = (unsigned char*) 0x13;
volatile unsigned char* pin_g = (unsigned char*) 0x12;

//motor
volatile unsigned char* port_b= (unsigned char*) 0x25; 
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23;

//LCD
volatile unsigned char* port_a = (unsigned char*) 0x22; 
volatile unsigned char* ddr_a = (unsigned char*) 0x21;
volatile unsigned char* pin_a = (unsigned char*) 0x20;

volatile unsigned char* port_f = (unsigned char*) 0x31; 
volatile unsigned char* ddr_f = (unsigned char*) 0x30;
volatile unsigned char* pin_f = (unsigned char*) 0x2f;

volatile unsigned char* port_k = (unsigned char*) 0x108; 
volatile unsigned char* ddr_k = (unsigned char*) 0x107;
volatile unsigned char* pin_k = (unsigned char*) 0x106;

//ABC 123
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

void setup() {

adc_init();

*ddr_h |= (0x01<<3);
*ddr_h |= (0x01<<4);
*ddr_h |= (0x01<<5);
*ddr_h |= (0x01<<6);
*ddr_g |= (0x01<<5);
*ddr_g &= ~(0x01<<5);
*ddr_e |= (0x01<<3);
*ddr_e &= ~(0x01<<3);
*ddr_e |= (0x01<<5);
*ddr_e &= ~(0x01<<5);
*ddr_b |= (0x01<<7);
*ddr_c |= (0x01<<7);
*ddr_c |= (0x01<<5);
*ddr_a |= (0x01<<4);
*ddr_a |= (0x01<<2);


*port_k &= ~(0x01<<6);
*port_k |= (0x01<<5);
*port_f &= ~(0x01<<0);
*port_f &= ~(0x01<<4);
*port_f &= ~(0x01<<2);
*port_f |= (0x01<<1);


  // Set LED pins as outputs
  //pinMode(LED_PIN_1, OUTPUT);
  // pinMode(LED_PIN_2, OUTPUT);
  // pinMode(LED_PIN_3, OUTPUT);
  // pinMode(LED_PIN_4, OUTPUT);

  // // Set button pins as inputs
  // pinMode(BUTTON_PIN_1, INPUT_PULLUP);
  // pinMode(BUTTON_PIN_2, INPUT_PULLUP);
  // pinMode(BUTTON_PIN_3, INPUT_PULLUP);

  //   pinMode(motorPin, OUTPUT);
  Serial.begin(9600);
  while (! Serial);
  Serial.println("Speed 0 to 255");

    myStepper.setSpeed(rolePerMinute);
      dht.begin(); // initialize the sensor
  // initialize the serial port:
  	pinMode(sensorPower, OUTPUT);
	
	// Set to LOW so no power flows through the sensor
	// digitalWrite(sensorPower, LOW);
  Serial.begin(9600);

  // pinMode(A14,OUTPUT);
  // pinMode(A13,OUTPUT);
  // pinMode(A4,OUTPUT);
  // pinMode(A0,OUTPUT);
  // pinMode(A2,OUTPUT);
  // pinMode(A1,OUTPUT);
  // digitalWrite(A14,LOW); 
  // digitalWrite(A13,HIGH); 
  // digitalWrite(A4,LOW); 
  // digitalWrite(A0,LOW);
  // digitalWrite(A2,LOW);
  // digitalWrite(A1,HIGH);
  lcd.begin(16, 2);
 // Print a message to the LCD.
  //lcd.print("joe mama");

}


void loop() {
  *port_e &= ~(0x01<<3);

  *port_h &= ~(0x01<<6);

  *port_h &= ~(0x01<<5);

  *port_h &= ~(0x01<<4);

  *port_h &= ~(0x01<<3);


  // Read button states
  if(state==0)
  {
    *port_e |= (0x01<<3);
    if(*pin_g & (0x01 <<5))
    {
      state=1;
      delay(500);
    }
  }
  else if(state==1)
  {
    *port_h |= (0x01<<5);
  }
  else if(state==2)
  {
    *port_h |= (0x01<<4);
  }
  else if(state==4)
  {
    *port_h |= (0x01<<5);
    lcd.clear();
    lcd.print("Water lvl Low!");
    if(*pin_g & (0x01 <<5))
    {
      state=0;
      delay(500);
    }
    if(*pin_e & (0x01 <<5))
    {
      state=1;
      delay(500);
    }    
  }
  if(state==1 || state==2)
  {
    //value = adc_read(11);
    lcd.clear();
	  int level = readSensor();
	
	  lcd.print("Water level: ");
	  lcd.println(level);

  float humi  = dht.readHumidity();
  // read temperature as Celsius
  float tempC = dht.readTemperature();
  // read temperature as Fahrenheit
  float tempF = dht.readTemperature(true);

  // check if any reads failed
  if (isnan(humi) || isnan(tempC) || isnan(tempF)) {
    lcd.println("Failed to read from DHT sensor!");
  } else {
    lcd.print("Humidity: ");
    lcd.print(humi);
    lcd.print("%");

    lcd.print("  |  "); 
    lcd.setCursor(0, 1);
    lcd.print("Temperature: ");
    lcd.print(tempC);
    lcd.print("°C ~ ");
    lcd.print(tempF);
    lcd.println("°F");
  }
      delay(250);

    if(humi>15.00)
    {
      state=2;
    }
    else
    {
      state=1;
    }
    if(*pin_g & (0x01 <<5))
    {
      state=0;
      delay(500);
    }

    // if(value>120)
    // {
    //   state=4;
    //   delay(500);
    // }

    if(*pin_e & (0x01 <<4))
    {
      myStepper.step(stepsPerRevolution*t);
      t*=-1;
      delay(500);
    }

  }






    // Serial.println("Opening..");
    // myStepper.step(stepsPerRevolution);
    // delay(500);
    // Serial.println("Open.");
    // Serial.println("Closing..");
    // myStepper.step(-stepsPerRevolution);
    // delay(500);
    // Serial.println("Closed.");
  
  //Turn on corresponding LED for each button that is pressed
  // if (button1State == LOW) {
  //    digitalWrite(LED_PIN_1, HIGH);
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

  //   if (Serial.available())
  // {
  //   int speed = Serial.parseInt();
  //   if (speed >= 0 && speed <= 255)
  //   {
  //     analogWrite(motorPin, speed);
  //   }
  // }

// 	int level = readSensor();
	
// 	Serial.print("Water level: ");
// 	Serial.println(level);

// float humi  = dht.readHumidity();
//   // read temperature as Celsius
//   float tempC = dht.readTemperature();
//   // read temperature as Fahrenheit
//   float tempF = dht.readTemperature(true);

//   // check if any reads failed
//   if (isnan(humi) || isnan(tempC) || isnan(tempF)) {
//     Serial.println("Failed to read from DHT sensor!");
//   } else {
//     Serial.print("Humidity: ");
//     Serial.print(humi);
//     Serial.print("%");

//     Serial.print("  |  "); 

//     Serial.print("Temperature: ");
//     Serial.print(tempC);
//     Serial.print("°C ~ ");
//     Serial.print(tempF);
//     Serial.println("°F");
//   }

  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):

  // print the number of seconds since reset:
  // lcd.print(millis() / 1000);

}
/////////////////////////




int readSensor() {
	digitalWrite(sensorPower, HIGH);	// Turn the sensor ON
	delay(10);							// wait 10 milliseconds
	val = analogRead(sensorPin);		// Read the analog value form sensor
	digitalWrite(sensorPower, LOW);		// Turn the sensor OFF
	return val;							// send current reading
}


void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

// Stepper Motor:
// 24, 26, 28, 32
// DC Motor:
// 13
// Button:
// 3,4,5
// LEDS:
// 6yellow,7blue,8green,9red
// TEMP:
// 10
// Water Sensor:
// 11




// if (temp < threshold){
//   display temp and hum
//   fan off
//   green led on
// }

// else if (temp > threshold){
//  display temp and hum
//  fan motor on
//  blue led on
// }


// else if (water < thres){
//   red light on
//   error message "water level is too low"
// }

// else if (water <= thres){
//   red light on
//   error message "water level is too low"
// }

// else if (reset == true){
//   display temp and hum
//   fan off
//   green led on
// }

// else if (stop == true){
//  Yellow light on
// }
// else if (start == true){
//   display temp and hum
//   fan off
//   green led on
// }
// else{

// }
