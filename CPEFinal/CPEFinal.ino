#include <DHT.h>
#include <DHT_U.h>

#include <LiquidCrystal.h>
#include <Adafruit_LiquidCrystal.h>

#include <Stepper.h>
#define DHTPIN 10
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// initializing all of the threshold and monitor variables
int t = 1;
const float waterthreshold = 100;
const float tempthreshold = 25;
int state = 1;
int level = 0;
int motorPin = 13;
const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
const int rolePerMinute = 17;         // Adjustable range of 28BYJ-48 stepper is 0~17 rpm
const int rs = A3, en = A5, d4 = A9, d5 = A10, d6 = A11, d7 = A12;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);      // initialize the lcd

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 24, 30, 26, 32);     //initialize the stepper


// Register declarations
     //LIGHTS
volatile unsigned char* port_h= (unsigned char*) 0x102; 
volatile unsigned char* ddr_h = (unsigned char*) 0x101;
volatile unsigned char* pin_h = (unsigned char*) 0x100;

volatile unsigned char* port_c = (unsigned char*) 0x28; 
volatile unsigned char* ddr_c = (unsigned char*) 0x27;
volatile unsigned char* pin_c = (unsigned char*) 0x26;

     //BUTTONS
volatile unsigned char* port_e= (unsigned char*) 0x2e; 
volatile unsigned char* ddr_e = (unsigned char*) 0x2d;
volatile unsigned char* pin_e = (unsigned char*) 0x2c;


volatile unsigned char* port_g= (unsigned char*) 0x34; 
volatile unsigned char* ddr_g = (unsigned char*) 0x33;
volatile unsigned char* pin_g = (unsigned char*) 0x32;

     //VENT
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
*ddr_b &= ~(0x01<<6);
*port_b |= (0x01<<6);

// Register implementations
*ddr_h |= (0x01<<3);
*ddr_h |= (0x01<<4);
*ddr_h |= (0x01<<5);
*ddr_h |= (0x01<<6);
*ddr_f |= (0x01<<0);
*ddr_f |= (0x01<<1);
*ddr_f |= (0x01<<2);
*ddr_f |= (0x01<<4);
*ddr_k |= (0x01<<5);
*ddr_k |= (0x01<<6);
*ddr_k &= ~(0x01<<7);
*port_k |= (0x01<<7);
*ddr_e |= (0x01<<3);
*ddr_e &= ~(0x01<<3);
*ddr_g |= (0x01<<5);
*ddr_g &= ~(0x01<<5);
*ddr_e |= (0x01<<5);
*ddr_e &= ~(0x01<<5);
*ddr_e |= (0x01<<4);
*ddr_b |= (0x01<<7);
*ddr_b |= (0x01<<5);
*ddr_b &= ~(0x01<<5);
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

  Serial.begin(9600);

  myStepper.setSpeed(rolePerMinute);
  dht.begin(); // initialize the sensor
  lcd.begin(16, 2);
}


void loop() {

// declares variables based on sensors
float temp = dht.readTemperature();
float humi = dht.readHumidity();
level = adc_read(11);

//measures variables to cause a state switch
if(*pin_e & (0x01<<3)){
 state=2;
}
if(*pin_e & (0x01<<5)){
state=1;
}

*port_e |= (0x01<<4); // sensor

// disabled state
if(state == 1){
  *port_b &= ~(0x01<<7);
 *port_h |= (0x01<<3);
   *port_h &= ~(0x01<<6);
  *port_h &= ~(0x01<<5);
  *port_h &= ~(0x01<<4);
}

// idle state
if(state == 2){
  *port_b &= ~(0x01<<7);
   *port_h |= (0x01<<5);
  float temp = dht.readTemperature();
 float humi = dht.readHumidity();
  lcd.println(temp);
  lcd.println(humi);
  *port_h &= ~(0x01<<6);
  *port_h &= ~(0x01<<4);
  *port_h &= ~(0x01<<3);
  if(level < waterthreshold){ // state switch
 state = 4;
}
if(temp > tempthreshold){ // state switch
 state = 3;
}
}

// running state
if(state == 3){
  *port_b |= (0x01<<7);
  *port_h |= (0x01<<4);
  float temp = dht.readTemperature();
 float humi = dht.readHumidity();
 lcd.print("Tempurature:");
  lcd.print(temp);
  lcd.print("Humidity:");
  lcd.print(humi);
    *port_h &= ~(0x01<<6);
  *port_h &= ~(0x01<<5);
  *port_h &= ~(0x01<<3);
if(temp <= tempthreshold){
 state = 2;  
}

if(level <= waterthreshold){
  state = 4;
}
}

// low water state
if (state == 4){
  *port_b &= ~(0x01<<7);
  lcd.println("Water level is too low");
  *port_h |= (0x01<<6);

    *port_h &= ~(0x01<<5);
  *port_h &= ~(0x01<<4);
  *port_h &= ~(0x01<<3);

  if(*pin_g & (0x01<<5)){
state=2;
  }
}

  if(*pin_b & (0x01<<6)){
    myStepper.step(stepsPerRevolution*t);
    t *= -1;
  }

  delay(1000);
  lcd.clear();
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