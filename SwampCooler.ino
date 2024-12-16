//Chanda Guy
//Final Project 
//CPE 301

#include <Wire.h>
#include <RTClib.h>
#include <DHT.h>
#include <LiquidCrystal.h>
#include <Stepper.h>

//GPIO Registers
//State LEDs - Digital Pins 30 (Y), 31 (G), 32 (R), 33 (B)
volatile unsigned char *portC = (unsigned char *) 0x28;
volatile unsigned char *myDDRC = (unsigned char *) 0x27;
//Fan Motor Control - Digital Pin 12
volatile unsigned char *portB = (unsigned char *) 0x25;
volatile unsigned char *myDDRB = (unsigned char *) 0x24;
//Water Sensor - Digital Pin 7
volatile unsigned char *portH = (unsigned char *) 0x102;
volatile unsigned char *myDDRH = (unsigned char *) 0x101;

//ADC Registers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

const int waterThreshold = 100;
const int tempThreshold = 20;

RTC_DS1307 rtc;
DateTime currentTime;
int nextMinute = 0; //value to update the temperature and humidity every minute

DHT dht(4, DHT11);

const int rs = 8, en = 9, d4 = 10, d5 = 11, d6 = 12, d7 = 13;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const int stepsPerRevolution = 512;
Stepper myStepper = Stepper(stepsPerRevolution, 7, 6, 4, 3);

int value = 0;

void setup() 
{
  Serial.begin(9600);
  *myDDRC |= 0xF0; //(PB[7:4] OUTPUT)
  *portC &= 0b10001111;
  *portC |= 0b10000000; //begin in DISABLED
  adc_init();
  *myDDRH |= (1<<4); //pinMode 7 OUTPUT
  *portH &= ~(1<<4); //digitalWrite LOW

  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  currentTime = rtc.now();
  nextMinute = currentTime.minute() + 1;
  
  dht.begin();
  myStepper.setSpeed(5);
  lcd.begin(16, 2);
  lcd.print("Caleb");
  lcd.setCursor(0,1);
  lcd.print("Mohammedali");

  Serial.print("Swamp cooler began at ");
  Serial.print(currentTime.hour(), DEC);
  Serial.print(':');
  Serial.print(currentTime.minute(), DEC);
  Serial.print(':');
  Serial.print(currentTime.second(), DEC);
  Serial.println();

  attachInterrupt(digitalPinToInterrupt(2), start, HIGH); //Start button 
  attachInterrupt(digitalPinToInterrupt(3), reset, HIGH); //Reset button
  attachInterrupt(digitalPinToInterrupt(18), turn, HIGH); //Turn button
  attachInterrupt(digitalPinToInterrupt(19), stop_button, LOW); // Stop button

  *myDDRB |= 0b01000000;
}

void loop() 
{
  currentTime = rtc.now();
  if(*portC != 0b10000000)
  {
    *portH |= (1<<4); //digitalWrite HIGH
    delay(20);
    value = adc_read(0); // Analog Pin 0
    *portH &= ~(1<<4);
    
    Serial.print("Sensor value: ");
    Serial.println(value);

    if(currentTime.minute() == nextMinute && *portC != 0b00100000)
    {
      nextMinute++;
      delay(10);
      float tempCel = dht.readTemperature();
      float humid = dht.readHumidity();

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(tempCel);
      lcd.print((char)223);
      lcd.print("C");
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(humid);
      lcd.print("%");
    }
    delay(1000);

    if(value < waterThreshold) // green or blue light on and water level too low
    {
      if(*portC == 0b01000000)
      {
        *portC &= ~(1<<6); // turn green off
      
        Serial.print("State changed from IDLE to ERROR at ");
        Serial.print(currentTime.hour(), DEC);
        Serial.print(':');
        Serial.print(currentTime.minute(), DEC);
        Serial.print(':');
        Serial.print(currentTime.second(), DEC);
        Serial.println();
      }
      else if(*portC == 0b00010000)
      {
        *portC &= ~(1<<4); // turn blue off
      
        *portB &= ~(1<<6);
        Serial.print("State changed from RUNNING to ERROR at ");
        Serial.print(currentTime.hour(), DEC);
        Serial.print(':');
        Serial.print(currentTime.minute(), DEC);
        Serial.print(':');
        Serial.print(currentTime.second(), DEC);
        Serial.println();
      }

      *portC |= (1<<5); // turn red on
      
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("ERROR: Water");
      lcd.setCursor(0, 1);
      lcd.print("Too Low.");
    }

    if(dht.readTemperature() > tempThreshold && *portC == 0b01000000)
    {
      *portC &= ~(1<<6); // turn off green
      *portC |= (1<<4); //turn on blue
      *portB |= (1<<6); 
      lcd.clear();
      Serial.print("State changed from IDLE to RUNNING at ");
      Serial.print(currentTime.hour(), DEC);
      Serial.print(':');
      Serial.print(currentTime.minute(), DEC);
      Serial.print(':');
      Serial.print(currentTime.second(), DEC);
      Serial.println();
    }
    else if(dht.readTemperature() < tempThreshold && *portC == 0b00010000)
    {
      *portC &= ~(1<<4); // turn off blue
      *portC |= (1<<6); //turn on green
      *portB |= (1<<6) ;
      lcd.clear();
      Serial.print("State changed from IDLE to RUNNING at ");
      Serial.print(currentTime.hour(), DEC);
      Serial.print(':');
      Serial.print(currentTime.minute(), DEC);
      Serial.print(':');
      Serial.print(currentTime.second(), DEC);
      Serial.println();
    }
  }
}

void start()
{
  if(*portC != 0b01000000)
  {
    *portC &= ~(1<<7); // turn yellow off
    *portC |= (1<<6); // turn green on
    Serial.print("State changed from DISABLED to IDLE at ");
    Serial.print(currentTime.hour(), DEC);
    Serial.print(':');
    Serial.print(currentTime.minute(), DEC);
    Serial.print(':');
    Serial.print(currentTime.second(), DEC);
    Serial.println();

    float t = dht.readTemperature();
    float h = dht.readHumidity();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print((int)t);
    lcd.print((char)223);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Humidity: ");
    lcd.print((int)h);
    lcd.print("%");
  }
}

void reset()
{
  if(*portC == 0b00100000)
  {
    int resetValue = adc_read(0);
    if(value >= waterThreshold)
    {
      *portC &= ~(1<<5); // turn off red
      *portC |= (1<<6); //turn on green
      lcd.clear();
      Serial.print("State changed from ERROR to IDLE at ");
      Serial.print(currentTime.hour(), DEC);
      Serial.print(':');
      Serial.print(currentTime.minute(), DEC);
      Serial.print(':');
      Serial.print(currentTime.second(), DEC);
      Serial.println();

      float t = dht.readTemperature();
      float h = dht.readHumidity();
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print((int)t);
      lcd.print((char)223);
      lcd.print("C");
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print((int)h);
      lcd.print("%");
    }
  }
}

void turn()
{
  if(*portC != 0b10000000)
  {
    Serial.println("Turn button pressed.");
    myStepper.step(stepsPerRevolution);
    Serial.println("90D clockwise");
  }
}

void stop_button()
{
  if(*portC == 0b00010000)
  {
     *portB &= ~(1<<6);
     *portC &= ~(1<<4); // turn off blue
     *portC |= (1<<7); //turn on yellow
     lcd.clear();
     Serial.print("State changed from RUNNING to DISABLED at ");
     Serial.print(currentTime.hour(), DEC);
     Serial.print(':');
     Serial.print(currentTime.minute(), DEC);
     Serial.print(':');
     Serial.print(currentTime.second(), DEC);
     Serial.println();
  }
}

void adc_init() // borrowed from Labs
{
  *my_ADCSRA |= 0b10000000;
  *my_ADCSRA &= 0b11010000;
  *my_ADCSRB &= 0b11110000;
  *my_ADMUX |= 0b01000000;
  *my_ADMUX &= 0b01000000;
}

unsigned int adc_read(unsigned char adc_channel_num) // borrowed from Labs
{
  *my_ADMUX &= 0b11100000;
  *my_ADCSRB &= 0b11011111;
  *my_ADMUX += adc_channel_num;
  *my_ADCSRA |= 0b01000000;
  while((*my_ADCSRA & 0x40) != 0);
  return *my_ADC_DATA;
}
