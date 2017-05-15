#include <Adafruit_GFX.h>
#include <gfxfont.h>
#include <Servo.h>
#include <Adafruit_PCD8544.h>
#include <SPI.h>
#include <Keypad.h>
#include <math.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>

#define RXPIN 12
#define TXPIN 13
#define GPSBAUD 9600

const byte ROWS = 4;
const byte COLS = 3;
const int  servoPin = 8;
const int  dolgotaSputnikPin = 33;
const int  dolgotaMestaPin = 30;
const int  shirotaPin = 29;
const int  angleKPin = 51;
const int  vperedButtonPin = 53;
const int  nazadButtonPin = 44;
const int  gpsButton = 50;
const int  ButtonPin5 = 52;
const int  resetButtonPin = 41;
const int  LedDolgotaSputnikPin = 26;
const int  LedDolgotaMestaPin = 27;
const int  LedShirotaPin = 24;
const int  LedRabotaPinRed = 28;
const int  LedPitaniePin = 31;
const int  ServoPitaniePin = 25;
int seconds;
int timeoffset = 2; // Пользователь должен изменить единицу на соответствующий часовой пояс
int  shirotaButton = 0;
int  dolgotaMestaButton = 0;
int  dolgotaSputnikButton = 0;
float  dolgotaSputnikNum, dolgotaMestaNum, shirotaNum, speedd;
int  angleKButton = 0;
float  angleKNum ;
char kpress;
int counter = 0;



TinyGPS gps;
SoftwareSerial uart_gps(RXPIN, TXPIN);

Adafruit_PCD8544 display = Adafruit_PCD8544(45, 42, 49, 48, 47);
Servo servo;
void getgps(TinyGPS &gps);
char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'-', '0', 'C'}
};
byte rowPins[ROWS] = {36, 39, 38, 32};
byte colPins[COLS] = {43, 34, 35};
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS);

struct MOTOR    
{
  int in1;      // INPUT1
  int in2;      // INPUT2
  int enable;   // ENABLE1
};

MOTOR MOTOR1 = { 2, 3, 9 };

void setup() {
  uart_gps.begin(GPSBAUD);
  pinMode(dolgotaSputnikPin, INPUT);
  pinMode(dolgotaMestaPin, INPUT);
  pinMode(shirotaPin, INPUT);
  pinMode(angleKPin, INPUT);
  pinMode(ServoPitaniePin, OUTPUT);
  pinMode (resetButtonPin, INPUT);
  pinMode (gpsButton, INPUT);
  digitalWrite(LedRabotaPinRed, LOW);
  digitalWrite(LedPitaniePin, HIGH);
  digitalWrite(ServoPitaniePin, HIGH);
  pinMode(46, OUTPUT);
  digitalWrite(46, HIGH);
  keypad.begin( makeKeymap(keys) );
  servo.attach(8);
  servo.write(90);
  display.begin();
  display.clearDisplay();
  display.display();
  display.setTextSize(1);
  display.setContrast(50);
  display.display();
  display.print("Gotov!");
  display.display();
  delay(1500);
  display.clearDisplay();
  display.display();

}

void loop() {
  shirotaButton = digitalRead(shirotaPin);
  dolgotaMestaButton = digitalRead(dolgotaMestaPin);
  dolgotaSputnikButton = digitalRead(dolgotaSputnikPin);
  ShirotaChange();
  DolgotaMestaChange();
  dolgotaSputnikChange();
  AngleCalc();
  vpered();
  nazad();
  resets();
  gpss();

}

void resets() {
  while (digitalRead(resetButtonPin) == HIGH) {
    digitalWrite(ServoPitaniePin, LOW);
    delay(1000);
  }
  digitalWrite(ServoPitaniePin, HIGH);
}
void vpered() {
  while (digitalRead(vperedButtonPin) == HIGH) {
    digitalWrite(LedRabotaPinRed, LOW);
    forward(500);
  }
  forward(0);
}
void nazad() {
  while (digitalRead(nazadButtonPin) == HIGH) {
    digitalWrite(LedRabotaPinRed, LOW);
    back(500);
  }
  back(0);
}
void ShirotaChange() {
  while (shirotaButton == HIGH)  {
    shirotaButton = digitalRead(shirotaPin);
    kpress = keypad.getKey();
    display.clearDisplay();
    display.print("Shirota mesta");
    display.print(shirotaNum);
    display.display();
    digitalWrite(LedShirotaPin, HIGH);

    if (kpress == 'C')
    {
      shirotaNum = 0;
      display.clearDisplay();
      display.display();

    }
    if ( kpress >= '0' && kpress <= '9')
    {
      shirotaNum = shirotaNum * 10 + (kpress - '0');

    }
    if (kpress == '-') {
      shirotaNum = shirotaNum - shirotaNum * 2;
    }
    if (shirotaButton == LOW)
    {
      display.clearDisplay();
      display.display();
      digitalWrite(LedShirotaPin, LOW);
      break;
    }
  }
}

void DolgotaMestaChange() {
  while (dolgotaMestaButton == HIGH)  {
    dolgotaMestaButton = digitalRead(dolgotaMestaPin);
    kpress = keypad.getKey();
    display.clearDisplay();
    display.print("Dolgota mesta");
    display.print(dolgotaMestaNum);
    display.display();
    digitalWrite(LedDolgotaMestaPin, HIGH);

    if (kpress == 'C')
    {
      dolgotaMestaNum = 0;
      display.clearDisplay();
      display.display();

    }
    if ( kpress >= '0' && kpress <= '9')
    {
      dolgotaMestaNum = dolgotaMestaNum * 10 + (kpress - '0');

    }
    if (kpress == '-') {
      dolgotaMestaNum = dolgotaMestaNum - dolgotaMestaNum * 2;
    }
    if (dolgotaMestaButton == LOW)
    {
      display.clearDisplay();
      display.display();
      digitalWrite(LedDolgotaMestaPin, LOW);
      break;
    }
  }
}

void dolgotaSputnikChange() {
  while (dolgotaSputnikButton == HIGH)  {
    dolgotaSputnikButton = digitalRead(dolgotaSputnikPin);
    kpress = keypad.getKey();
    display.clearDisplay();
    display.print("Dolgota sputnika");
    display.print("          ");
    display.print(dolgotaSputnikNum);
    display.display();
    digitalWrite(LedDolgotaSputnikPin, HIGH);

    if (kpress == 'C')
    {
      dolgotaSputnikNum = 0;
      display.clearDisplay();
      display.display();

    }
    if ( kpress >= '0' && kpress <= '9' )
    {
      dolgotaSputnikNum = dolgotaSputnikNum * 10 + (kpress - '0');


    }
    if (kpress == '-') {
      dolgotaSputnikNum = dolgotaSputnikNum - dolgotaSputnikNum * 2;
    }
    if (dolgotaSputnikButton == LOW)
    {
      display.clearDisplay();
      display.display();
      digitalWrite(LedDolgotaSputnikPin, LOW);
      break;
    }
  }
}



void AngleCalc() {
  angleKButton = digitalRead(angleKPin);
  if (angleKButton == HIGH)  {
    digitalWrite(LedRabotaPinRed, HIGH);
    angleKNum = 90 + (atan(sin(dolgotaMestaNum * (PI / 180) - dolgotaSputnikNum * (PI / 180)) / tan(shirotaNum * (PI / 180)))) * ( 180 / PI);
    delay(100);
    servo.write(dolgotaSputnikNum);
    delay(500);
    display.clearDisplay();
    display.display();
    display.println((angleKNum-90));
    display.display();

  }
  else  digitalWrite(LedRabotaPinRed, LOW);
}


void forward(int pwm) // первый вперёд
{
  digitalWrite(MOTOR1.in1, HIGH);
  digitalWrite(MOTOR1.in2, LOW);
  analogWrite(MOTOR1.enable, pwm);
}

void back(int pwm) // первый назад
{
  digitalWrite(MOTOR1.in1, LOW);
  digitalWrite(MOTOR1.in2, HIGH);
  analogWrite(MOTOR1.enable, pwm);
}

void gpss()
{
  while (digitalRead(gpsButton) == HIGH) {
  gpsON();
}
  }


void gpsON(){
for (int i=0;i<100;i++){
  display.clearDisplay(); 
  display.print("POISK        SPUTNIKOV");  display.display();
  display.print("..............");  display.display();
  while(uart_gps.available())
  {
    int c = uart_gps.read();
    if(gps.encode(c))
    {
      getgps(gps);
    }
  }
}
display.clearDisplay();  display.display();
}
void getgps(TinyGPS &gps)
{
  int year;
  float latitude, longitude;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.f_get_position(&latitude, &longitude);
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  hour = hour + timeoffset;
  display.clearDisplay();
  display.display();
  display.print("TIME:        ");
  display.display();

  if (hour <= 9)
  {
    display.print("0"); display.print(hour, DEC); display.display();
  }
  else
  {
    display.print(hour, DEC); display.display();
  }
  display.print(":"); display.display();
  if (minute <= 9)
  {
    display.print("0"); display.print(minute, DEC); display.display();
  }
  else
  {
    display.print(minute, DEC); display.display();
  }
  display.print(":"); display.display();
  if (second <= 9)
  {
    display.print("0"); display.print(second, DEC); display.display();
  }
  else
  {
    display.print(second, DEC); display.display();
  }

  display.print("     DATE:        "); display.display();
  char buff[10];
  sprintf(buff, "%02d-%02d-%02d", day, month, year);
  display.print(buff); display.display();
  delay(2000);
  display.clearDisplay(); display.display();
  display.print("SHIROTA:     "); display.print(latitude, DEC); display.display();
  display.print("DOLGOTA:     "); display.print(longitude, DEC); display.display();
  dolgotaMestaNum = longitude;
  shirotaNum = latitude;
  delay(2000);
}
