/*******************GulecTech Mushroom Greenhouse Automation V1.1********************
Author:  Firat Gulec   :      firat_gulec@hotmail.com
         Harika Gulec  :      Harikag@hotmail.com
 
Lisence: Attribution-NonCommercial
 
Note:    This piece of source code is supposed to be used as a demostration ONLY. More
         sophisticated calibration is required for industrial field application. 
 
                                                            Fırat Gulec    2016-02-21
************************************************************************************/

//Libraries
#include <dht.h>
#include <SPFD5408_Adafruit_GFX.h>    // Core graphics library
#include <SPFD5408_Adafruit_TFTLCD.h> // Hardware-specific library
#include <SPFD5408_TouchScreen.h>

#include <Wire.h>
#include <DS3231.h>

DS3231 clock;
RTCDateTime dt;

#include <SPI.h>
#include <SD.h>

const int chipSelect = 53;


//Constants
dht DHT;
const int dhtPin = A12; //Data pin of DHT-22
const int  prob = A13; //AnalogData  pin of TP Sensor  

//Variables
int chk;
int hum;  //Stores humidity value
int temp; //Stores temperature value
int Olcum_sonucu = 0 ; //TP Sensor
int toprak ;
char fff;


//Hardware Related Macros
#define         MG_PIN                       (11)     //define which analog input channel you are going to use
#define         BOOL_PIN                     (48)
#define         DC_GAIN                      (8.5)   //define the DC gain of amplifier
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin
#define R4 19
#define R3 18
#define R2 17
#define R1 16
 
//Software Related Macros
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interval(in milisecond) between each samples in 
#define  BLACK   0x0000
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);


//Application Related Macros
//These two values differ from sensor to sensor. user should derermine this value.
#define         ZERO_POINT_VOLTAGE           (0.220) //define the output of the sensor in volts when the concentration of CO2 is 400PPM
#define         REACTION_VOLTGAE             (0.020) //define the voltage drop of the sensor when move the sensor from air into 1000ppm CO2
 
//Globals
float           CO2Curve[3]  =  {2.602,ZERO_POINT_VOLTAGE,(REACTION_VOLTGAE/(2.602-3))};   
                                                     //two points are taken from the curve. 
                                                     //with these two points, a line is formed which is
                                                     //"approximately equivalent" to the original curve.
                                                     //data format:{ x, y, slope}; point1: (lg400, 0.324), point2: (lg4000, 0.280) 
                                                     //slope = ( reaction voltage ) / (log400 –log1000) 

void setup(void)
{
    pinMode(R4,OUTPUT);
    pinMode(R3,OUTPUT);
    pinMode(R2,OUTPUT);
    pinMode(R1,OUTPUT); 
    Serial.begin(9600);                              
    pinMode(BOOL_PIN, INPUT);                        //set pin to input
    digitalWrite(BOOL_PIN, HIGH);                    //turn on pullup resistors
//    digitalWrite(R4, HIGH);
//    digitalWrite(R3, HIGH);
//    digitalWrite(R2, HIGH);
//    digitalWrite(R1, HIGH);
    Serial.print("Mushroom CO2\n");    
    tft.reset();
    tft.begin(0x9341); // SDFP5408
    tft.setRotation(0); // Rotation Screen
    clock.begin();

  // Set sketch compiling time
  clock.setDateTime(__DATE__, __TIME__);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

             
}
 
void loop()
{





// *** Sensors Read -- Begin

dt = clock.getDateTime();

  // For leading zero look to DS3231_dateformat example

  Serial.print("Tarih -- Saat: ");

  Serial.print(dt.day);    Serial.print(".");
  Serial.print(dt.month);  Serial.print(".");
  Serial.print(dt.year);   Serial.print(" ");
  
  Serial.print(dt.hour);   Serial.print(":");
  Serial.print(dt.minute); Serial.print(":");
  Serial.print(dt.second); Serial.println("");

 
    chk = DHT.read(dhtPin); //Check data pin and read values
    hum = DHT.humidity;
    temp= DHT.temperature;
    Serial.print("Humidity: ");
    Serial.print(hum);
    Serial.print(" %, Temp: ");
    Serial.print(temp);
    Serial.println(" Celsius");

    int percentage;
    float volts;
    int sensorValue  = analogRead(prob);
    sensorValue = constrain(sensorValue, 485, 1023);
    
    Serial.print("Toprak Nem Olcum Sonucu : ");
    Olcum_sonucu = map(sensorValue, 485, 1023, 100, 0);
    Serial.print(Olcum_sonucu); Serial.println("%");
    
    volts = MGRead(MG_PIN);
    Serial.print( "CO2 Seviyesi  :" );
    Serial.print(volts); 
    Serial.print( "V           " );
    percentage = MGGetPercentage(volts,CO2Curve);
    Serial.print("CO2:");
    if (percentage == -1) {
        Serial.print( "<400" );
    } else {
        Serial.print(percentage);
    }
    Serial.print( "ppm" );  
    Serial.print("\n");
    if (digitalRead(BOOL_PIN) ){
        Serial.print( "=====BOOL is HIGH======" );
    } else {
        Serial.print( "=====BOOL is LOW======" );
    }
    Serial.print("\n");



// *** Sensors Read -- End




    

// *** SPFD5408 -- Begin
  
  
  tft.fillScreen(BLACK);
  unsigned long start = micros();
  tft.setCursor(0, 0); tft.setTextColor(YELLOW); tft.setTextSize(1);
  tft.println("CO2 SEVIYESI :");
  tft.println(); tft.setTextColor(CYAN); tft.setTextSize(6);
  tft.println(percentage);
  tft.setCursor(147, 40); tft.setTextSize(3);
  tft.print("ppm");
  tft.setCursor(0, 45);
  tft.println(); tft.setTextColor(YELLOW); tft.setTextSize(1);
  tft.println("SICAKLIK - NEM :");
  tft.println(); tft.setTextColor(WHITE); tft.setTextSize(5);
  tft.println(temp);
  tft.setCursor(55, 94); tft.setTextSize(4);
  tft.print(" C");
  tft.setCursor(130, 85); tft.setTextSize(5);
  tft.print(hum);
  tft.print("%");
  tft.println();
  tft.setCursor(0, 135); tft.setTextColor(YELLOW); tft.setTextSize(1);
  tft.println("CEKIRDEK NEM SEVIYESI :"); tft.println(); 
  tft.setCursor(0, 165);tft.setTextSize(6); tft.setTextColor(GREEN);
  tft.print(Olcum_sonucu); tft.print("%");
  tft.println(); tft.setTextSize(2);
  tft.println(); tft.setTextSize(5); tft.setTextColor(RED);
  tft.print(dt.hour); tft.print(":");
  tft.print(dt.minute); tft.print(" ");
  tft.println(); tft.setTextSize(1);
  tft.println(); tft.setTextSize(4); tft.setTextColor(RED);
  tft.print(dt.day); tft.print(".");
  tft.print(dt.month); tft.print(".");
  tft.print(dt.year); tft.println(" ");
  
// *** SPFD5408 change -- End

// *** Ports change -- Begin

if (percentage > 900) {
digitalWrite(R4, HIGH);  // CO2 Alım..
Serial.println(" CO2 Alim.. ");
tft.setCursor(160, 5); tft.setTextSize(5);
tft.setTextColor(RED);  tft.print(".");
}
if (percentage < 700) {
digitalWrite(R4, LOW);  // C02 Normal..
}

if (temp > 20)  {
digitalWrite(R3, HIGH);  // Sıcaklık Düşük ..
Serial.println(" Sicaklik Düşük .. ");
tft.setCursor(51, 80); tft.setTextSize(5);
tft.setTextColor(RED);  tft.print(".");
}
if (temp < 17)  {
digitalWrite(R3, LOW);  // Sıcaklık Yüksek ..
} 

if (hum < 85) {
digitalWrite(R2, HIGH);  // Nem Düşük ..
Serial.println(" Nem Dusuk .. ");
tft.setCursor(210, 80); tft.setTextSize(5);
tft.setTextColor(RED);  tft.print(".");
}
if (hum > 90) {
digitalWrite(R2, LOW);  // Nem Yüksek ..
} 

if (dt.hour == 14 )  {
  digitalWrite(R1, HIGH);  // LED Açık..
  Serial.println("LED Açık..");
  tft.setCursor(150, 215); tft.setTextSize(6);
  tft.setTextColor(WHITE);  tft.print(".");
}
else
{
  digitalWrite(R1, LOW);  // LED Kapalı..
  }
if (dt.hour == 18 )  {
  digitalWrite(R1, LOW);  // LED Kapalı..
  Serial.println("LED Kapalı..");
}



// *** Ports change -- End

// open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.

    
   
  
  File dataFile = SD.open( "Datalog.txt" , FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
  dataFile.print("CO2 Seviyesi:");
  dataFile.print(percentage);
  dataFile.println("ppm");
  dataFile.print("Sicaklik - Nem :");
  dataFile.print(temp);
  dataFile.print(" C ");
  dataFile.print(hum);
  dataFile.println("%");
  dataFile.print("Cekirdek Nem Seviyesi :");
  dataFile.print(Olcum_sonucu); 
  dataFile.println("%");
  dataFile.print(dt.hour);   dataFile.print(":");
  dataFile.println(dt.minute);
  
  dataFile.print(dt.day); dataFile.print(" "); 
  dataFile.print(dt.month);  dataFile.print(".");
  dataFile.print(dt.year); dataFile.println(".");
  
  dataFile.println("******************");
    dataFile.close();
    // print to the serial port too:
    
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
      tft.setCursor(0, 310); tft.setTextSize(1);
      tft.setTextColor(WHITE);  tft.print("sdcard problemi...");
  }



delay(90000);

}


 
/*****************************  MGRead *********************************************
Input:   mg_pin - analog channel
Output:  output of SEN-000007
Remarks: This function reads the output of SEN-000007
************************************************************************************/ 
float MGRead(int mg_pin)
{
    int i;
    float v=0;
 
    for (i=0;i<READ_SAMPLE_TIMES;i++) {
        v += analogRead(mg_pin);
        delay(READ_SAMPLE_INTERVAL);
    }
    v = (v/READ_SAMPLE_TIMES) *5/1024 ;
    return v;  
}
 
/*****************************  MQGetPercentage **********************************
Input:   volts   - SEN-000007 output measured in volts
         pcurve  - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(MG-811 output) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
int  MGGetPercentage(float volts, float *pcurve)
{
   if ((volts/DC_GAIN )>=ZERO_POINT_VOLTAGE) {
      return -1;
   } else { 
      return pow(10, ((volts/DC_GAIN)-pcurve[1])/pcurve[2]+pcurve[0]);
   }
}
