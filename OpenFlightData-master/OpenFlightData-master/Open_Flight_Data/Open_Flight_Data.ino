////////////////////////////////////////
//          OPEN FLIGHT DATA          //
//       by Luis Briones, 2015        //
//             and others             //
//          Open Source Code          //
//   See Open Source Licence after    //
////////////////////////////////////////

//Arduino Vario Project found in:
//http://www.instructables.com/id/GoFly-paraglidinghanglidinggliding-altimeter-v/?ALLSTEPS

/*
Arduino Vario by Jaros, 2012 (dedicated to atmega328 based arduinos)
Part of the "GoFly" project
https://sites.google.com/site/jarosrwebsite/para-nav
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

Arduino board creates NMEA like protocol with variometer output and beping sound.
LK8000 EXTERNAL INSTRUMENT SERIES 1 - NMEA SENTENCE: LK8EX1
VERSION A, 110217

$LK8EX1,pressure,altitude,vario,temperature,battery,*checksum

Field 0, raw pressure in hPascal:hPA*100 (example for 1013.25 becomes 101325)
no padding (987.25 becomes 98725, NOT 098725)
If no pressure available, send 999999 (6 times 9)
If pressure is available, field 1 altitude will be ignored
Field 1, altitude in meters, relative to QNH 1013.25
If raw pressure is available, this value will be IGNORED (you can set it to 99999
but not really needed)!(if you want to use this value, set raw pressure to 999999)
This value is relative to sea level (QNE). We are assuming that currently at 0m
altitude pressure is standard 1013.25.If you cannot send raw altitude, then send
what you have but then you must NOT adjust it from Basic Setting in LK.
Altitude can be negative. If altitude not available, and Pressure not available, set Altitude
to 99999. LK will say "Baro altitude available" if one of fields 0 and 1 is available.
Field 2, vario in cm/s
If vario not available, send 9999. Value can also be negative.
Field 3, temperature in C , can be also negative.If not available, send 99
Field 4, battery voltage or charge percentage.Cannot be negative.If not available, send 999.
Voltage is sent as float value like: 0.1 1.4 2.3 11.2. To send percentage, add 1000.
Example 0% = 1000. 14% = 1014 . Do not send float values for percentages.
Percentage should be 0 to 100, with no decimals, added by 1000!
Credits:
(1) http://code.google.com/p/bmp085driver/                             //bmp085 library
(2) http://mbed.org/users/tkreyche/notebook/bmp085-pressure-sensor/    //more about bmp085 and average filter
(3) http://code.google.com/p/rogue-code/                               //helpfull tone library to make nice beeping without using delay
(4) http://www.daqq.eu/index.php?show=prj_sanity_nullifier             //how to make loud piezo speaker
(5) http://lk8000.it                                                   //everything because of that
(6) http://taturno.com/2011/10/30/variometro-la-rivincita/             //huge thanks for Vario algorithm
(7) http://code.google.com/p/tinkerit/wiki/SecretVoltmeter             //how to measure battery level using AVR ucontroller

LX Protocol
http://git.xcsoar.org/cgit/master/xcsoar.git/tree/src/Device/Driver/LX/Parser.cpp
http://max.kellermann.name/download/xcsoar/devel/lxnav/V7%20dataport%20specification%202.01.pdf

*/

////////////////////////////////////////
//   Modified by Luis Briones, 2015   //
////////////////////////////////////////

/*
$POV protocol
https://github.com/Turbo87/openvario-protocol
Formato:
$POV,<type>,<value>,<type>,<value>,...*<checksum>

Airspeed

    S: true airspeed in km/h
    Example: $POV,S,123.45*05

Pressure

    P: static pressure in hPa
    Example: $POV,P,1018.35*39

    Q: dynamic pressure in Pa
    Example: $POV,Q,23.3*04

    R: total pressure in hPa
    Example: $POV,R,1025.17*35

Temperature

    T: temperature in °C
    Example: $POV,T,23.52*35

Vario

    E: TE vario in m/s
    Example: $POV,E,2.15*14
--------------------------------------
Example
$POV,P,1018.35,T,23.52,E,2.15*<checksum>

*/

#include <Wire.h>                      //i2c library
#include <Tone.h>                      //tone library, download from url link (3)
#include <Adafruit_BMP085.h>           //https://github.com/adafruit/Adafruit-BMP085-Library
#include <stdlib.h>                    //we need that to use dtostrf() and convert float to string
/////////////////////////////////////////
///////////////////////////////////////// variables that You can test and try
short speaker_pin1 = 8;                //arduino speaker output -
short speaker_pin2 = 9;                //arduino speaker output +
float vario_climb_rate_start = 0.4;    //minimum climb beeping value(ex. start climbing beeping at 0.4m/s)
float vario_sink_rate_start = -1.1;    //maximum sink beeping value (ex. start sink beep at -1.1m/s)
#define SAMPLES_ARR 6                  //define moving average filter array size (2->30), more means vario is less sensitive and slower
#define UART_SPEED 9600                //define serial transmision speed (9600,19200, etc...)
/////////////////////////////////////////
/////////////////////////////////////////
Adafruit_BMP085 bmp085;                //set up bmp085 sensor
Tone     tone_out1;
Tone     tone_out2;
long     Pressure = 101325;
long     dynamic_pressure = 0;
float    Temperature = 0;
float    Altitude;
int      Battery_Vcc = 0;              //variable to hold the value of Vcc from battery
const float p0 = 101325;               //Pressure at sea level (Pa)
unsigned long get_time1 = millis();
unsigned long get_time2 = millis();
unsigned long get_time3 = millis();
boolean  thermalling = false;
int      my_temperature = 1;         //
char     average_pressure_arr[6];    //wee need this array to translate float to string
char     my_temperature_arr[5];      //wee need this array to translate float to string
char     altitude_arr[6];            //wee need this array to translate float to string
char     vario_arr[5];               //wee need this array to translate float to string
char     dynamic_pressurel_arr[6];   //wee need this array to translate float to string
char     total_arr[6];
int      samples=40;
int      maxsamples=50;
float    alt[51];
float    tim[51];
float    beep;
float    Beep_period;
//float    my_temperature;           //
float    presionTotal = 619.38;
static long k[SAMPLES_ARR];

static long Averaging_Filter(long input);
static long Averaging_Filter(long input) // moving average filter function
{
  long sum = 0;
  for (int i = 0; i < SAMPLES_ARR; i++) {
    k[i] = k[i+1];
  }
  k[SAMPLES_ARR - 1] = input;
  for (int i = 0; i < SAMPLES_ARR; i++) {
    sum += k[i];
  }
  return ( sum / SAMPLES_ARR ) ;
}

void play_welcome_beep()                 //play only once welcome beep after turning on arduino vario
{
  for (int aa=300;aa<=1500;aa=aa+100)
  {
    tone_out1.play(aa,200);             // play beep on pin 8 (note,duration)
    tone_out2.play(aa+3,200);           // play beep on pin 9 (note,duration), it is louder if we move aplitude phase
    delay(100);
  }
  for (int aa=1500;aa>=100;aa=aa-100)
  {
    tone_out1.play(aa,200);             // play beep on pin 8 (note,duration)
    tone_out2.play(aa+3,200);           // play beep on pin 8 (note,duration)
    delay(100);
  }
}

long readVcc()                         // function to read battery value - still in developing phase
{
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

void setup()                // setup() function to setup all necessary parameters before we go to endless loop() function
{
  Serial.begin(UART_SPEED);           // set up arduino serial port
  Wire.begin();                       // lets init i2c protocol
  tone_out1.begin(speaker_pin1);      // piezo speaker output pin8 -
  tone_out2.begin(speaker_pin2);      // piezo speaker output pin9 +
  bmp085.begin(BMP085_ULTRAHIGHRES);  // BMP085 ultra-high-res mode, using Pa units
                            
  play_welcome_beep();      //everything is ready, play "welcome" sound
}

void loop(void)
{
  float tempo=millis();
  float vario=0;
  float N1=0;
  float N2=0;
  float N3=0;
  float D1=0;
  float D2=0;
  Pressure = bmp085.readPressure();                                   //get one sample from BMP085 in every loop
  
  
  long average_pressure = Averaging_Filter(Pressure);                   //put it in filter and take average
  Altitude = (float)44330 * (1 - pow(((float)Pressure/p0), 0.190295));  //take new altitude in meters
  //Serial.println(Battery_Vcc);
  for(int cc=1;cc<=maxsamples;cc++){                                   //samples averaging and vario algorithm
    alt[(cc-1)]=alt[cc];
    tim[(cc-1)]=tim[cc];
  };
  alt[maxsamples]=Altitude;
  tim[maxsamples]=tempo;
  float stime=tim[maxsamples-samples];
  for(int cc=(maxsamples-samples);cc<maxsamples;cc++){
    N1+=(tim[cc]-stime)*alt[cc];
    N2+=(tim[cc]-stime);
    N3+=(alt[cc]);
    D1+=(tim[cc]-stime)*(tim[cc]-stime);
    D2+=(tim[cc]-stime);
  };

  vario=1000*((samples*N1)-N2*N3)/(samples*D1-D2*D2);
  if ((tempo-beep)>Beep_period)                          // make some beep
  {
    beep=tempo;
    if (vario>vario_climb_rate_start && vario<15 )
    {
      Beep_period=350-(vario*5);
      tone_out1.play((1000+(100*vario)),300-(vario*5)); //when climbing make faster and shorter beeps
      tone_out2.play((1003+(100*vario)),300-(vario*5));
      thermalling = true;                               //ok,we have thermall in our hands
    } else if ((vario < 0 ) && (thermalling == true))   //looks like we jump out the thermall
    {
      Beep_period=200;
      play_siren();                                   //oo, we lost thermall play alarm
      thermalling = false;
    } else if (vario< vario_sink_rate_start){           //if you have high performace glider you can change sink beep to -0.95m/s ;)
      Beep_period=200;
      tone_out1.play(300,340);
      tone_out2.play(303,340);
      thermalling = false;
    }
  }

  if (millis() >= (get_time2+1000))      //every second get temperature and battery level
  {
    Temperature = bmp085.readTemperature(); // get temperature in celsius from time to time, we have to divide that by 10 to get XY.Z
    //my_temperature = (float)Temperature/10;
    my_temperature = Temperature;
    Battery_Vcc =(readVcc()/42)+1000;    // get voltage and prepare in percentage
    get_time2 = millis();
  }

if (millis() >= (get_time3+333))       //every 1/3 second send NMEA output over serial port
  {
    
    // $POV,P,1018.35,T,23.52,E,2.15*<checksum>
    
    String str_out =                                                                                  //combine all values and create part of NMEA data string output
      String("POV,")
      +String("P,")+String(dtostrf((float)average_pressure/100,0,2,average_pressure_arr))+String(",") // Static Pressure
      +String("Q,")+String(dtostrf(dynamic_pressure,0,2,dynamic_pressurel_arr))+String(",")           // Total Pressure in hPa
      //+String("A,")+String(dtostrf(Altitude,0,2,altitude_arr))+String(",")                          // Altitude, not in $POV protocol yet
      +String("E,")+String(dtostrf((vario),0,2,vario_arr))+String(",")                                // Vario
      +String("T,")+String(dtostrf((my_temperature),0,2,my_temperature_arr))+String(",")              // Temperaure
      
      ; // fin de la sentencia NMEA*/
    
    /*
     String str_out =                                                                 //combine all values and create part of NMEA data string output
      String("LK8EX1"+String(",")+String(average_pressure,DEC)+ String(",")+String(dtostrf(Altitude,0,0,altitude_arr))+String(",")+
      String(dtostrf((vario*100),0,0,vario_arr))+String(",")+String(my_temperature,DEC)+String(",")+String(Battery_Vcc,DEC)+String(","));
     */
    
    
    
    unsigned int checksum_end,ai,bi;                                                 // Calculating checksum for data string
    for (checksum_end = 0, ai = 0; ai < str_out.length(); ai++)
    {
      bi = (unsigned char)str_out[ai];
      checksum_end ^= bi;
    }
Serial.print(vario);
    Serial.println(" m/s");

/*
  //creating now NMEA serial output for LK8000. LK8EX1 protocol format:
    //$LK8EX1,pressure,altitude,vario,temperature,battery,*checksum
    Serial.print("$");                     //print first sign of NMEA protocol
    Serial.print(str_out);                 // print data string
    Serial.print("*");                     //end of protocol string
    Serial.println(checksum_end,HEX);      //print calculated checksum on the end of the string in HEX
*/
 //creating now NMEA serial output for XCSOAR. $POV protocol format:
/*    Serial.print("$");                     //print first sign of NMEA protocol
    Serial.print(str_out);                 // print data string
    Serial.print("*");                     //end of protocol string
    Serial.println(checksum_end,HEX);      //print calculated checksum on the end of the string in HEX
*/
/*Serial.println("");

    Serial.print("Temperatura = ");
    Serial.print(Temperature);
    Serial.println(" *C");
    
    Serial.print("Pressure = ");
    Serial.print(Pressure);
    Serial.println(" Pa");
    */
    get_time3 = millis();
  }
}
//The End
