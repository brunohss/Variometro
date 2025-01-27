#include "Wire.h"
#include <BMP080.h>
MS5611 ms5611;
#include <KalmanFilter.h>
KalmanFilter a_filter;    //altitude filter
#include <SoftwareSerial.h>
#include "DHT.h"
#define DHTPIN 2  
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE); 
const int key = 12;
SoftwareSerial PDV(10, 11);
const byte buff_len = 90;
char CRCbuffer[buff_len];
float a1,a2;//Altitudes

void setup()
{  
        pinMode(key, OUTPUT);
        Serial.begin(9600); 
        Wire.begin();  
        PDV.begin(9600);    
        dht.begin();  
        while(!ms5611.begin())
        {
        Serial.println("Erreur MS5611 sensor");
        return;
        }
        checkSettings();
}  

void loop()
{   
        digitalWrite(key, LOW); 
        int crc;
        String CRC;
        byte buf[4];
//Température en °C
        float h = dht.readHumidity();
        float t = dht.readTemperature();
        if (isnan(h) || isnan(t)) 
        {
        Serial.println("Erreur DHT");
        return;
        }
        Serial.print("Temperature C: ");
        Serial.println(t);
//Humidité en %  
        Serial.print("Humidity %: ");
        Serial.println(h);
//Pression en Pa
        uint32_t rawPressure = ms5611.readRawPressure();
        long p = ms5611.readPressure();
        float Pa = p;
        Serial.print("Pression Pa: ");
        Serial.println(Pa);
//Altitude en m
        float a = ms5611.getAltitude(p);
        float a1 = a_filter.Filter(a); 
        Serial.print("Altitude m: ");
        if (a1 >= 0.0) Serial.print(" "); 
        Serial.println(a1,2);
//Vario en m/s  
        float va = a1-a2;
        a2 = a1;
        Serial.print("Vario m/s: ");
        if (a1 >= 0.0) Serial.print(" "); 
        Serial.println(va,2);  
// Calcul de la pression de vapeur d'eau saturante dans l'air en Pascal
        float Psat = (611.213*exp((17.5043*t)/(241.02+t)));
// Calcul de la constante de l'air humide
        float Rh = (287.06/(1-(((h/100)/Pa)*(1-(287.06/461)))));
// Calcul de la masse volumique de l'air humide en kg/m3
        float rho = (Pa/(Rh*(t+273.15)));
// Calcul de densityratio
        int dsr = 1024*sqrt (1.22/rho);
        Serial.print("Rho kg/m3: ");
        Serial.println(rho);
        Serial.print("Dsr : ");
        Serial.println(dsr);
 //Lecture Pitot 
        int n = Wire.requestFrom(0x28, 4);
        if (n == 4)  
        {
        Wire.readBytes( buf, 4);
        }
        else  
        {
        Serial.println("Erreur,pas de MS5915");
        return;
        }
        unsigned int rawP = word( buf[0], buf[1]);
        rawP &= 0x3FFF;
        float x =((float) rawP - 1638)*10000/(1.293*13107);
        if (x < 0)
        {
        x =0 ;
        }
//Vitesse Propre en 0,1 m/s
        int vp=10*sqrt(x); 
        String msg =  "$PDVDV,";      
        int ps = Pa/10;
        int vz = 10*va;
        int al = a1;      
        msg = msg + vz + "," + vp + "," + dsr +"," + al + "," + ps +",1*" ; 
        msg.toCharArray(CRCbuffer,sizeof(CRCbuffer));
        crc = convertToCRC(CRCbuffer);
        if (crc < 16) Serial.println("Erreur CRC");
        CRC = String (crc, HEX);
        msg = msg + CRC;
        Serial.println(msg);
        PDV.println(msg);
//Température en 10x°K
        int T = (t + 273)*10;
        int H = h;
        msg = "$PDVVT,";
        msg = msg + T + "," + H +"*";
        msg.toCharArray(CRCbuffer,sizeof(CRCbuffer));
        crc = convertToCRC(CRCbuffer);
        if (crc < 16) Serial.println("Erreur CRC");
        CRC = String (crc, HEX);
        msg = msg + CRC;
        Serial.println(msg);
        PDV.println(msg);
        delay(1000);
}
//Fin de boucle
//Fonctions
void checkSettings()
      {
      Serial.print("Oversampling: ");
      Serial.println(ms5611.getOversampling());
      }

byte convertToCRC(char *buff) 
      {
// CRC: XOR each byte with previous for all chars between '$' and '*'
      char c;
      byte i;
      byte start_with = 0;
      byte end_with = 0;
      byte CRC = 0;
      for (i = 0; i < buff_len; i++) 
      {
      c = buff[i];
      if(c == '$')
      {
      start_with = i;
      }
      if(c == '*')
      {
      end_with = i;
      }      
      }
      if (end_with > start_with)
      {
      for (i = start_with+1; i < end_with; i++)
      { 
// XOR every character between '$' and '*'
      CRC = CRC ^ buff[i] ;  
// Compute CRC
      }
      }
      else 
      { 
      Serial.println("CRC ERROR");
      }
      return CRC;
 }
//Fin
 
