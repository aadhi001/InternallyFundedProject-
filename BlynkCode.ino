#include <OneWire.h>
#include <DallasTemperature.h>

#define BLYNK_PRINT SwSerial
#define echoPin 7
#define trigPin 3

#define TdsSensorPin A1
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point

#define ONE_WIRE_BUS 4
const int watermeterPin = 2;
const int RELAY_PIN = A5;


volatile int  pulse_frequency;
unsigned int  literperhour;
unsigned long currentTime, loopTime;
byte sensorInterrupt = 0;

int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
int forcestop=0,emailflag=0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;

float cel=0.0;
// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);  

// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

long duration;
int distance;


#include <SoftwareSerial.h>
SoftwareSerial SwSerial(10, 11); // RX, TX

#include <BlynkSimpleStream.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "O7n0PVuZbivMYuQQdRhjWeqpL7x_0FIu";
BlynkTimer timer;

// This function sends Arduino's up time every second to Virtual Pin (5).
// In the app, Widget's reading frequency should be set to PUSH. This means
// that you define how often to send data to Blynk App.

void setup()
{
  pinMode(watermeterPin, INPUT);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(TdsSensorPin,INPUT);
  // Debug console
  SwSerial.begin(9600);

  // Blynk will work through Serial
  // Do not read or write this serial manually in your sketch
  Serial.begin(9600);
  attachInterrupt(sensorInterrupt, getFlow, FALLING);                                 
  currentTime = millis();
  loopTime = currentTime;
  Blynk.begin(Serial, auth);
  sensors.begin();

  

  // Setup a function to be called every second
  timer.setInterval(1000L, myTimerEvent);
}
void myTimerEvent()
{
  sensors.requestTemperatures(); 

 
      Blynk.virtualWrite(V4,pulse_frequency * 60 / 7.5);
      
      Blynk.virtualWrite(V5, sensors.getTempCByIndex(0));  
      Blynk.virtualWrite(V6,getdistance());
      Blynk.virtualWrite(V2,getTDS());
      Blynk.virtualWrite(V3,analogRead(A3)*(5.0/1024.0));
      pulse_frequency=0;
      
      if(getdistance()<80 && forcestop==0)
      {
        startmotor();
        }
      else
      {
        stopmotor();
        }

      if(getTDS()>1000 && emailflag==0)
      {
        Blynk.email("aadhithyanpandian@gmail.com","Unsafe Water","The tds level has reached unsafe levels!!");
        emailflag=1;
        }
      if(getTDS()<500 && emailflag==1)
      {
        emailflag=0;
        } 
     
}

long getdistance()
{
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
  
 duration=pulseIn(echoPin,HIGH);
 distance=(duration*0.034/2);

    return (20-distance)*5;
}

void loop()
{
   static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }   
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 800U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
      tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");
      
   }

  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.run();
  timer.run(); // Initiates BlynkTimer
  
}


void getFlow ()
{ 
   pulse_frequency++;
} 

void startmotor()
{
  digitalWrite(RELAY_PIN, HIGH);
}

void stopmotor()
{
  digitalWrite(RELAY_PIN, LOW);
}

int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}

float getTDS()
{
  return tdsValue;
  }


BLYNK_WRITE(V7)
{
  forcestop=param.asInt();
  }
