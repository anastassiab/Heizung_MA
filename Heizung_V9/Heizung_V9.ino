
#include "analogWrite.h"
#include "max6675.h"


char temp1 = 0 ;
String temp1_1 = "";
boolean not_number =false;
double sign = 1;


int pinThermoDO = 4;
int pinThermoCS_1 = 16;
int pinThermoCS_2 = 17;
int pinThermoCS_3 = 5;
int pinThermoCS_4 = 18;
int pinThermoCLK = 2;


MAX6675 thermocouple_1(pinThermoCLK, pinThermoCS_1, pinThermoDO);
MAX6675 thermocouple_2(pinThermoCLK, pinThermoCS_2, pinThermoDO);
MAX6675 thermocouple_3(pinThermoCLK, pinThermoCS_3, pinThermoDO);
MAX6675 thermocouple_4(pinThermoCLK, pinThermoCS_4, pinThermoDO);

// PWM-Signal
int PWM_Frequency = 50;
int Channel = 0;      //rot
int Channel2 = 1;      //gelb
int PWM_Resolution = 8;
int pin_PWM_1 = 12;         //Relay 1 (rotes Kabel)
int pin_PWM_2 =14;          //Relay 2 (gelbes Kabel)

int PWM1_DutyCycle = 51;          //20% von 100% in Bit
int PWM2_DutyCycle = 51;          //20% von 100% in Bit 

// Potentiometer

int pin_Poti_1 = 34; // Anaqlog pin für Poti
int potValue_1 = 0;
//float voltage = 0;
int pin_Poti_2 = 35; // Analog pin für Poti
int potValue_2 = 0;


//PID Regler für Heizung
#include "PID_v1.h"

double SetpointTemperatur1, InputTemperatur1, OutputPWM1;   //Variable für PID Regelung
double SetpointTemperatur2, InputTemperatur2, OutputPWM2;   //Variable für PID Regelung
double Kp=6, Ki=0.01, Kd=0;       //Parameter für PID Regelung
double PIDSAMPLETIME_MS = 1000 ;  // milliseconds
PID TemperaturControl1(&InputTemperatur1, &OutputPWM1, &SetpointTemperatur1, Kp, Ki, Kd, DIRECT); 
PID TemperaturControl2(&InputTemperatur2, &OutputPWM2, &SetpointTemperatur2, Kp, Ki, Kd, DIRECT); 





double doubleMap(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/*
float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
*/

void setup() {
  Serial.begin(115200);

  Serial.println("MAX6675 test");
  // wait for MAX chip to stabilize
  delay(500);

  pinMode(pin_Poti_1, INPUT);//_______pin Poti als Input definiert
  pinMode(pin_Poti_2, INPUT);
  pinMode(pin_PWM_1,OUTPUT);//_______PWM als Output definiert
  pinMode(pin_PWM_2,OUTPUT);


  //ESP32 Konfiguration PWM-Signal 1 (rot)
  ledcSetup(Channel,PWM_Frequency, PWM_Resolution);
  ledcAttachPin(pin_PWM_1,Channel);

  //ESP32 Konfiguration PWM-Signal 2 (gelb)
  ledcSetup(Channel2,PWM_Frequency, PWM_Resolution);
  ledcAttachPin(pin_PWM_2,Channel2);


  //PID Setup

//  SetpointTemperatur1 = -22.44;

  TemperaturControl1.SetMode(AUTOMATIC); // PID-Regler an- (AUTOMATIC) oder ausschalten (MANUAL)
  TemperaturControl1.SetSampleTime(PIDSAMPLETIME_MS);
  TemperaturControl1.SetOutputLimits(0, 51.2); //Begrenzung Regelbereich ( PWM-Grenzen an Output-Pin werden begrenzt,hier 0-20%; 20% von 8Bit)


  //SetpointTemperatur2 = 35;

  TemperaturControl2.SetMode(AUTOMATIC); // PID-Regler an- (AUTOMATIC) oder ausschalten (MANUAL)
  TemperaturControl2.SetSampleTime(PIDSAMPLETIME_MS);
  TemperaturControl2.SetOutputLimits(0, 51.2); //Begrenzung Regelbereich ( PWM-Grenzen an Output-Pin werden begrenzt,hier 0-20%; 20% von 8Bit)

  
}




void loop() {
  Solltemp1();


 // int potValue_1 = analogRead(pin_Poti_1);

  
 // SetpointTemperatur1 = doubleMap(double(potValue_1), 0, 4095, 0, 80);
  
 // Serial.print("Poti 1 = ");
 // Serial.print(potValue_1);
//  Serial.print("\t");
 // Serial.print(" ");

// Ausgabe der Daten von Heizelement 1
  InputTemperatur1 = (thermocouple_1.readCelsius());
  Serial.print(InputTemperatur1);
 // Serial.print("\t");
  Serial.print(" ");
  
  Serial.print(SetpointTemperatur1);
  //Serial.print("\t");
 Serial.print(" ");


 // InputTemperatur1 = double(IstTemperatur1);
  TemperaturControl1.Compute();
  analogWrite(12, OutputPWM1);
  
  double PWM_1_Wert = (OutputPWM1 *100) / 256; // Umrechnung in Prozent
  
  Serial.print(PWM_1_Wert);
  //Serial.print("\t");
   Serial.print(" ");
   
  Serial.print(OutputPWM1);
   Serial.print("    ");






// Ausgabe der Daten von Heizelement 2


  int potValue_2 = analogRead(pin_Poti_2);


// SetpointTemperatur2 = doubleMap(double(potValue_2), 0, 4095, 0, 80);


  Serial.print(potValue_2);
 //Serial.print("\t");
  Serial.print(" ");

 
  InputTemperatur2 = (thermocouple_2.readCelsius());
  Serial.print(InputTemperatur2);
 // Serial.print("\t");
  Serial.print(" ");
  
  Serial.print(SetpointTemperatur2);
  //Serial.print("\t");
   Serial.print(" ");
  
  TemperaturControl2.Compute();
  analogWrite(14, OutputPWM2);
  
  double PWM_2_Wert = (OutputPWM2 *100) / 256; // Umrechnung in Prozent
  
  Serial.print(PWM_2_Wert);
  //Serial.print("\t");
 Serial.print(" ");
  
  Serial.println(OutputPWM2);




/*
  //Regelung Temperatur durch PID um ein fester Wert
  InputTemperatur1 = (thermocouple_1.readCelsius());
  Serial.println("gelesene T1 =" + String(thermocouple_1.readCelsius()));
  TemperaturControl1.Compute();
  analogWrite(12,OutputPWM1);

  Serial.println("PWM OutputSignal 1 =" + String(OutputPWM1));
  Serial.println("Soll-Temperatur 1 ="  + String(SetpointTemperatur1));
 

  InputTemperatur2 = (thermocouple_2.readCelsius());
  Serial.println("gelesene T2 =" + String(thermocouple_2.readCelsius()));
  TemperaturControl2.Compute();
  analogWrite(14,OutputPWM2);

  Serial.println("PWM OutputSignal 2 =" + String(OutputPWM2));
  Serial.println("Soll-Temperatur 2 ="  + String(SetpointTemperatur2));
*/

  /*
  if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
  windowStartTime += WindowSize;
  }
  if (Output < millis() - windowStartTime) digitalWrite(RELAY_PIN, HIGH);
  else digitalWrite(RELAY_PIN, LOW);
 */

/*
  //PWM Signal 1
  if (thermocouple_1.readCelsius() <= 35) 
    {
     ledcWrite(Channel, PWM1_DutyCycle);
     Serial.print("Heizung AN");
      Serial.print("Temp_1 C = "); 
   Serial.println(thermocouple_1.readCelsius());
    }
  else{
    ledcWrite(Channel, 0);
     Serial.print("Heizung AUS"); 
     Serial.print("Temp_1 C = "); 
   Serial.println(thermocouple_1.readCelsius());
  }
 //PWM Signal 2
  if (thermocouple_2.readCelsius() <= 35) 
    {
     ledcWrite(Channel2, PWM2_DutyCycle);
       Serial.print("Heizung AN");
      Serial.print("Temp_2 C = "); 
   Serial.println(thermocouple_2.readCelsius());
    }
  else{
    ledcWrite(Channel2, 0);
    Serial.print("Heizung AUS");
      Serial.print("Temp_2 C = "); 
   Serial.println(thermocouple_2.readCelsius());
  }

  // basic readout test, just print the current temp
  
  // Serial.print("Temp_1 C = "); 
  // Serial.println(thermocouple_1.readCelsius());
  

  // Serial.print("Temp_2 C = "); 
  //Serial.println(thermocouple_2.readCelsius());

  //  Serial.print("Temp_3 C = "); 
  //Serial.println(thermocouple_3.readCelsius());

  // Serial.print("Temp_4 C = "); 
  //Serial.println(thermocouple_4.readCelsius());
  */

delay(250);
   
}

// Online-Anpassung der Sollwert Temperatur 
void Solltemp1() {                 
  if (Serial.available() > 0) {
   
    temp1 = Serial.read(); 
    
    if ((temp1 >= '0') && (temp1 <= '9')) {
      
        temp1_1 += temp1;
    }  
    else if (temp1 == '-') {
      sign = -1; 
    }
 
     
     else if (temp1 == '\n') {
        if (not_number) {
            Serial.println("not a number");
        }
        else { 
      SetpointTemperatur1 = temp1_1.toDouble() * sign;
            Serial.print("Solltemp1 = ");
            Serial.println(SetpointTemperatur1);
           
        }
     not_number = false;
     temp1_1 = "";
  }
  else {
      
      not_number =true;
      sign = 1;
  }
      
  }
}
      

    
  
