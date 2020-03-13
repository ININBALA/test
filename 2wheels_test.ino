// Test MD03a / Pololu motor with encoder
// speed control (PI), V & I display
// Credits:
//   Dallaby   http://letsmakerobots.com/node/19558#comment-49685
//   Bill Porter  http://www.billporter.info/?p=286
//   bobbyorr (nice connection diagram) http://forum.pololu.com/viewtopic.php?f=15&t=1923


#define InA1            4                      // INA motor pin
#define InB1            5                      // INB motor pin
#define InA2            6                      // INA motor pin
#define InB2            7                      // INB motor pin
#define PWM1            13                       // PWM motor pin
#define encodPinA1      31                       // encoder A pin
#define encodPinB1      21                       // encoder B pin
#define encodPinA1      30                       // encoder A pin
#define encodPinB1      18                       // encoder B pin
//#define Vpin            0                       // battery monitoring analog pin
//#define Apin            1                       // motor current monitoring analog pin

#define CURRENT_LIMIT   1000                     // high current warning
#define LOW_BAT         10000                   // low bat warning
#define LOOPTIME        100                     // PID loop time
#define NUMREADINGS     10                      // samples for Amp average

int readings[NUMREADINGS];
unsigned long lastMilli = 0;                    // loop timing
unsigned long lastMilliPrint = 0;               // loop timing
int speed_req1 = 0;                            // speed (Set Point)
int speed_act1 = 0;                              // speed (actual value)
int PWM_val1 = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int speed_req2 = 0;                            // speed (Set Point)
int speed_act2 = 0;                              // speed (actual value)
int PWM_val2  = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int voltage = 0;                                // in mV
int current = 0;                                // in mA
volatile long count1 = 0;                        // rev counter
volatile long count2 = 0;
float Kp =   0.3;                                // PID proportional control Gain
float Kd =    1;                                // PID Derivitave control gain


void setup() {
 analogReference(EXTERNAL);                            // Current external ref is 3.3V
 Serial.begin(115600);
 pinMode(InA1, OUTPUT);
 pinMode(InB1, OUTPUT);
 pinMode(InA2, OUTPUT);
 pinMode(InB2, OUTPUT);
// pinMode(PWM1, OUTPUT);
 pinMode(encodPinA1, INPUT);
 pinMode(encodPinB1, INPUT);
 pinMode(encodPinA2, INPUT);
 pinMode(encodPinB2, INPUT);
 /*
 digitalWrite(encodPinA1, HIGH);                      // turn on pullup resistor
 digitalWrite(encodPinB1, HIGH);*/
 attachInterrupt(2, rencoder1, FALLING);
 attachInterrupt(5, rencoder2, FALLING);
 for(int i=0; i<NUMREADINGS; i++)   readings[i] = 0;  // initialize readings to 0

 //analogWrite(PWM1, PWM_val);
 analogWrite(InA1, LOW);
 analogWrite(InB1, PWM_val1);
 analogWrite(InA2, LOW);
 analogWrite(InB2, PWM_val2);
}

void loop() {
 getParam();                                                                 // check keyboard ao3
 if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter tmed loop
   lastMilli = millis();
   getMotorData();                                                           // calculate speed, volts and Amps
   PWM_val1= updatePid(PWM_val1, speed_req1, speed_act1);                        // compute PWM value
   PWM_val2= updatePid(PWM_val2, speed_req2, speed_act2);
   analogWrite(InB1, PWM_val1);                                               // send PWM to motor
   analogWrite(InB2, PWM_val2);
 }
 printMotorInfo();                                                           // display data
}

void getMotorData()  {                                                        // calculate speed, volts and Amps
static long countAnt1 = 0;                                                   // last count
static long countAnt2 = 0;
 speed_act1 = ((count1 - countAnt1)*(60*(1000/LOOPTIME)))/(16*29);          // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
 countAnt1 = count1;       
 speed_act2 = ((count2 - countAnt2)*(60*(1000/LOOPTIME)))/(16*29);          // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
 countAnt2 = count2;                 
/* voltage = int(analogRead(Vpin) * 3.22 * 12.2/2.2);                          // battery voltage: mV=ADC*3300/1024, voltage divider 10K+2K
 current = int(analogRead(Apin) * 3.22 * .77 *(1000.0/132.0));               // motor current - output: 130mV per Amp
 current = digital_smooth(current, readings);                                // remove signal noise*/
}

int updatePid(int command, int targetValue, int currentValue)   {             // compute PWM value
float pidTerm = 0;                                                            // PID correction
int error=0;                                  
static int last_error=0;                            
 error = abs(targetValue) - abs(currentValue);
 pidTerm = (Kp * error) + (Kd * (error - last_error));                            
 last_error = error;
 return constrain(command + int(pidTerm), 0, 255);
}

void printMotorInfo()  {                                                      // display data
 if((millis()-lastMilliPrint) >= 500)   {                    
   lastMilliPrint = millis();
   Serial.print("SP1:");             Serial.print(speed_req1);  
   Serial.print("  RPM1:");          Serial.print(speed_act1);
   Serial.print("  PWM1:");          Serial.print(PWM_val1);  
   Serial.print("SP1:");             Serial.print(speed_req2);  
   Serial.print("  RPM1:");          Serial.print(speed_act2);
   Serial.print("  PWM1:");          Serial.print(PWM_val2);  
   Serial.print("  V:");            Serial.print(float(voltage)/1000,1);
   Serial.print("  mA:");           Serial.println(current);

   if (current > CURRENT_LIMIT)               Serial.println("*** CURRENT_LIMIT ***");                
   if (voltage > 1000 && voltage < LOW_BAT)   Serial.println("*** LOW_BAT ***");                
 }
}

 void rencoder1() {
    if (digitalRead(encodPinB1) == HIGH) {
      if (digitalRead(encodPinA1) == LOW) {
        count1++;
      }
      else {
        count1--;
      }
    }
    else {
      if (digitalRead(encodPinA1) == LOW) {
        count1--;
      }
      else {
        count1++;
      }
    }
  }


 void rencoder2() {
    if (digitalRead(encodPinB2) == HIGH) {
      if (digitalRead(encodPinA2) == LOW) {
        count2++;
      }
      else {
        count2--;
      }
    }
    else {
      if (digitalRead(encodPinA1) == LOW) {
        count2--;
      }
      else {
        count2++;
      }
    }
  }
int getParam()  {
char param1, cmd1, param2, cmd2;
 if(!Serial.available())    return 0;
 delay(10);                  
 param1 = Serial.read();                              // get parameter byte
 if(!Serial.available())    return 0;
 cmd1 = Serial.read();                                // get command byte
 if(!Serial.available())    return 0;
 delay(10);                  
 param2 = Serial.read();                              // get parameter byte
 if(!Serial.available())    return 0;
 cmd2 = Serial.read();                                // get command byte
 Serial.flush();
 switch (param1) {
   case 'v':                                         // adjust speed
     if(cmd1=='+')  {
       speed_req += 20;
       if(speed_req>400)   speed_req=400;
     }
     if(cmd1=='-')    {
       speed_req -= 20;
       if(speed_req<0)   speed_req=0;
     }
     break;
   case 's':                                        // adjust direction
     if(cmd1=='+'){
       digitalWrite(InA1, LOW);
       digitalWrite(InB1, HIGH);
     }
     if(cmd1=='-')   {
       digitalWrite(InA1, HIGH);
       digitalWrite(InB1, LOW);
     }
     break;
   case 'o':                                        // user should type "oo"
     digitalWrite(InA1, LOW);
     digitalWrite(InB1, LOW);
     speed_req = 0;
     break;
   default:
     Serial.println("???");
   }
   switch (param2) {
   case 'v':                                         // adjust speed
     if(cmd2=='+')  {
       speed_req += 20;
       if(speed_req>400)   speed_req=400;
     }
     if(cmd2=='-')    {
       speed_req -= 20;
       if(speed_req<0)   speed_req=0;
     }
     break;
   case 's':                                        // adjust direction
     if(cmd2=='+'){
       digitalWrite(InA1, LOW);
       digitalWrite(InB1, HIGH);
     }
     if(cmd2=='-')   {
       digitalWrite(InA1, HIGH);
       digitalWrite(InB1, LOW);
     }
     break;
   case 'o':                                        // user should type "oo"
     digitalWrite(InA1, LOW);
     digitalWrite(InB1, LOW);
     speed_req = 0;
     break;
   default:
     Serial.println("???");
   }
}
/*
int digital_smooth(int value, int *data_array)  {    // remove signal noise
static int ndx=0;                                                        
static int count=0;                          
static int total=0;                          
 total -= data_array[ndx];              
 data_array[ndx] = value;                
 total += data_array[ndx];              
 ndx = (ndx+1) % NUMREADINGS;                                
 if(count < NUMREADINGS)      count++;
 return total/count;
}*/
