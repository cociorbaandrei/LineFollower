#include <QTRSensors.h>
#include <IRremote.h>

QTRSensorsRC qtrrc((unsigned char[]) {10, 4, 7, 8, 14, 15, 16, 17} ,8, 2000); // sensor connected through analog pins A0 - A5 i.e. digital pins 14-19
unsigned int sensorValues[8];

int motor_left[] = {9, 6};
int motor_right[] = {5, 3};
int motorSpeed = 255;
int SampleTime = 50; 
int _speed = 255;
int RECV_PIN = 13;
IRrecv irrecv(RECV_PIN);
decode_results results;

int PWM_LEFT = 0;
int PWM_RIGHT = 0;

int RIGHT        = 0;
int MIDDLE_RIGHT = 1;
int MIDDLE_LEFT  = 2;
int LEFT         = 3;


unsigned long lastTime;

double speedRatio = 1;
double Input, Output, Setpoint = 3.5;
double ITerm, lastInput;
double kp, ki, kd;
double outMin, outMax;

bool inAuto = false;
bool printSensorValues = false;

#define MANUAL 0
#define AUTOMATIC 1

void drive_forward(int _speed){
  analogWrite(motor_left[0], _speed);
  analogWrite(motor_left[1], 0);
  
  analogWrite(motor_right[0], _speed);
  analogWrite(motor_right[1], 0);
}


void drive_backward(int _speed){
  analogWrite(motor_left[0], 0);
  analogWrite(motor_left[1], _speed );
  
  analogWrite(motor_right[0], 0);
  analogWrite(motor_right[1], _speed);;
    
}

void turn_left(int _speed){
  analogWrite(motor_left[0], LOW);
  analogWrite(motor_left[1], LOW);

  analogWrite(motor_right[0], _speed);
  analogWrite(motor_right[1], LOW);
}

void turn_hardleft(int _speed){
  analogWrite(motor_left[0], LOW);
  analogWrite(motor_left[1], _speed);
  
  analogWrite(motor_right[0], _speed);
  analogWrite(motor_right[1], LOW);
}


void turn_right(int _speed){
  analogWrite(motor_left[0], _speed);
  analogWrite(motor_left[1], LOW);
  
  analogWrite(motor_right[0], LOW);
  analogWrite(motor_right[1], LOW);
}

void turn_hardright(int _speed){
  analogWrite(motor_left[0], _speed);
  analogWrite(motor_left[1], LOW);
  
  analogWrite(motor_right[0], LOW);
  analogWrite(motor_right[1], _speed);
}



void motor_stop(){
  digitalWrite(motor_left[0], LOW);
  digitalWrite(motor_left[1], LOW);
  
  digitalWrite(motor_right[0], LOW);
  digitalWrite(motor_right[1], LOW);
  delay(25);
}

void setup() {
  for(int i = 0; i < 2; i++){
    pinMode(motor_left[i], OUTPUT);
    pinMode(motor_right[i], OUTPUT);
  }
  
  irrecv.enableIRIn(); 
  Serial.begin(9600);
  printSensorValues = false;
  
  SetTunings(72,.4,4000);
  SetMode(AUTOMATIC);
  SetOutputLimits(-255,255);


   if(inAuto){
      for (int i = 0; i < 200; i++){ 
         qtrrc.calibrate();   
       //  drive_backward(255);
         delay(20); 
       //   turn_hardright(255);
          // delay(300); 
      }
  }
  
}



void loop() {
   if (irrecv.decode(&results)) {
      Serial.println(results.value, HEX);
      switch(results.value){
        case 0xF4BA2988:
         // canFollowLine = true;
          SetMode(AUTOMATIC);
          break;
        case 0x8B8510E8:
         // canFollowLine = false;
         SetMode(MANUAL);
          break;
        case 0xE13DDA28:
        //   powerPercent = 40;
          break;
        case 0xAD586662 :
            //powerPercent = 50;
          break;
        case 0x273009C4 :
           //powerPercent = 70;
          break;
        case 0xF5999288:
        
          // powerPercent = 80;
          break;
        case 0x731A3E02 :
        //  powerPercent = 100;
          break;
        case 0xC26BF044:
          drive_forward(255);
         // Serial.println("Inainte");
          break;
        case 0x53801EE8:
         // Serial.println("La stanga");
          
          turn_left(255);
          break;
        case 0x758C9D82:
         // Serial.println("La dreapta");
           turn_right(255);
           
           break;
        case 0xC4FFB646:
         // Serial.println("Inapoi");
          drive_backward(255);
          break;
        default:
          motor_stop();
          break;
      }
      irrecv.resume(); // Receive the next value
  }

  if(inAuto){
    PID();
    computeSpeeds();
    powerMotors();
  }
}

void clamp(int * value, int MIN, int MAX){
  *value = *value > MAX ? MAX : (*value < MIN ? MIN : *value);  
}


void powerMotors(){
  if(!inAuto) return;
  analogWrite(motor_left[0], int(PWM_LEFT * speedRatio));
  analogWrite(motor_right[0], int(PWM_RIGHT * speedRatio));
  analogWrite(motor_right[1], 0); 
  analogWrite(motor_left[1], 0);
}

void computeSpeeds(){
   if(Output<0) // Turn right
  {
    PWM_LEFT = motorSpeed;
    PWM_RIGHT = motorSpeed - abs(int(Output));
  }else // Turn left
  {
    PWM_LEFT = motorSpeed - int(Output);
    PWM_RIGHT = motorSpeed;
  }
  clamp(&PWM_LEFT, 0, 255);
  clamp(&PWM_RIGHT, 0, 255);
}


void PID()
{
   if(!inAuto) return;
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      Input = qtrrc.readLine(sensorValues)/1000; 
      double error = Setpoint - Input;
      ITerm+= (ki * error);
      if(ITerm> outMax) ITerm= outMax;
      else if(ITerm< outMin) ITerm= outMin;
      double dInput = (Input - lastInput);
 
      /*Compute PID Output*/
      Output = kp * error + ITerm - kd * dInput;
      if(Output> outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
 
      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
      
      if(printSensorValues){
        for(int i = 0;i < 8;i++){
          Serial.print(float(sensorValues[i]));
          Serial.print("     ");
        }
        Serial.println("");
      }
   }
}
 
void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}
 
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
 
void SetOutputLimits(double Min, double Max)
{
   if(Min > Max) return;
   outMin = Min;
   outMax = Max;
    
   if(Output > outMax) Output = outMax;
   else if(Output < outMin) Output = outMin;
 
   if(ITerm> outMax) ITerm= outMax;
   else if(ITerm< outMin) ITerm= outMin;
}
 
void SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    { 
        Initialize();
    }
    inAuto = newAuto;
}
 
void Initialize()
{
   lastInput = Input;
   ITerm = Output;
   if(ITerm> outMax) ITerm= outMax;
   else if(ITerm< outMin) ITerm= outMin;
}
