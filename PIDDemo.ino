/*
 * This is code for a PID demonstration based on Tim Wescott's 
 * YouTube video  https://www.youtube.com/watch?v=2elEXcv0AV8&t=128s
 * 
 */
// Streaming supports Serial << "text"; (like C++)
#include <Streaming.h>
#include <Adafruit_MotorShield.h>

// #include "ControlDemo.h"
//Apparently Arduino doesn't like to let you write include files or handle separate modules

// definitions
#define UL unsigned long

// constants 
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const char ESC = 0x1B;

// globals
int     potCount = 0;        // value read from the pot
float   angleRead;
UL   sTime;  // for tic(), toc()
UL   gTime;  // global for gTime = toc()
UL   startMSec;
// motor related
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Select which 'port' M1, M2, M3 or M4. In this case, M4
Adafruit_DCMotor *myMotor = AFMS.getMotor(4);

void setup() {
  // Set up serial port
  Serial.begin(115200);
  Help();
  AFMS.begin();  // create with the default frequency 1.6KHz
  myMotor->run(RELEASE); // make sure motor is stopped
}

void loop() {
  PID();
}

// functions are alphabetical

/*
 * in “Multirate Filtering and Filter Banks” by Shlomo Engelberg (Circuit Cellar #324 July 2017); 
 * the author describes the moving sum (boxcar integrator) low pass filter and explains that flipping the sign 
 * of every other coefficient converts the low pass filter to a high pass filter.  
 * He then says that by every other term you can get two sub-sums, a and b.  
 * LP = a + b and HP = a – b.
 * Try setting up a class for the integrator and derivative calculations
 */

 class IntDeriv {
  public:
  float oddSum;
  float evenSum;
  boolean odd;
  const float windup = 100000.;

  IntDeriv(){
    this->Clear();
  }
  
  void AddTerm( float x, float dt ){
    this->odd = !this->odd;
    if( this->odd ){
      this->oddSum += x * dt * 1e-6;
    }
    else{
      this->evenSum += x * dt * 1e-6;
    }
  }
  void Clear(){
    this->oddSum = 0;
    this->evenSum = 0;
    this->odd = false;
  }
  float getDeriv(){
    if ( this->odd ){
      return this->oddSum - this->evenSum;
    }
    else {
      return this->evenSum - this->oddSum;
    }
    
  }
  float getIntegral(){
    return constrain( this->evenSum + this->oddSum, -this->windup, this->windup );
  }
 }  ID;
 

void Help() {
  Serial << "PID controller demo\r\n";
  Serial << "q - stop motor, restore defaults\r\n";
  Serial << "a<number> - set target angle\r\n";
  Serial << "p<number> - set proportional gain\r\n";
  Serial << "i<number> - set integral gain\r\n";
  Serial << "d<number> - set derivative gain\r\n";
  Serial << "g - go\r\n";
  Serial << "s - stop (leave set values)\r\n\n";
}


void PID() {
  int     inByte;
  bool    controllerOn = false, starting;
  int     sp;
  float   target;
  float  p, i, d;  // gain parameters
  float   integral, deriv;
  UL   loopTime, lastTime;
  String  pS ="0.00", iS ="0.00", dS ="0.00";
    
  while ( true ) {
    // process command if present 
    inByte = Serial.read();
    switch( inByte ) {
    case -1 : // no character
      break;
    case 'q' :
      controllerOn = false;
      myMotor->run(RELEASE); // make sure motor is stopped (coasting)
      Serial << "Stopping controller, resetting parameters\r\n"; 
      return;
      break;
    case 'h' :
    case '?' :
      Help();
      Serial << "\r\nPID parameters\r\n";
      Serial << "Target angle = " << target << " (p, i, d) = (" << pS << ", " << iS << ", " << dS << ")" << endl;
      //Serial << p/(p-10.) << endl; //test
      break;
    case 'a' : // set desired angle
      target = Serial.parseFloat();
      target = constrain( target, -20, 140);
      Serial << "new target = " << target << endl;
      break;
    case 'p' :
      // The Stream library's float only displays 2 decimal places
      // this lets us see more
      pS = Serial.readString(); 
      p = pS.toFloat();
      Serial << "new p = " << pS << endl;
      break;
    case 'i' : 
      iS = Serial.readString(); 
      i = iS.toFloat();
      Serial << "new i = " << iS << endl;
      break;
    case 'd' : 
      dS = Serial.readString(); 
      d = dS.toFloat();
      Serial << "new d = " << dS << endl;
      break;
    case 'g' : // go - start controller
      controllerOn = true;
      starting = true;
      startMSec = millis();
      ID.Clear();
      break;
    case 's' : // stop controller
      controllerOn = false;
      break;
    default: // bad input, just eat the character
      break;
    } 

    if ( controllerOn ) {
      float errorAng, commandVal;
      if (starting) {
        lastTime = micros();
        integral = deriv = 0;
      }
      else {
        loopTime = micros() - lastTime;
        lastTime += loopTime; // so lastTime = micros()
      }

      errorAng = ReadAngle() - target;
      // pid calculation
      commandVal = p * errorAng + i * integral + d * deriv;

       
      // display periodically
      if ( Timer( 500 ) ) {
        Serial << "at " << (millis()-startMSec)/1000. <<" sec. Target angle = " << target << " measured angle = " << ReadAngle() << endl;
        Serial << "(p, i, d) = (" << pS << ", " << iS << ", " << dS << ")" << endl;
        Serial << "Error angle = " << errorAng <<  " integral = " << integral << " derivative = " << deriv << " loop time = " << loopTime << endl; 
        Serial << "commandVal = " << pS << " * "<< errorAng << " + " << iS  << " * "<< integral << " + " << dS << " * "<< deriv <<endl;
        Serial << "commandVal = " << p * errorAng << " + " << i * integral << " + " << d * deriv <<endl;
        Serial << "    = " << commandVal << endl << endl;
      }
      // get new integral and derivative
      ID.AddTerm( errorAng, float(loopTime));
//      integral = getIntegral(errorAng, (float)loopTime, starting );
//      deriv =  getDeriv(errorAng, (float)loopTime, starting );  // note flotAng
      integral = ID.getIntegral();
      deriv = ID.getDeriv();
      starting = false;
      
      // motor speed
      sp = constrain( int(commandVal), -255, 255 );
      myMotor->setSpeed( abs(sp) );
      if ( sp <= 0 ) {
        myMotor->run(FORWARD);
      }
      else {
        myMotor->run(BACKWARD);    
      }
    }
    else {
      myMotor->run(RELEASE); // make sure motor is stopped (coasting);      
    }
    //pause until current time is > wait usec since last time
    waitFor( 10000 );
  } // while true
}

// read the angle and return it.  
// potCount and angle read are globals,for convenience in DisplayAngle
// returns value for use in other pieces of code
float ReadAngle() {
  // read the analog in value:
  potCount = analogRead(analogInPin);
  // emperically, angleRead = 143.996650585439 + (-0.24148432002069) * potCount
  angleRead = 143.996650585439 + (-0.241484320020696) * potCount;
  return angleRead;
}

// if at least t milliseconds has passed since last call, reset timer and return true
// allows non-blocking timed events
boolean Timer( UL t ) {
  static UL  last = 0;
  UL ms = millis();
  
  if ( ms >= t + last ) {
    last = ms;
    return true;
  }
  else {
    return false;
  }
}

// Timer utility functions.  Concept from Matlab
void tic() {
  sTime = micros();
}

UL toc() {
  return micros() - sTime;
}

//pause until current time is > wait usec since last time
void waitFor( UL wait ) {
  static UL lastTime = 0;
  UL t;
  do {
    t = micros();
  } while( lastTime + wait > t );
  lastTime = t;  
}

