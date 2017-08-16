/*
 * This is code for a PID demonstration based on Tim Wescott's 
 * YouTube video  https://www.youtube.com/watch?v=2elEXcv0AV8&t=128s
 * 
 */
// Streaming supports Serial << "text"; (like C++)
#include <Streaming.h>
#include <Adafruit_MotorShield.h>

// #include "ControlDemo.h"
//Apparently Arduino doesn't like to let you write include files pr handle separate modules

// constants 
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const char ESC = 0x1B;

// globals
int     potCount = 0;        // value read from the pot
float   angleRead;
unsigned long   sTime;  // for tic(), toc()
unsigned long   gTime;  // global for gTime = toc()

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

// using df(t) = (f(t)-f(t-h))/h)
// where h is the time step

float getDeriv( float x, float dt, bool startup ) {
  float oldX, ret;

  if (startup) { 
    // initilize old values to current values
    oldX = x;
  }
  ret = (x-oldX)/(dt * 1e-6);
  oldX = x;
  return ret;   
}

// this integrator just adds up rectangles.  There are much better ones
// but for short times they're not worth the effort
float getIntegral( float x, float dt, bool startup ) {
  static float integral = 0;
  const float windup = 10000;
  
  if (startup) {
    integral = 0;  
  }
  else {
    integral += x * 1e-6 * dt; // t is in usec
  }
  // integral must be limited to avoid windup
  integral = constrain( integral, -windup, windup );
  return integral;
}


void Help() {
  Serial << "PID controller demo\r\n";
  Serial << "q - stop motor, retstore defaults\r\n";
  Serial << "a<number> - set target angle\r\n";
  Serial << "p<number> - set proportional gain\r\n";
  Serial << "i<number> - set integral gain\r\n";
  Serial << "d<number> - set derivative gain\r\n";
  Serial << "g - go\r\n";
  Serial << "s - stop\r\n\n";
}



void PID() {
  int     inByte;
  bool    controllerOn = false, starting;
  int     sp;
  float   target;
  float   p, i, d;  // gain parameters
  float   integral, deriv;
  unsigned long   loopTime, lastTime;
    
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
      Serial << "Target angle = " << target << " (p, i, d) = (" << p << ", " << i << ", " << d << ")" << endl;
      break;
    case 'a' : // set desired angle
      target = Serial.parseFloat();
      target = constrain( target, -20, 140);
      Serial << "new target = " << target << endl;
      break;
    case 'p' : 
      p = Serial.parseFloat();
      Serial << "new p = " << p << endl;
      break;
    case 'i' : 
      i = Serial.parseFloat();
      Serial << "new i = " << i << endl;
      break;
    case 'd' : 
      d = Serial.parseFloat();
      Serial << "new d = " << d << endl;
      break;
    case 'g' : // go - start controller
      controllerOn = true;
      starting = true;
      break;
    case 's' : // stop controller
      controllerOn = false;
      break;
    default: // bad input, just eat the character
      break;
    } 

    if ( controllerOn ) {
      float errorAng, err;
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
      err = p * errorAng + i * integral + d * deriv;

       
      // display periodically
      if ( Timer( 500 ) ) {
        Serial << "Target angle = " << target << " measured angle = " << ReadAngle() << endl;
        Serial << "(p, i, d) = (" << p << ", " << i << ", " << d << ")" << endl;
        Serial << "Error angle = " << errorAng <<  " integral =" << integral << " derivative = " << deriv << " loop time = " << loopTime << endl; 
        Serial << "err = " << p << " * "<< errorAng << " + " << i  << " * "<< integral << " + " << d << " * "<< deriv <<endl;
        Serial << "    = " << err << endl << endl;
      }
      // get new integral and derivative
      integral = getIntegral(errorAng, (float)loopTime, starting );
      deriv =  getDeriv(errorAng, (float)loopTime, starting );  // note flotAng
      starting = false;
      
      // motor speed
      sp = constrain( (int)err, -255, 255 );
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
    delay(1);// extra pause
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
boolean Timer( unsigned long t ) {
  static unsigned long  last = 0;
  unsigned long ms = millis();
  
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

unsigned long toc() {
  return micros() - sTime;
}


