/*
 * This is code for a PID demonstration based on Tim Wescott's 
 * YouTube video  https://www.youtube.com/watch?v=2elEXcv0AV8&t=128s
 * 
 */
// Streaming supports Serial << "text"; (like C++)
#include <Streaming.h>
#include <Adafruit_MotorShield.h>

// #include "ControlDemo.h"
//Apparently Arduino doesn't like to let you write include files to handle separate modules

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

// using df(t) = (f(t)-f(t-h))/h)
// where h is the time step

float getDeriv( float x, float dt, bool startup ) {
  static float oldX; 
  float ret;

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
  const float windup = 100000.;
  
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
  Serial << F(\
  "PID controller demo\r\n"\
  "h or ? - print this help\r\n"\
  "q - stop motor, restore defaults\r\n"\
  "a<number> - set target angle\r\n"\
  "p<number> - set proportional gain\r\n"\
  "i<number> - set integral gain\r\n"\
  "d<number> - set derivative gain\r\n"\
  "l<number> - set loop time (usec)\r\n"\
  "c - clear calculated values (int, deriv)\r\n"\
  "g - go\r\n"\
  "s - stop\r\n"\
  "o - old controller commands\r\n"\ 
  " # - comment (just display this line in output)\r\n\n");
  // 'o' handles commands formerly in separate Control Demo program
}



void PID() {
  int     inByte;
  bool    controllerOn = false, starting;
  int     sp;
  float   target;
  float  p, i, d;  // gain parameters
  float   integral, deriv;
  UL   loopTime, lastTime, loopDelay = 50000;
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
      Serial << F("Stopping controller, resetting parameters\r\n"); 
      return;
      break;
    case 'h' :
    case '?' :
      Help();
      Serial << F("\r\nPID parameters\r\n");
      Serial << F("Target angle = ") << target << " (p, i, d) = (" << pS << ", " << iS << ", " << dS << ")"; 
      Serial << F(" Loop is ") << loopDelay << " usec\r\n";
      //Serial << p/(p-10.) << endl; //test
      break;
    case 'a' : // set desired angle
      target = Serial.parseFloat();
      target = constrain( target, -20, 140);
      Serial << F("new target = ") << target << endl;
      break;
    case 'p' :
      // The Stream library's float only displays 2 decimal places
      // this lets us see more
      //pS = Serial.readString(); 
      //p = pS.toFloat();
      p = ReadSerialFloat( &pS ); 
      Serial << F("new p = ") << pS << endl;
      break;
    case 'i' : 
      i = ReadSerialFloat( &iS );
      Serial << F("new i = ") << iS << endl;
      break;
    case 'd' : 
      d = ReadSerialFloat( &dS );
      Serial << F("new d = ") << dS << endl;
      break;
    case 'l' :
      loopDelay = Serial.parseInt(); 
      Serial << F("new loop_time = ") << loopDelay << endl;
      break;
    case 'c' :
      starting = true;
      break;
    case 'g' : // go - start controller
      controllerOn = true;
      starting = true;
      startMSec = millis();
      break;
    case 's' : // stop controller
      controllerOn = false;
      break;
    case 'o' : // old (Control Demo) commands
      ControlDemoEntry();
    case '#' : // comment
     Serial << '#' << Serial.readString()<< endl;
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
        Serial << "at " << (millis()-startMSec)/1000. << F(" sec. Target angle = ") << target << F(" measured angle = ") << ReadAngle() << endl;
        Serial << F("(p, i, d) = (") << pS << ", " << iS << ", " << dS << ")" << endl;
        Serial << F("Error angle = ") << errorAng <<  " integral = " << integral << " derivative = " << deriv << " loop time = " << loopTime << endl; 
        Serial << F("commandVal = ") << pS << " * "<< errorAng << " + " << iS  << " * "<< integral << " + " << dS << " * "<< deriv <<endl;
        Serial << F("commandVal = ") << p * errorAng << " + " << i * integral << " + " << d * deriv <<endl;
        Serial << "    = " << commandVal << endl << endl;
      }
      // get new integral and derivative
      integral = getIntegral(errorAng, (float)loopTime, starting );
      deriv =  getDeriv(errorAng, (float)loopTime, starting );  // note flotAng
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
    waitFor( loopDelay );
  } // while true
} // PID()




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

// Auxiliary function to read floats with more precision than Serial.parseFloat
// The string passed in (as a ointer) gets the value read from Serial
float ReadSerialFloat(String *s) {
  *s = Serial.readString();
  return (*s).toFloat();
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

// code for 'old' (ControlDemo) tests
// states
const char stIdle = 'i';
const char stDisplay = 'a'; // angle display
const char stManual = 'm';
const char stBang = 'b';

// globals
char    state, oldState;

void ControlDemoEntry() {
  myMotor->run(RELEASE); // make sure motor is stopped (coasting)
  state = oldState = stIdle;
  HelpIdle();
  CDLoop();
}

void CDLoop() {
  bool running = true;
  while ( running ) {
    switch( state ) {
    case stIdle :
      HelpIdle();
      break;
    case stDisplay :
      Serial << F(\
      "Angle display mode\r\n"\
      "q to return to Idle\r\n");
      break;
    case stManual :
      Serial << "Manual motor control mode \r\n";
      Serial << "Enter a number -255 to 255 to set motor speed\r\n";
      Serial << "q to stop motor return to Idle state\r\n";
      break;
    case stBang :
      HelpBang();
      break;
    case stExit :
      running = false;
      myMotor->run(RELEASE); // make sure motor is stopped (coasting)
      break;
    default :
      Serial << "Unknown state 0x" << _HEX(state) << endl;
      Serial << "returning to Idle state\r\n";
      Quit();
      break;
    }
  }  
}

void HelpBang() {
  Serial << "Bang bang controller mode\r\n";
  Serial << "q - stop motor return to Idle state\r\n";
  Serial << "a<number> - set target angle\r\n";
  Serial << "d<number> - set (1/2) deadband\r\n";
  Serial << "g - start\r\n";
  Serial << "s - stop but stay in bang-bang mode\r\n";
  Serial << "v<number> - set motor speed, 0-255 default 150\r\n\n";  
}

void HelpIdle() {
  Serial << F( \
  "Control Demo code\r\n"\
  "idle mode\r\n"\
  "Use lower case commands\r\n"\
  "depending on your terminal you may need to hit enter\r\n"\
  "q will stop the motor and return to the main loop\r\n"\
  "h or ? - print this help\r\n"\
  "a - display angle\r\n"\
  "m - manual motor control\r\n"\
  "b - bang-bang controller\r\n\n"\
  "x - eXit old code, return to PID\r\n");
}

void ManualMotor() {
  int inByte, inNum;
  static int sp = 0, oldSp = 0;

  inByte = Serial.peek();
  switch( inByte ) {
  case 'q' :
    Quit();
    Serial.read(); // remove character
    return;
    break;
  case 'h' :
  case '?' :
   CDHelp();
    Serial.read(); // remove character
    return;
    break;
  case -1 : // no input
    Serial << "speed " << sp << "  angle " << ReadAngle() << endl;
    delay(500);
    return;
    break;
  default: // assume input is a number
    inNum = Serial.parseInt();
    break;
  }
  Serial << "inByte = " << inByte << endl;
  Serial << "inNum = " << inNum << endl;
  oldSp = sp;
  //limit the speed
  sp = constrain( inNum, -255, 255 );
  Serial   << "Changing speed from " << oldSp << " to " << sp << endl;
  myMotor->setSpeed( abs(sp) );
  if ( sp >= 0 ) {
      myMotor->run(FORWARD);
  }
  else {
      myMotor->run(BACKWARD);    
  }
}


void ProcessInput() {
  int byteIn;
  
  if (Serial.available() > 0 ) {
    byteIn = Serial.read();
    switch( byteIn ) {
    case 'q' :
      Quit();
      break;
    case 'h' :
    case '?' :
      CDHelp();
      break;
    case 'a' :
      myMotor->run(RELEASE); // make sure motor is stopped (coasting) just in case
      oldState = state;
      state = stDisplay;
      break;
    case 'm' :
      oldState = state;
      state = stManual;
      break;
    case 'b' :
      oldState = state;
      state = stBang;
      break;
    case 'x' :
      oldState = state;
      state = stExit;
    default :  
      break;
    }
  }
}

void Quit() {
  myMotor->run(RELEASE); // make sure motor is stopped (coasting)
  oldState = state;
  state = stIdle;
}

// if at least t milliseconds has passed since last call, reset timer and return true
// allows non-blocking timed events


