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
      
      // get new integral and derivative
      integral = getIntegral(errorAng, (float)loopTime, starting );
      deriv =  getDeriv(errorAng, (float)loopTime, starting );  // note flotAng
      starting = false;

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
  // emperically, angleRead = -89.4 + 0.267 * potCount
  angleRead = -89.4 + 0.267 * potCount;
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

/********************************************************************************/
// code for 'old' (ControlDemo) tests
// states
const char stIdle = 'i';
const char stAngle = 'a'; // angle display
const char stManual = 'm';
const char stBang = 'b';
const char stAngTable = 't';
const char stExit = 'x';

// globals
char    state;
bool    gotNewState;

//function declarations to avoid errors, but why?
/*
void HelpIdle();
void ProcessInput();
void DisplayAngle();
void ManualMotor();
void HelpBang();
void BangBang();
*/

void ControlDemoEntry() {
  myMotor->run(RELEASE); // make sure motor is stopped (coasting)
  state =  stIdle;
  gotNewState = true;
  HelpIdle();
  CDLoop();
}

void CDLoop() {
  bool running = true;
  while ( running ) {
    ProcessInput();
    switch( state ) {
    case stIdle :
      if (gotNewState) {
        HelpIdle();
      }
      gotNewState = false;
      break;
    case stAngle :
      if (gotNewState) {
        Serial << F(\
        "Angle display mode\r\n"\
        "q to return to Idle\r\n");
        gotNewState = false;
      }
      DisplayAngle();
      break;
    case stManual :
      if (gotNewState ) {
        Serial << F(\
        "Manual motor control mode \r\n"\
        "Enter a number -255 to 255 to set motor speed\r\n"\
        "q to stop motor return to Idle state\r\n");
        gotNewState = false;
      }
      ManualMotor();
      break;
    case stBang :
      if ( gotNewState ){
        HelpBang();
        gotNewState = false;
      }
      BangBang();
      break;
    case stExit :
      running = false;
      myMotor->run(RELEASE); // make sure motor is stopped (coasting)
      break;
    case stAngTable :
      AngleTable();
      break;
    default :
      Serial << F("Unknown state 0x") << _HEX(state) << endl;
      Serial << F("returning to Idle state\r\n");
      Quit();
      break;
    }
  }  
} // CDLoop()

void AngleTable() {
  int i, n, Angle[10], Count[10];
  int byteIn;
  
  Serial << F(\
  "Set an angle on the arm and enter it.\r\n"\
  "The angle and pot count will be recorded.\r\n"\
  "When you press 'q' or have entered  10 angles\r\n"\
  "a table will be printed.\r\n\n");
  for ( n = 0; n < 10; n++ ) {
    while (Serial.available() <= 0 ) { /* wait */}
    byteIn = Serial.peek();
    if ( byteIn == 'q' ) break;
    Angle[n] = Serial.parseInt();
    Count[n] = analogRead(analogInPin);
  }
  Serial << F("idx, angle, count\r\n");
  for ( i = 0; i < n; i++ ){
    Serial << i+1 << ", " << Angle[i] << ", " << Count[i] << endl; 
  }
  Serial << endl;
}

void BangBang() {
  int inByte;
  bool controllerOn = false;
  int sp = 250;
  float target;
  float deadBand = 10;
  
  while ( stBang == state ) {
    inByte = Serial.read();
    switch( inByte ) {
    case -1 : // no character
      break;
    case 'q' :
      controllerOn = false;
      Quit();
      return;
      break;
    case 'h' :
    case '?' :
      HelpBang();
      Serial << F("\r\nBang Bang parameters\r\n");
      Serial << "Target angle = " << target << " deadband = " << deadBand << " motor speed = " << sp << endl;
      break;
    case 'a' : // set desired angle
      target = Serial.parseFloat();
      target = constrain( target, -20, 140);
      Serial << "new target = " << target << endl;
      break;
    case 'd' : // set (half) deadband default 10
      deadBand = Serial.parseFloat();
      deadBand = constrain( deadBand, 0, 45);
      Serial << "new deadband = " << deadBand << endl;
      break;
    case 'g' : // go - start controller
      controllerOn = true;
      break;
    case 's' : // stop controller
      controllerOn = false;
      break;
    case 'v' : // set motor speed (default = 150)
      sp = Serial.parseInt();
      sp = constrain( sp, 0, 255);
      Serial << "new speed = " << sp << endl;
      break;
      break;
    default: // bad input, just eat the character
      break;
    } 

    if ( controllerOn ) {
      float errorAng = ReadAngle() - target;

      if ( Timer( 1000 ) ) {
        Serial << "Target angle = " << target << " deadband = " << deadBand << " motor speed = " << sp << endl;
        Serial <<  "Error = " << errorAng << endl << endl;
      }
      myMotor->setSpeed( sp );
      if( abs( errorAng ) <= deadBand ) {
        myMotor->run(RELEASE); // make sure motor is stopped (coasting);      
      }
      else if ( errorAng < 0 ) {
        myMotor->run(FORWARD);
      }
      else {
        myMotor->run(BACKWARD);    
      }
   }
    else {
      myMotor->run(RELEASE); // make sure motor is stopped (coasting);      
    }
  } // while in stBang 
}

void DisplayAngle() {
  int inByte;
  
  angleRead = ReadAngle();  
  // map it to the range of the analog out:
  //outputValue = map(sensorValue, 0, 1023, 0, 255);

  // print the results to the serial monitor:
  Serial << "potCount  = " << potCount;
  Serial << " angle = " << angleRead << "\r\n";
  delay(1000); 
  inByte = Serial.read();
  if ( 'q' == inByte ) Quit();
  if ( '?' == inByte || 'h' == inByte ) Serial << F("in Angle state 'q' to return to Idle state\r\n");
}

void HelpBang() {
  Serial << F(\
  "Bang bang controller mode\r\n"\
  "q - stop motor return to Idle state\r\n"\
  "a<number> - set target angle\r\n"\
  "d<number> - set (1/2) deadband\r\n"\
  "g - start\r\n"\
  "s - stop but stay in bang-bang mode\r\n"\
  "v<number> - set motor speed, 0-255 default 150\r\n\n");  
}

void HelpIdle() {
  Serial << F( \
  "Control Demo code\r\n"\
  "in idle mode\r\n"\
  "Use lower case commands\r\n"\
  "depending on your terminal you may need to hit enter\r\n"\
  "q will stop the motor and return to the idle mode\r\n"\
  "h or ? - print this help\r\n"\
  "a - display angle\r\n"\
  "m - manual motor control\r\n"\
  "b - bang-bang controller\r\n"\
  "t - set up angle table\r\n"\
  "x - eXit old code, return to PID\r\n\n");
}

void HelpManual() {
  Serial << F(\
  "Manual motor mode\r\n"\
  "q will stop the motor and return to the idle mode\r\n"\
  "h or ? - print this help\r\n"\
  "enter an integer between -255 and 255 to set motor speed\r\n\n");
}
void ManualMotor() {
  int inByte, inNum;
  static int sp = 0, oldSp = 0;

  while ( true ) {
    inByte = Serial.peek();
    switch( inByte ) {
    case 'q' :
      Quit();
      Serial.read(); // remove character
      return;
      break;
    case 'h' :
    case '?' :
      HelpManual();
      Serial.read(); // remove character
      break;
    case -1 : // no input
      Serial << "speed " << sp << "  angle " << ReadAngle() << endl;
      delay(500);
      break;
    default: // assume input is a number
      inNum = Serial.parseInt();
      oldSp = sp;
      //limit the speed
      sp = constrain( inNum, -255, 255 );
      Serial   << "Changing speed from " << oldSp << " to " << sp << endl;
    }
//    Serial << "inByte = " << inByte << endl;
//    Serial << "inNum = " << inNum << endl;
    myMotor->setSpeed( abs(sp) );
    if ( sp >= 0 ) {
      myMotor->run(FORWARD);
    }
    else {
      myMotor->run(BACKWARD);    
    }
  }
}


void ProcessInput() {
  int byteIn;
  
   if (Serial.available() > 0 ) {
    byteIn = Serial.peek();
    switch( byteIn ) {
    case 'q' :
      Quit();
      break;
    case 'h' :
    case '?' :
      HelpIdle();
      break;
    case 'a' :
      myMotor->run(RELEASE); // make sure motor is stopped (coasting) just in case
      state = stAngle;
      gotNewState = true;
      break;
    case 'm' :
      state = stManual;
      gotNewState = true;
      break;
    case 'b' :
      state = stBang;
      gotNewState = true;
      break;
    case 't' :
      state = stAngTable;
      gotNewState = true;
      break;
    case 'x' :
      state = stExit;
      gotNewState = true;
    default : 
      return; // avoid reading unknowm byte 
      break;
    }
    Serial.read();
  }
}

void Quit() {
  myMotor->run(RELEASE); // make sure motor is stopped (coasting)
  state = stIdle;
  gotNewState = true;
}


