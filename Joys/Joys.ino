//TODO
// capacitor on motor cable




// CONFIG
#define MAX_SWITCHES 13 // the number of switches 13+13+5+4+1+1
#define VIRTUAL_SWITCHES 21
byte switch_pin[MAX_SWITCHES] = {100,0,1,2,3,9,6,7,8,9,10,11,12}; // digital input pins --- issue with 4
#define DEBOUNCE_TIME 5 // ms delay before the push button changes state
#define MAX_ANALOG 5 // the number of analog inputs
byte analog_pin[MAX_ANALOG] = {A1,A2,A3,A4,A5}; // analog input pins X,Y,Z,THROT
#define ENC_ZERO 0 // value to jump to after pressing encoder switch
#define VIB_ENABLED 1
#define VIB_SWITCH 3 //NOT 3
#define VIB_AXIS 1
#define VIB_OUT 5
#define VIB_HPF_COEFF 0.06 // low = more delay
#define VIB_PWM_MAX 251  // /255
#define VIB_PWM_MIN 108   // /255
#define VIB_AXIS_MAX 5  // /255
#define VIB_AXIS_MIN 4  // /255
#define VIB_VOLUME_SWITCH 7  // /255
// END CONFIG

//issue with 4 - maybe because of PWM frequency

// DECLARATIONS
#include "Joystick.h" 
Joystick_ Joystick(
  0x04,//JOYSTICK_DEFAULT_REPORT_ID, //hidReportId
  JOYSTICK_TYPE_JOYSTICK, //joystickType
  VIRTUAL_SWITCHES, //buttonCount
  0,     //hatSwitchCount
  true,  //XAxis
  true,  //YAxis
  true,  //ZAxis
  false, //RxAxis
  false, //RyAxis
  false, //RzAxis
  true,  //rudder
  true,  //throttle
  false, //accelerator
  false, //brake
  false  //steering
  );
  
byte reading;
byte prechecked = 0;
byte switch_state[MAX_SWITCHES];
byte switch_state_old[MAX_SWITCHES] = {2,2,2,2,2,2,0,0,0,0,0,0,0};
int analog_value[MAX_ANALOG];
unsigned long debounce_time[MAX_SWITCHES]; // +1 for CLK of rotary encoder
float pwmOutput = 0.0;

bool pwmOutputFix = true;
int pwmOutputFixed = 0;
const int motorDeadMin = 0;
const int motorDeadMax = 100;
const int motorDeadConvTo = 100;

// END DECLARATIONS

// FUNCTIONS

void button3way(byte real1, byte real2, byte fake1, byte fake2, byte fake3) {
  if (switch_state[real1-1]) {
    Joystick.pressButton(  fake1-1);
    Joystick.releaseButton(fake2-1);
    Joystick.releaseButton(fake3-1);}
  else if (switch_state[real2-1]) {
    Joystick.releaseButton(fake1-1);
    Joystick.releaseButton(fake2-1);
    Joystick.pressButton(  fake3-1);}
    else {
    Joystick.releaseButton(fake1-1);
    Joystick.pressButton(  fake2-1);
    Joystick.releaseButton(fake3-1);}

}

void button2way(byte real, byte fake1, byte fake2) {
  if (switch_state[real-1]) {
    Joystick.pressButton(  fake2-1);
    Joystick.releaseButton(fake1-1);}
  else {
    Joystick.pressButton(  fake1-1);
    Joystick.releaseButton(fake2-1);}
}

void buttonflaps(byte real1, byte real2, byte real3, byte flap_init) {
  byte flapconv[] = {3,5,4,1,2};
  byte flap = flapconv[switch_state[real3-1] * 4 + switch_state[real2-1] * 1 + switch_state[real1-1] * 2 - 1];
    //Serial.println(flap);

  for (byte i=1; i<6; i++)
    if (i==flap)
      Joystick.pressButton(flap_init - 1 + i - 1);
    else
      Joystick.releaseButton(flap_init - 1 + i - 1);
}





void update_virtual(byte but_i) {
  but_i = but_i + 1; 
  
  switch(but_i) {
      case 1:
        button2way(1,1,2);
        break;
      case 2:
        button2way(2,3,4);
        break;
      case 3:
      case 7:    
        button3way(3,7,7,6,5);
        break;
      case 4:
      case 8:    
        button3way(4,8,10,9,8);
        break;
      case 5:
      case 9:    
        button3way(5,9,13,12,11);
        break;
      case 6:
      case 10:    
        button3way(6,10,16,15,14);
        break;
      case 11:
      case 12:    
      case 13:
        buttonflaps(11,12,13,17);
        break;
        
      
      
  }
}


// END FUNCTIONS

// SETUP
void setup() {
    for (byte i=0; i<5;            i++) pinMode(switch_pin[i],INPUT_PULLUP);
    for (byte i=6; i<MAX_SWITCHES; i++) pinMode(switch_pin[i],INPUT_PULLUP); //i=5 not a switch

    //noise reduction::
    //TODO
    //https://www.pololu.com/docs/0J15/9 >> this is for electric noise not mechanical
    //http://ww1.microchip.com/downloads/en/AppNotes/00771b.pdf << capacitor for acoustic noise (sat delivery)
    //Foam Isolation
    //OIL
    //LC circuit design for DC output  https://www.ti.com/lit/an/slaa701a/slaa701a.pdf

    //PWM speed change: https://www.etechnophiles.com/change-frequency-pwm-pins-arduino-uno/
    // Also https://playground.arduino.cc/Code/PwmFrequency/
    //Code for Available PWM frequency for D5 & D6:
    //TCCR0B = TCCR0B & B11111000 | B00000001; // for PWM frequency of 62500.00 Hz
    //TCCR0B = TCCR0B & B11111000 | B00000010; // for PWM frequency of 7812.50 Hz
    //TCCR0B = TCCR0B & B11111000 | B00000011; // for PWM frequency of 976.56 Hz (The DEFAULT)
    //TCCR0B = TCCR0B & B11111000 | B00000100; // for PWM frequency of 244.14 Hz
    //TCCR0B = TCCR0B & B11111000 | B00000101; // for PWM frequency of 61.04 Hz
    pinMode(VIB_OUT, OUTPUT); 
    analogWrite(VIB_OUT, 0);

    Joystick.begin(false); 
    Joystick.setXAxisRange(0, 1023);
    Joystick.setYAxisRange(0, 1023);
    Joystick.setZAxisRange(0, 1023);
    Joystick.setRudderRange(0, 1023);//-511, 511);
    Joystick.setThrottleRange(0, 1023);

    Serial.begin(9600);

  } // END SETUP

// LOOP
void loop() {
  for (byte i=0; i<MAX_SWITCHES; i++) { // read the switches - 0 == special case  
    if (i==0)
      reading = prechecked;
    else
      reading = !digitalRead(switch_pin[i]);
     
    if (reading == switch_state[i]) debounce_time[i] = millis() + (unsigned long)DEBOUNCE_TIME;
    else if (millis() > debounce_time[i]) switch_state[i] = reading;
    if (switch_state[i] != switch_state_old[i]) { // debounced button has changed state
      // this code is executed once after change of state
      digitalWrite(13,switch_state[i]); //LED?
      update_virtual(i);
      switch_state_old[i] = switch_state[i]; // store new state such that the above gets done only once
    }
        //Serial.print(switch_state[i]);
    //Serial.print (" ");
  } //END read the switches
    //Serial.println (" ");


  for (byte i=0; i<MAX_ANALOG; i++) { // read analog inputs
    analog_value[i] = analogRead(analog_pin[i]);
  prechecked = (analogRead(A0)==0);


    //if (analog_value[i] < 256) analog_value[i] = analog_value[i] * 1.5;
    //else if (analog_value[i] < 768) analog_value[i] = 256 + analog_value[i] / 2;
    //else analog_value[i] = 640 + (analog_value[i] - 768) * 1.5;
    switch(i) {
      case 0:
        Joystick.setThrottle(analog_value[0]);
      break;
      case 1:
        Joystick.setRudder(analog_value[1]);
      break;
      case 2:
        Joystick.setZAxis(analog_value[2]);
      break;
      case 3:
        Joystick.setXAxis(analog_value[3]);
      break;
      case 4:
        Joystick.setYAxis(analog_value[4]);
      break;
      
    }
    }

    if (VIB_ENABLED) {
/*
      if (switch_state[VIB_SWITCH-1]) {
        pwmOutput = (float)map(analog_value[VIB_AXIS-1], 0, 1023, VIB_PWM_MIN , VIB_PWM_MAX) * VIB_HPF_COEFF + pwmOutput * (1-VIB_HPF_COEFF);
        analogWrite(VIB_OUT, (int)pwmOutput); // Send PWM signal to L298N Enable pin

        
        */


      if (!switch_state[VIB_SWITCH-1] || switch_state[VIB_VOLUME_SWITCH-1]){
        if (switch_state[VIB_VOLUME_SWITCH-1]) {
          pwmOutput = (float)map(analog_value[VIB_AXIS-1], 0, 1023, 
            min(analog_value[VIB_AXIS_MIN-1], analog_value[VIB_AXIS_MAX-1])/4,
            max(analog_value[VIB_AXIS_MIN-1], analog_value[VIB_AXIS_MAX-1])/4)
            * VIB_HPF_COEFF + pwmOutput * (1-VIB_HPF_COEFF); //use knobs for boundary

        }
        else {
          pwmOutput = (float)map(analog_value[VIB_AXIS-1], 0, 1023, VIB_PWM_MIN , VIB_PWM_MAX) * VIB_HPF_COEFF + pwmOutput * (1-VIB_HPF_COEFF);
              //use predefined for boundary
          

        }

        if (pwmOutputFix) {
          pwmOutputFixed = (int)pwmOutput;
          if ((pwmOutputFixed >= motorDeadMin) && (pwmOutputFixed <= motorDeadMax))
            pwmOutputFixed = motorDeadConvTo;
          analogWrite(VIB_OUT, pwmOutputFixed);
Serial.print(pwmOutput);
Serial.print("   ");
Serial.println(pwmOutputFixed);
        }
        else {
          analogWrite(VIB_OUT, (int)pwmOutput);
        }

/*
        Serial.print((float)map(analog_value[VIB_AXIS-1], 0, 1023, VIB_PWM_MIN , VIB_PWM_MAX) * VIB_HPF_COEFF);
        Serial.print("   ");
        Serial.print((float)pwmOutput * (1-VIB_HPF_COEFF));
        Serial.print("   ");
        Serial.println(pwmOutput);
*/

      }
      else {
        analogWrite(VIB_OUT, 0);
      }         
    }    
    else {
      analogWrite(VIB_OUT, 0);
    }
    
    
    //Serial.println (" ");

  Joystick.sendState();
  delay(20);
} // END LOOP
