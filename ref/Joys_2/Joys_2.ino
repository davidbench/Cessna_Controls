// DECLARATIONS
#include "Joystick.h" 

#define TRIM 200

Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID, //hidReportId
  JOYSTICK_TYPE_JOYSTICK, //joystickType
  0, //buttonCount
  0,     //hatSwitchCount
  true,  //XAxis
  true,  //YAxis
  false,  //ZAxis
  false, //RxAxis
  false, //RyAxis
  false, //RzAxis
  false,  //rudder
  false,  //throttle
  false, //accelerator
  false, //brake
  false  //steering
  );
  
int analog_value_x;
int analog_value_y;

// SETUP
void setup() {
    Joystick.begin(false); 
    Joystick.setXAxisRange(0, 1023);
    Joystick.setYAxisRange(0, 1023);

    Serial.begin(9600);

  } // END SETUP

// LOOP
void loop() {
  analog_value_x = 1023 - analogRead(A0);
  analog_value_y = analogRead(A1);

  Joystick.setXAxis(
    map(analog_value_x, 0+TRIM , 1023-TRIM, 0, 1023));
  Joystick.setYAxis(
    map(analog_value_y, 0+TRIM , 1023-TRIM, 0, 1023));
      
        /*
        Serial.print((float)map(analog_value[VIB_AXIS-1], 0, 1023, VIB_PWM_MIN , VIB_PWM_MAX) * VIB_HPF_COEFF);
        Serial.print("   ");
        Serial.print((float)pwmOutput * (1-VIB_HPF_COEFF));
        Serial.print("   ");
        Serial.println(pwmOutput);
        */
      
  Joystick.sendState();
  delay(20);
} // END LOOP
