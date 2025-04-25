#include <IBT.h>

IBT r_wheel(2, 3);
IBT rb_wheel(4, 5);
IBT l_wheel(6, 7);
IBT lb_wheel(8, 9);

const double b = 15;
const double r = 16.5;

const double  max_wheel_speed = 7.2;

double pwm_l = 0;
double pwm_r = 0;

char rc;
boolean exitloop;
String recvSerial;

void cmd_vel_stuffs(double linear_vel, double angular_vel){

  double r_wheel_vel = constrain(((linear_vel + (angular_vel * (b/2)))/r), -max_wheel_speed, max_wheel_speed);
  double l_wheel_vel = constrain(((linear_vel - (angular_vel * (b/2)))/r), -max_wheel_speed, max_wheel_speed);

  pwm_r = map(r_wheel_vel,-max_wheel_speed, max_wheel_speed, -200, 200);
  pwm_l = map(l_wheel_vel,-max_wheel_speed, max_wheel_speed, -200, 200);

  r_wheel.setRawSpeed(pwm_r);
  l_wheel.setRawSpeed(-pwm_l);
  rb_wheel.setRawSpeed(pwm_r);
  lb_wheel.setRawSpeed(-pwm_l);
  Serial.println("r_wheel");
  Serial.println(pwm_r);
  Serial.println("l_wheel");
  Serial.println(pwm_l);
  Serial.println("rb_wheel");
  Serial.println(pwm_r);
  Serial.println("lb_wheel");
  Serial.println(pwm_l);
  Serial.println("=========");

}

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Check if data is available to read from the serial port
  if (Serial.available() > 0) {
    rc = Serial.read();
//    Serial.println("rec");
    if (rc == '<'){
      recvSerial = Serial.readStringUntil('>');  // Read until '>'
//      Serial.println(recvSerial);
      delay(10);
//      Serial.println("test");
      if (recvSerial.length() > 0) {
          int comma_index = recvSerial.indexOf(',');
          if (comma_index != -1) { // Check if comma character was found
              // Extract the first part (before the comma)
              String firstPart = recvSerial.substring(0, comma_index);
              // Extract the second part (after the comma)
              String secondPart = recvSerial.substring(comma_index + 1);
              // Convert the extracted substrings to float
              double lin_vel = firstPart.toDouble();
              double ang_vel = secondPart.toDouble();
//              Serial.println("Linear Velocity: " + String(lin_vel));
//              Serial.println("Angular Velocity: " + String(ang_vel));
              cmd_vel_stuffs(lin_vel, ang_vel);
              delay(150);
          } else {
              Serial.println("Invalid input format");
          }
    }
   }
  }
  Serial.flush();
}
