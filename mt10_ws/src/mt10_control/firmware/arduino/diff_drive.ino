  #include <IBT.h>

IBT l_wheel(2, 3);
IBT lb_wheel(4, 5);
IBT r_wheel(6, 7);
IBT rb_wheel(8, 9);

const double b = 15;
const double r = 16.5;
const double  max_wheel_speed = 7.2;

double pwm_l = 0;
double pwm_r = 0;

void cmd_vel_stuffs(double linear_vel, double angular_vel){

  double r_wheel_vel = constrain(((linear_vel + (angular_vel * (b/2)))/r), -max_wheel_speed, max_wheel_speed);
  double l_wheel_vel = constrain(((linear_vel - (angular_vel * (b/2)))/r), -max_wheel_speed, max_wheel_speed);

  pwm_r = map(r_wheel_vel,-max_wheel_speed, max_wheel_speed, -200, 200);
  pwm_l = map(l_wheel_vel,-max_wheel_speed, max_wheel_speed, -200, 200);

  r_wheel.setRawSpeed(-pwm_r);
  l_wheel.setRawSpeed(pwm_l);
  rb_wheel.setRawSpeed(-pwm_r);
  lb_wheel.setRawSpeed(pwm_l);

}

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Check if data is available to read from the serial port
  if (Serial.available() > 0) {
    
   String incomingByte = Serial.readString();

    int space_index = incomingByte.indexOf(' ');

    if (space_index != -1) { // Check if space character was found
      // Extract the first part (before the space)
      String firstPart = incomingByte.substring(0, space_index);

      // Extract the second part (after the space)
      String secondPart = incomingByte.substring(space_index + 1);

      // Convert the extracted substrings to float
      double lin_vel = firstPart.toDouble();
      double ang_vel = secondPart.toDouble();

    cmd_vel_stuffs(lin_vel, ang_vel);
    delay(150);
    
    }
  }
}
