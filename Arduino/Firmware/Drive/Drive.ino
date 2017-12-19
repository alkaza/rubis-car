#include <Servo.h>
#include <RobotEQ.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <race/drive_values.h>

#define CHANNEL_1 1

ros::NodeHandle  nh;

boolean flagStop = false;

int pwm_angle_center_value = 90;  //  15% duty cycle
int pwm_angle_lowerlimit = 30;    //  10% duty cycle
int pwm_angle_upperlimit = 150;   //  20% duty cycle

int pwm_drive_center_value = 0;  //  15% duty cycle
int pwm_drive_lowerlimit = -1000;    //  10% duty cycle
int pwm_drive_upperlimit = 1000;   //  20% duty cycle

RobotEQ controller(&Serial3);
Servo steering;

std_msgs::Int32 str_msg;
ros::Publisher chatter("chatter", &str_msg);

int kill_pin = 2;
unsigned long duration = 0;

void messageDrive( const race::drive_values& pwm ) {
  //  Serial.print("Pwm drive : ");
  //  Serial.println(pwm.pwm_drive);
  //  Serial.print("Pwm angle : ");
  //  Serial.println(pwm.pwm_angle);

  if (controller.isConnected()) {
    if (flagStop == false) {
      str_msg.data = pwm.pwm_drive;
      chatter.publish( &str_msg );

      if (pwm.pwm_drive < pwm_drive_lowerlimit) {
        controller.commandMotorPower(CHANNEL_1, pwm_drive_lowerlimit);    //  Safety lower limit
      }
      else if (pwm.pwm_drive > pwm_drive_upperlimit) {
        controller.commandMotorPower(CHANNEL_1, pwm_drive_upperlimit);	//  Safety upper limit
      }
      else {
        controller.commandMotorPower(CHANNEL_1, pwm.pwm_drive);	//  Incoming data
      }


      if (pwm.pwm_angle < pwm_angle_lowerlimit) {
        steering.write(pwm_angle_lowerlimit);    //  Safety lower limit
      }
      else if (pwm.pwm_angle > pwm_angle_upperlimit) {
        steering.write(pwm_angle_upperlimit);    //  Safety upper limit
      }
      else {
        steering.write(pwm.pwm_angle);	//  Incoming data
      }

    }
    else {
      controller.commandMotorPower(CHANNEL_1, pwm_drive_center_value);
      steering.write(pwm_angle_center_value);
    }
  }
}

void messageEmergencyStop( const std_msgs::Bool& flag ) {
  flagStop = flag.data;
  if ((flagStop == true) && controller.isConnected()) {
    controller.commandMotorPower(CHANNEL_1, pwm_drive_center_value);
    steering.write(pwm_angle_center_value);
  }
}


ros::Subscriber<race::drive_values> sub_drive("drive_pwm", &messageDrive );
ros::Subscriber<std_msgs::Bool> sub_stop("eStop", &messageEmergencyStop );


void setup() {
  Serial3.begin(115200);
  steering.attach(8);
  
  if (controller.isConnected()) {
    controller.commandMotorPower(CHANNEL_1, pwm_drive_center_value);
    steering.write(pwm_angle_center_value);
  }

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  pinMode(2, INPUT);
  //  digitalWrite(2,LOW);

  nh.initNode();
  nh.subscribe(sub_drive);
  nh.subscribe(sub_stop);

  nh.advertise(chatter);
}

void loop() {
  nh.spinOnce();
  duration = pulseIn(kill_pin, HIGH, 30000);

  while ((duration > 1900) && controller.isConnected()) {
    duration = pulseIn(kill_pin, HIGH, 30000);
    controller.commandMotorPower(CHANNEL_1, pwm_drive_center_value);
    steering.write(pwm_angle_center_value);
  }

  /*
    if(Serial.available() && (controller.isConnected())
    {
    int spd = Serial.read();
    if(spd>127) {
      spd = spd-128;
      spd = map(spd,0,100,410,820);
      controller.commandMotorPower(CHANNEL_1, spd);
    }
    else {
      //angle servo
      spd = map(spd,0,100,410,820);
      steering.write(spd);
    }

    }
  */
}

