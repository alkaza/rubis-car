#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <RobotEQ.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

#define CHANNEL_1 1

ros::NodeHandle  nh;

// General bounds for the steering servo
const int center_steering = 90;
const int min_steering = 30;
const int max_steering = 150;

// RobotEQ motor controller
const int center_throttle = 0;
const int min_throttle = -1000;
const int max_throttle = 1000;

// Create servo object to control a servo
Servo steering;
// Configure Motor Controllers
RobotEQ controller(&Serial3);

std_msgs::Int32 str_msg;
ros::Publisher chatter("chatter", &str_msg);

// Arduino 'map' funtion for floating point
double fmap (double toMap, double in_min, double in_max, double out_min, double out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void twistMsgDrive ( const geometry_msgs::Twist& twistMsg )
{
  // Check to make sure steering angle is within car range
  int steering_angle = fmap(twistMsg.angular.z, 0.0, 1.0, min_steering, max_steering);

  // The following could be useful for debugging:
  str_msg.data = steering_angle;
  chatter.publish(&str_msg);    

  if(steering_angle < min_steering)
  {
    steering_angle = min_steering;    //  Safety lower limit        
  }
  else if(steering_angle > max_steering)
  {
    steering_angle = max_steering;    //  Safety upper limit
  }
  steering.write(steering_angle);     //  Incoming data


  // Forward is between 0.5 and 1.0
  int motor_command;
  if (twistMsg.linear.x >= 0.5) {
    motor_command = (int)fmap(twistMsg.linear.x, 0.5, 1.0, center_throttle, max_throttle) ;
  } 
  else {
    motor_command = (int)fmap(twistMsg.linear.x, 0.0, 1.0, min_throttle, max_throttle) ;
  }

  // Check to make sure throttle command is within bounds
  if(motor_command < min_throttle)
  {
    motor_command = min_throttle;	//  Safety lower limit  
  }
  else if(motor_command > max_throttle)
  {
    motor_command = max_throttle;	//  Safety upper limit  
  }

  // The following could be useful for debugging
  str_msg.data = motor_command;
  chatter.publish(&str_msg);

  controller.commandMotorPower(CHANNEL_1, motor_command);	//  Incoming data                  
}


ros::Subscriber<geometry_msgs::Twist> drive_sub("cmd_vel", &twistMsgDrive);


void setup() {
  Serial.begin(57600);
  Serial3.begin(115200);
  
  nh.initNode();
  // This can be useful for debugging purposes
  nh.advertise(chatter);
  // Subscribe to the steering and throttle messages
  nh.subscribe(drive_sub);
  
  // Attache the steering servo to pin 9
  steering.attach(8);	
  steering.write(center_steering) ;
  
  controller.commandMotorPower(CHANNEL_1, center_throttle);
    
  delay(1000);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
