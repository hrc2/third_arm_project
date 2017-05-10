#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <ArduinoHardware.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

// Define the ros node
ros::NodeHandle nh;

// calibration status
std_msgs::UInt8 cal_msg;
ros::Publisher pub_cal("cal_status", &cal_msg);

//imu message setup
sensor_msgs::Imu imu_msg;
ros::Publisher pub_imu("imu", &imu_msg);

//button state
std_msgs::Bool pushed_msg;
ros::Publisher pub_button("grip", &pushed_msg);
const int button_pin = 7;
const int led_pin = 13;

// button debounce logic
bool last_reading;
long last_debounce_time=0;
long debounce_delay=50;
bool published = true;

// Set the delay between fresh samples
#define BNO055_SAMPLERATE_DELAY_MS (20)

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() {
  nh.initNode();
  nh.advertise(pub_cal);
  nh.advertise(pub_imu); // start the imu publisher
  nh.advertise(pub_button);

  //initialize an LED output pin
  //and an input pin for the push button
  pinMode(led_pin, OUTPUT);
  pinMode(button_pin, INPUT);

  //Enable the pullup resistor on the button
  digitalWrite(button_pin, HIGH);
  last_reading = ! digitalRead(button_pin);

  bno.begin(bno.OPERATION_MODE_IMUPLUS); // start the sensor

  bno.setExtCrystalUse(true); // set the imu to use the Arduino's timing crystal
}

long publisher_timer;
bool is_calibrated;
void loop() {

  // LED logic
  bool reading = ! digitalRead(button_pin);
  if (last_reading!=reading){
    last_debounce_time = millis();
    published = false;
  }

  if (!published && (millis() - last_debounce_time) > debounce_delay) {
    digitalWrite(led_pin, reading);
    pushed_msg.data = reading;
    pub_button.publish(&pushed_msg);
    published = true;
  }
  last_reading = reading;

  // IMU stuffs
  if (millis() > publisher_timer) {
    
    // Publish calibration satus of the magnetometer
    uint8_t system, gyro, accel, mag;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    cal_msg.data = mag;
    pub_cal.publish(&cal_msg);

      imu_msg.header.stamp = nh.now();
      imu_msg.header.frame_id = "imu";
    
      // Get orientation quaternion from IMU
      imu::Quaternion quat = bno.getQuat();
      imu_msg.orientation.x = quat.x();
      imu_msg.orientation.y = quat.y();
      imu_msg.orientation.z = quat.z();
      imu_msg.orientation.w = quat.w();
  
      // Get angular velocity from IMU
      imu::Vector<3> ang_vel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      imu_msg.angular_velocity.x = ang_vel.x();
      imu_msg.angular_velocity.y = ang_vel.y();
      imu_msg.angular_velocity.z = ang_vel.z();
  
      // Get linear acceleration from IMU
      imu::Vector<3> lin_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      imu_msg.linear_acceleration.x = lin_accel.x();
      imu_msg.linear_acceleration.y = lin_accel.y();
      imu_msg.linear_acceleration.z = lin_accel.z();

      // Publish the new imu message
      pub_imu.publish(&imu_msg);

    publisher_timer = millis() + BNO055_SAMPLERATE_DELAY_MS;
  }
  nh.spinOnce();
}
