#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>
#include <rmw_microros/rmw_microros.h>

#include <PID_v1.h>
#include "motor_control.h"
#include "robot_config.h"
#include "MPU9250.h"

MPU9250 mpu;

// ros2 vars definition
rcl_subscription_t cmd_sub;
rcl_publisher_t odom_pub;
rcl_publisher_t imu_pub;

rcl_timer_t odom_timer; 
rcl_timer_t imu_timer;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// ros2 msgs definitions
nav_msgs__msg__Odometry odom_msg_pub;
geometry_msgs__msg__TransformStamped odom_tf;
geometry_msgs__msg__Quaternion odom_quat;

sensor_msgs__msg__Imu imu_msg_pub;

geometry_msgs__msg__Twist cmd_msg_sub;

int lef_enc_count = 0;
int right_enc_count = 0;

double speed_current_left, speed_current_right;
double pwm_right, pwm_left;
double setpoint_left=0, setpoint_right=0;
double Kp=250, Ki=250, Kd=30;
// double Kp=80, Ki=10, Kd=3;
PID leftPID(&speed_current_left, &pwm_left, &setpoint_left, Kp, Ki, Kd, DIRECT);
PID rightPID(&speed_current_right, &pwm_right, &setpoint_right, Kp, Ki, Kd, DIRECT);

// Odometry variables
double x_pos = 0.0;      // X position in meters
double y_pos = 0.0;      // Y position in meters
double theta = 0.0;      // Heading angle in radians
long total_left_enc = 0;  
long total_right_enc = 0;

unsigned long last_odom_time = 0;
double dx = 0.0;  // Linear velocity
double dr = 0.0;  // Angular velocity
double dt = 0.0;  // Delta time

unsigned long last_print_time = 0;
const unsigned long pub_interval = 100;
unsigned long last_pid_time = 0;
const unsigned long pid_interval = 50;

// imu calib
float gyro_bias_x = 0.0;
float gyro_bias_y = 0.0;
float gyro_bias_z = 0.0;

void get_cmd(const void *msgin)
{
    const geometry_msgs__msg__Twist *cmd_msg_sub = (const geometry_msgs__msg__Twist *)msgin;
    double linear = (double)cmd_msg_sub->linear.x;
    double angular = (double)cmd_msg_sub->angular.z;
    setpoint_left = linear - angular * (WHEEL_BASE / 2);
    setpoint_right = linear + angular * (WHEEL_BASE / 2);
}

void imu_pub_callback(rcl_timer_t *imu_time, int64_t last_call_time)
{
    RCL_UNUSED(last_call_time);
    RCL_UNUSED(imu_time);
}
void pub_vels_callback(rcl_timer_t *vels_time, int64_t last_call_time)
{
    RCL_UNUSED(last_call_time);
    RCL_UNUSED(vels_time);
}

void setup() {
  set_microros_transports();
  motor_init();

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  leftPID.SetOutputLimits(60,255);
  rightPID.SetOutputLimits(60,255);
  leftPID.SetSampleTime(pid_interval);
  rightPID.SetSampleTime(pid_interval);

  setpoint_left = 0.0;
  setpoint_right = 0.0;

  if (!mpu.begin()) {
    while (1);
  }
  // Calibrate gyroscope (robot must be stationary!)
  delay(1000);
  float sum_x = 0, sum_y = 0, sum_z = 0;
  int samples = 100;

  for(int i = 0; i < samples; i++) {
    mpu.readSensor();
    sum_x += mpu.getGyroX_rads();
    sum_y += mpu.getGyroY_rads();
    sum_z += mpu.getGyroZ_rads();
    delay(10);
  }

  gyro_bias_x = sum_x / samples;
  gyro_bias_y = sum_y / samples;
  gyro_bias_z = sum_z / samples;

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "gladius_node", "", &support);

  odom_msg_pub.header.frame_id.data = (char*)"odom_enc";
  odom_msg_pub.header.frame_id.size = strlen(odom_msg_pub.header.frame_id.data);
  odom_msg_pub.header.frame_id.capacity = odom_msg_pub.header.frame_id.size + 1;
  
  odom_msg_pub.child_frame_id.data = (char*)"base_footprint";
  odom_msg_pub.child_frame_id.size = strlen(odom_msg_pub.child_frame_id.data);
  odom_msg_pub.child_frame_id.capacity = odom_msg_pub.child_frame_id.size + 1;
  
  for(int i = 0; i < 36; i++) {
    odom_msg_pub.pose.covariance[i] = 0.0;
    odom_msg_pub.twist.covariance[i] = 0.0;
  }
  
  odom_msg_pub.pose.covariance[0] = 0.001;   // x
  odom_msg_pub.pose.covariance[7] = 0.001;   // y
  odom_msg_pub.pose.covariance[35] = 0.1; 
  
  odom_msg_pub.twist.covariance[0] = 0.001;  // vx
  odom_msg_pub.twist.covariance[7] = 0.001;  // vy
  odom_msg_pub.twist.covariance[35] = 0.01;  // vyaw
  
  imu_msg_pub.header.frame_id.data = (char*)"imu_link";
  imu_msg_pub.header.frame_id.size = strlen(imu_msg_pub.header.frame_id.data);
  imu_msg_pub.header.frame_id.capacity = imu_msg_pub.header.frame_id.size + 1;
  
  for(int i = 0; i < 9; i++) {
    imu_msg_pub.orientation_covariance[i] = 0.0;
    imu_msg_pub.angular_velocity_covariance[i] = 0.0;
    imu_msg_pub.linear_acceleration_covariance[i] = 0.0;
  }
  
  imu_msg_pub.orientation_covariance[0] = -1.0;
  
  imu_msg_pub.angular_velocity_covariance[0] = 0.02;
  imu_msg_pub.angular_velocity_covariance[4] = 0.02;
  imu_msg_pub.angular_velocity_covariance[8] = 0.02;
  
  imu_msg_pub.linear_acceleration_covariance[0] = 0.04;
  imu_msg_pub.linear_acceleration_covariance[4] = 0.04;
  imu_msg_pub.linear_acceleration_covariance[8] = 0.04;

  rclc_subscription_init_default(
    &cmd_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel");

  rclc_publisher_init_default(
        &odom_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom_enc");

  rclc_publisher_init_default(
      &imu_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu_data");

  rclc_timer_init_default(
      &odom_timer,
      &support,
      RCL_MS_TO_NS(pub_interval),
      pub_vels_callback);

  rclc_timer_init_default(
      &imu_timer,
      &support,
      RCL_MS_TO_NS(pub_interval),
      imu_pub_callback);

  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg_sub, &get_cmd, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &odom_timer);
  rclc_executor_add_timer(&executor, &imu_timer);

  attachInterrupt(left_enc_ch1, left_encoder, RISING);
  attachInterrupt(right_enc_ch1, right_encoder, RISING);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  
  static unsigned long last_encoder_time = 0;
  unsigned long current_time = millis();
  unsigned long elapsed_time = current_time - last_encoder_time;
  
  float x, y, z;
  float gx, gy, gz;

  if (current_time - last_pid_time >= pid_interval) {
    if (elapsed_time > 0) {
      speed_current_left = ((double)lef_enc_count / ENCODER_TPR) * (2 * PI * WHEEL_RADIUS) * (1000.0 / elapsed_time);
      speed_current_right = ((double)right_enc_count / ENCODER_TPR) * (2 * PI * WHEEL_RADIUS) * (1000.0 / elapsed_time);
      
      updateOdometry(lef_enc_count, right_enc_count);
    }

    leftPID.Compute();
    rightPID.Compute();

    if (setpoint_left > 0){
      left_motor_forward(pwm_left);
    }
    else if (setpoint_left < 0){
      left_motor_backward(abs(pwm_left));
    }
    else {
      left_motor_stop();
    }

    if (setpoint_right > 0){
      right_motor_forward(pwm_right);
    }
    else if (setpoint_right < 0){
      right_motor_backward(abs(pwm_right));
    }
    else {
      right_motor_stop();
    }

    lef_enc_count = 0;
    right_enc_count = 0;
    last_encoder_time = current_time;
    last_pid_time = current_time;
  }

  if (current_time - last_print_time >= pub_interval) {
    mpu.readSensor();
    x = mpu.getAccelX_mss();
    y = mpu.getAccelY_mss();
    z = mpu.getAccelZ_mss();
    gx = mpu.getGyroX_rads() - gyro_bias_x;
    gy = mpu.getGyroY_rads() - gyro_bias_y;
    gz = mpu.getGyroZ_rads() - gyro_bias_z;

    imu_msg_pub.header.stamp.sec = (int32_t)(current_time / 1000);
    imu_msg_pub.header.stamp.nanosec = (uint32_t)((current_time % 1000) * 1000000);

    imu_msg_pub.linear_acceleration.x = x; // Already in m/s²
    imu_msg_pub.linear_acceleration.y = y;
    imu_msg_pub.linear_acceleration.z = z;

    imu_msg_pub.angular_velocity.x = gx; // Already in rad/s
    imu_msg_pub.angular_velocity.y = gy;
    imu_msg_pub.angular_velocity.z = gz;

    rcl_publish(&imu_pub, &imu_msg_pub, NULL);

    odom_quat.x = 0.0;
    odom_quat.y = 0.0;
    odom_quat.z = sin(theta / 2.0);
    odom_quat.w = cos(theta / 2.0);

    odom_msg_pub.header.stamp.sec = (int32_t)(current_time / 1000);
    odom_msg_pub.header.stamp.nanosec = (uint32_t)((current_time % 1000) * 1000000);
    
    odom_msg_pub.pose.pose.position.x = x_pos;
    odom_msg_pub.pose.pose.position.y = y_pos;
    odom_msg_pub.pose.pose.position.z = 0.0;
    odom_msg_pub.pose.pose.orientation = odom_quat;

    odom_msg_pub.twist.twist.linear.x = dx;
    odom_msg_pub.twist.twist.linear.y = 0.0;
    odom_msg_pub.twist.twist.angular.z = dr;

    rcl_publish(&odom_pub, &odom_msg_pub, NULL);

    last_print_time = current_time;
  }
}

void updateOdometry(int left_enc_delta, int right_enc_delta) {
  unsigned long current_time = millis();
  dt = (current_time - last_odom_time) / 1000.0; 
  
  if (dt <= 0) return; // Avoid division by zero
  
  last_odom_time = current_time;
  
  total_left_enc += left_enc_delta;
  total_right_enc += right_enc_delta;
  
  // Calculate distances traveled by each wheel
  double left_distance = (left_enc_delta / (double)ENCODER_TPR) * (2.0 * PI * WHEEL_RADIUS);
  double right_distance = (right_enc_delta / (double)ENCODER_TPR) * (2.0 * PI * WHEEL_RADIUS);
  
  // Calculate change in orientation first (before updating theta)
  double delta_theta = (right_distance - left_distance) / WHEEL_BASE;
  
  // Calculate center distance
  double center_distance = (left_distance + right_distance) / 2.0;
  
  // Update position using the current heading (not averaged)
  // This is more accurate for small time steps
  x_pos += center_distance * cos(theta);
  y_pos += center_distance * sin(theta);
  
  // Update heading AFTER position calculation
  theta += delta_theta;
  
  // Normalize theta to [-PI, PI]
  while (theta > PI) theta -= 2.0 * PI;
  while (theta < -PI) theta += 2.0 * PI;
  
  // Calculate velocities
  dx = center_distance / dt;
  dr = delta_theta / dt;
}

void left_encoder(){
  if(digitalRead(left_enc_ch2)==1){
    lef_enc_count++;
  }
  else{
    lef_enc_count--;
  }
}

void right_encoder(){
  if(digitalRead(right_enc_ch2)==1){
    right_enc_count++;
  }
  else{
    right_enc_count--;
  }
}