#include <ros/ros.h>
#include "rosproxy_msgs/Imu.h"
#include "rosproxy_msgs/ImuCovariances.h"
#include "rosproxy_msgs/RequestImuCovariances.h"
#include <sensor_msgs/Imu.h>
#include <signal.h>
#include <string>

void mySigintHandler(int sig)
{
  ROS_INFO("Received SIGINT signal, shutting down...");
  ros::shutdown();
}

class ImuProxyNode
{

public:
  ImuProxyNode();

public:
  void imu_callback(const rosproxy_msgs::Imu::ConstPtr& imuMsg);
  int run();

protected:
  ros::NodeHandle nh;
  ros::Publisher imu_pub;
  ros::Subscriber imu_sub;
  ros::ServiceClient imu_client;
  sensor_msgs::Imu imu_msg;

  std::string imu_frame;

};

ImuProxyNode::ImuProxyNode()
{

  // CBA Read local params (from launch file)
  ros::NodeHandle nhLocal("~");
  nhLocal.param<std::string>("imu_frame", imu_frame, "");
  ROS_INFO_STREAM("imu_frame: " << imu_frame);

  /*
  # Orientation covariance estimation:
  # Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
  # Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
  # Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
  # cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
  # i.e. variance in yaw: 0.0025
  # Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
  # static roll/pitch error of 0.8%, owing to gravity orientation sensing
  # error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
  # so set all covariances the same.
  */
  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;
  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[5] = 0;
  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0.0025;

  /*
  # Angular velocity covariance estimation:
  # Observed gyro noise: 4 counts => 0.28 degrees/sec
  # nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
  # Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
  */
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;

  /*
  # linear acceleration covariance estimation:
  # observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
  # nonliniarity spec: 0.5% of full scale => 0.2m/s^2
  # Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
  */
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

}

void ImuProxyNode::imu_callback(const rosproxy_msgs::Imu::ConstPtr& imuMsg)
{
  imu_msg.header.stamp = imuMsg->header.stamp;
  if ( !imu_frame.empty() )
    imu_msg.header.frame_id = imu_frame;
  else
    imu_msg.header.frame_id = imuMsg->header.frame_id;
  imu_msg.orientation.x = imuMsg->orientation.x;
  imu_msg.orientation.y = imuMsg->orientation.y;
  imu_msg.orientation.z = imuMsg->orientation.z;
  imu_msg.orientation.w = imuMsg->orientation.w;
  imu_msg.linear_acceleration.x = imuMsg->linear_acceleration.x;
  imu_msg.linear_acceleration.y = imuMsg->linear_acceleration.y;
  imu_msg.linear_acceleration.z = imuMsg->linear_acceleration.z;
  imu_msg.angular_velocity.x = imuMsg->angular_velocity.x;
  imu_msg.angular_velocity.y = imuMsg->angular_velocity.y;
  imu_msg.angular_velocity.z = imuMsg->angular_velocity.z;
  imu_msg.orientation_covariance[0] = imuMsg->orientation_covariance[0];
  imu_msg.orientation_covariance[4] = imuMsg->orientation_covariance[1];
  imu_msg.orientation_covariance[8] = imuMsg->orientation_covariance[2];
  imu_msg.linear_acceleration_covariance[0] = imuMsg->linear_acceleration_covariance[0];
  imu_msg.linear_acceleration_covariance[4] = imuMsg->linear_acceleration_covariance[1];
  imu_msg.linear_acceleration_covariance[8] = imuMsg->linear_acceleration_covariance[2];
  imu_msg.angular_velocity_covariance[0] = imuMsg->angular_velocity_covariance[0];
  imu_msg.angular_velocity_covariance[4] = imuMsg->angular_velocity_covariance[1];
  imu_msg.angular_velocity_covariance[8] = imuMsg->angular_velocity_covariance[2];
  imu_pub.publish(imu_msg);
}

int ImuProxyNode::run()
{
/*
  ROS_INFO("Requesting Imu Covariances");
  imu_client = nh.serviceClient<rosproxy_msgs::RequestImuCovariances>("rosproxy/imu_covar_srv");
  rosproxy_msgs::RequestImuCovariances imuCovarSrv;
  if (imu_client.call(imuCovarSrv))
  {
    //imu_msg.orientation_covariance[0] = imuCovarSrv.response.orientation_covariance[0];
  }
  else
  {
    ROS_WARN("Failed to retrieve Imu Covariances");
  }
*/
  ROS_INFO("Publishing to topic /imu/data");
  imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 1000);
  ROS_INFO("Subscribing to topic /rosproxy/imu");
  imu_sub = nh.subscribe("rosproxy/imu", 1000, &ImuProxyNode::imu_callback, this);
  ROS_INFO("Relaying between topics...");
  ros::spin();
  ROS_INFO("Exiting");
  return 0;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "imu_proxy_node");

  ImuProxyNode node;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigintHandler);

  return node.run();
}

