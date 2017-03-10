#include <ros/ros.h>
#include "rosproxy_msgs/Odometry.h"
#include "rosproxy_msgs/OdometryCovariances.h"
#include "rosproxy_msgs/RequestOdometryCovariances.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include <string>

void mySigintHandler(int sig)
{
  ROS_INFO("Received SIGINT signal, shutting down...");
  ros::shutdown();
}

class OdomProxyNode
{

public:
  OdomProxyNode();

public:
  void odom_callback(const rosproxy_msgs::Odometry::ConstPtr& odomMsg);
  int run();

protected:
  ros::NodeHandle nh;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Publisher odom_pub;
  ros::Subscriber odom_sub;
  ros::ServiceClient odom_client;
  geometry_msgs::TransformStamped trans_msg;
  nav_msgs::Odometry odom_msg;

  bool pub_odom_tf;
  std::string odom_frame;
  std::string base_frame;

};

OdomProxyNode::OdomProxyNode()
{

  // CBA Read local params (from launch file)
  ros::NodeHandle nhLocal("~");
  nhLocal.param("pub_odom_tf", pub_odom_tf, true);
  ROS_INFO_STREAM("pub_odom_tf: " << pub_odom_tf);
  nhLocal.param<std::string>("odom_frame", odom_frame, "");
  ROS_INFO_STREAM("odom_frame: " << odom_frame);
  nhLocal.param<std::string>("base_frame", base_frame, "");
  ROS_INFO_STREAM("base_frame: " << base_frame);

}

void OdomProxyNode::odom_callback(const rosproxy_msgs::Odometry::ConstPtr& odomMsg)
{
  if ( pub_odom_tf )
  {
  trans_msg.header.stamp = odomMsg->header.stamp;
  if ( !odom_frame.empty() )
    trans_msg.header.frame_id = odom_frame;
  else
    trans_msg.header.frame_id = odomMsg->header.frame_id;
  if ( !base_frame.empty() )
    trans_msg.child_frame_id = base_frame;
  else
    trans_msg.child_frame_id = odomMsg->child_frame_id;
  trans_msg.transform.translation.x = odomMsg->pose.position.x;
  trans_msg.transform.translation.y = odomMsg->pose.position.y;
  trans_msg.transform.translation.z = odomMsg->pose.position.z;
  trans_msg.transform.rotation.x = odomMsg->pose.orientation.x;
  trans_msg.transform.rotation.y = odomMsg->pose.orientation.y;
  trans_msg.transform.rotation.z = odomMsg->pose.orientation.z;
  trans_msg.transform.rotation.w = odomMsg->pose.orientation.w;
  odom_broadcaster.sendTransform(trans_msg);
  }

  odom_msg.header.stamp = odomMsg->header.stamp;
  if ( !odom_frame.empty() )
    odom_msg.header.frame_id = odom_frame;
  else
    odom_msg.header.frame_id = odomMsg->header.frame_id;
  if ( !base_frame.empty() )
    odom_msg.child_frame_id = base_frame;
  else
    odom_msg.child_frame_id = odomMsg->child_frame_id;
  odom_msg.pose.pose.position.x = odomMsg->pose.position.x;
  odom_msg.pose.pose.position.y = odomMsg->pose.position.y;
  odom_msg.pose.pose.position.z = odomMsg->pose.position.z;
  odom_msg.pose.pose.orientation.x = odomMsg->pose.orientation.x;
  odom_msg.pose.pose.orientation.y = odomMsg->pose.orientation.y;
  odom_msg.pose.pose.orientation.z = odomMsg->pose.orientation.z;
  odom_msg.pose.pose.orientation.w = odomMsg->pose.orientation.w;
  odom_msg.twist.twist.linear.x = odomMsg->twist.linear.x;
  odom_msg.twist.twist.linear.y = odomMsg->twist.linear.y;
  odom_msg.twist.twist.linear.z = odomMsg->twist.linear.z;
  odom_msg.twist.twist.angular.x = odomMsg->twist.angular.x;
  odom_msg.twist.twist.angular.y = odomMsg->twist.angular.y;
  odom_msg.twist.twist.angular.z = odomMsg->twist.angular.z;
  odom_msg.pose.covariance[0] = odomMsg->pose_covariance[0];
  odom_msg.pose.covariance[7] = odomMsg->pose_covariance[1];
  odom_msg.pose.covariance[14] = odomMsg->pose_covariance[2];
  odom_msg.pose.covariance[21] = odomMsg->pose_covariance[3];
  odom_msg.pose.covariance[28] = odomMsg->pose_covariance[4];
  odom_msg.pose.covariance[35] = odomMsg->pose_covariance[5];
  odom_msg.twist.covariance[0] = odomMsg->twist_covariance[0];
  odom_msg.twist.covariance[7] = odomMsg->twist_covariance[1];
  odom_msg.twist.covariance[14] = odomMsg->twist_covariance[2];
  odom_msg.twist.covariance[21] = odomMsg->twist_covariance[3];
  odom_msg.twist.covariance[28] = odomMsg->twist_covariance[4];
  odom_msg.twist.covariance[35] = odomMsg->twist_covariance[5];
  odom_pub.publish(odom_msg);
}

int OdomProxyNode::run()
{
/*
  ROS_INFO("Requesting Odometry Covariances");
  odom_client = nh.serviceClient<rosproxy_msgs::RequestOdometryCovariances>("rosproxy/odom_covar_srv");
  rosproxy_msgs::RequestOdometryCovariances odomCovarSrv;
  if (odom_client.call(odomCovarSrv))
  {
    //odom_msg.orientation_covariance[0] = odomCovarSrv.response.orientation_covariance[0];
  }
  else
  {
    ROS_WARN("Failed to retrieve Odometry Covariances");
  }
*/
  ROS_INFO("Broadcasting odom tf");
  ROS_INFO("Publishing to topic /odom");
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);
  ROS_INFO("Subscribing to topic /rosproxy/odom");
  odom_sub = nh.subscribe("rosproxy/odom", 1000, &OdomProxyNode::odom_callback, this);
  ROS_INFO("Relaying between topics...");
  ros::spin();
  ROS_INFO("Exiting");
  return 0;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "odom_proxy_node");

  OdomProxyNode node;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigintHandler);

  return node.run();
}

