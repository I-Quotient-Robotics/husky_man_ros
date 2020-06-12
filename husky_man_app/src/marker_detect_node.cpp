#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <aruco_msgs/MarkerArray.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

bool receive_data = false;

geometry_msgs::TransformStamped transformStamped1, transformStamped2, transformStamped3;

void MarkerPoseCallback(const aruco_msgs::MarkerArray::ConstPtr& msg) {
  static tf2_ros::TransformBroadcaster br;
  // geometry_msgs::TransformStamped transformStamped1, transformStamped2;
  
  transformStamped1.header.stamp = ros::Time::now();
  transformStamped1.header.frame_id = "kinect2_rgb_optical_frame";
  transformStamped1.child_frame_id = "marker";
  transformStamped1.transform.translation.x = msg->markers[0].pose.pose.position.x;
  transformStamped1.transform.translation.y = msg->markers[0].pose.pose.position.y;
  transformStamped1.transform.translation.z = msg->markers[0].pose.pose.position.z;
  // tf2::Quaternion q;
  // q.setRPY(0, 0, msg->theta);
  transformStamped1.transform.rotation.x = msg->markers[0].pose.pose.orientation.x;
  transformStamped1.transform.rotation.y = msg->markers[0].pose.pose.orientation.y;
  transformStamped1.transform.rotation.z = msg->markers[0].pose.pose.orientation.z;
  transformStamped1.transform.rotation.w = msg->markers[0].pose.pose.orientation.w;
  // br.sendTransform(transformStamped1);

  transformStamped2.header.stamp = ros::Time::now();
  transformStamped2.header.frame_id = "marker";
  transformStamped2.child_frame_id = "object";
  transformStamped2.transform.translation.x = 0.00;
  transformStamped2.transform.translation.y = -0.036;
  transformStamped2.transform.translation.z = 0.00;

  tf2::Quaternion q;
  q.setRPY(M_PI/2.0, M_PI/2.0, 0.0);
  transformStamped2.transform.rotation.x = q.x();
  transformStamped2.transform.rotation.y = q.y();
  transformStamped2.transform.rotation.z = q.z();
  transformStamped2.transform.rotation.w = q.w();
  // br.sendTransform(transformStamped2);

  receive_data = true;

  // ROS_INFO("update Marker pose");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "markder_detect_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::WallDuration(1.0).sleep();

  ros::Subscriber marker_pose_sub = nh.subscribe("aruco_marker_publisher/markers", 1000, MarkerPoseCallback);

  ROS_INFO("Wait for first marker...");
  while(receive_data == false) {
    ros::WallDuration(1.0).sleep();
  }

  ROS_INFO("Start object tf publish...");
  tf2_ros::TransformBroadcaster br;
  while(ros::ok()) {
    br.sendTransform(transformStamped1);
    br.sendTransform(transformStamped2);
  }
  ROS_INFO("Stop object tf publish");
  ros::waitForShutdown();
  return 0;
}