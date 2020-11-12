#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

float initial_pos[3] = {2.0, -11.0, 0.0};
float pickup[3] = {-2.0, -4.0, 0.0};
float dropoff[3] = {0.0, 3.0, 0.0};

bool reached_pickup = false;
bool reached_dropoff = false;

void callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (std::abs(initial_pos[0] + msg->pose.pose.position.x - pickup[0]) < 0.2 
     && std::abs(initial_pos[1] + msg->pose.pose.position.y - pickup[1]) < 0.2 
     && std::abs(pickup[2] - msg->pose.pose.position.z) < 0.001)
    reached_pickup = true;
  ROS_INFO("Pose x: [%f], y: [%f]", initial_pos[0] + msg->pose.pose.position.x,
             initial_pos[1] + msg->pose.pose.position.y);
  
  if (reached_pickup){
  	if (std::abs(initial_pos[0] + msg->pose.pose.position.x - dropoff[0]) < 0.8 
     	&& std::abs(initial_pos[1] + msg->pose.pose.position.y - dropoff[1]) < 0.8 
     	&& std::abs(dropoff[2] - msg->pose.pose.position.z) < 0.001)
    	reached_dropoff = true;
    ROS_INFO("Pose x: [%f], y: [%f]", initial_pos[0] + msg->pose.pose.position.x,
             initial_pos[1] + msg->pose.pose.position.y);
  }
  //ROS_INFO("Pose x: [%f], y: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y);
  //ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe("odom", 1, callback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pickup[0];
    marker.pose.position.y = pickup[1];
    marker.pose.position.z = pickup[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    
    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
   
    // Pick up marker 
    bool pickup_zone = false;
    do {
      if (reached_pickup){
        ROS_INFO("The robot has reached the pick up zone");
        ros::Duration(5.0).sleep();
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
        ROS_INFO("The robot has picked up the virtual object");
        pickup_zone = true;
      }
      //ROS_INFO("The robot is heading to the pickup zone");
      ros::spinOnce();
    } while(!pickup_zone);
    
    
    // Drop off marker
    bool dropoff_zone = false;
    do {
      if (reached_dropoff){
        marker.pose.position.x = dropoff[0];
        marker.pose.position.y = dropoff[1];
        marker.pose.position.z = dropoff[2];
        marker.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(marker);
        ROS_INFO("The robot has dropped off the virtual object");
        dropoff_zone = true;
      }
      ros::spinOnce();
    } while(!dropoff_zone);
        
    return 0;
  }
}