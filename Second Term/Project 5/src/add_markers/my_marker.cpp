#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
using namespace std;

class odometry{
public:
	double velocity;
	double x_position, y_position, z_position, x_orientation, y_orientation, z_orientation, w_orientation;
	void odom_listener(const nav_msgs::Odometry::ConstPtr& msg);
};
void odometry::odom_listener(const nav_msgs::Odometry::ConstPtr& msg){
	x_position = msg->pose.pose.position.x;
	y_position = msg->pose.pose.position.y;
	z_position = msg->pose.pose.position.z;
	x_orientation = msg->pose.pose.orientation.x;
	y_orientation = msg->pose.pose.orientation.y;
	z_orientation = msg->pose.pose.orientation.z;
	w_orientation = msg->pose.pose.orientation.w;
	velocity = msg->twist.twist.linear.x;
}

int main( int argc, char** argv )
{
  odometry odom;
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub_odom = n.subscribe("/odom", 1000, &odometry::odom_listener, &odom); 
	
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
    marker.ns = "basic_shape_1";
    marker.id = 1;

    // Set the marker type. 
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified 	in the header
    marker.pose.position.x = 4.5;
    marker.pose.position.y = 2.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.5;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();  //this line keeps the marker alive for only 5 seconds. 
    marker_pub.publish(marker);
	
	double th_x =  fabs(fabs(marker.pose.position.x) - fabs(odom.x_position));
	double th_y =  fabs(fabs(marker.pose.position.y) - fabs(odom.y_position));
	double th_z =  fabs(fabs(marker.pose.position.z) - fabs(odom.z_position));
	double th_x_o =  fabs(fabs(marker.pose.orientation.x) - fabs(odom.x_orientation));
	double th_y_o =  fabs(fabs(marker.pose.orientation.y) - fabs(odom.y_orientation));
	double th_z_o =  fabs(fabs(marker.pose.orientation.z) - fabs(odom.z_orientation));
	double th_w_o =  fabs(fabs(marker.pose.orientation.w) - fabs(odom.w_orientation));
	double module_1 = sqrt(pow(marker.pose.position.x-odom.x_position,2) + pow(marker.pose.position.y-odom.y_position,2));
	
	bool condition = (th_x < 0.4) && (th_y < 0.5) && (odom.velocity < 1e-3);

	ROS_INFO("th_x: [%f]", th_x);
	ROS_INFO("th_y: [%f]", th_y);
    ROS_INFO("module_1: [%f]", module_1);

	if ((condition == true) && (module_1 < 0.5)){
		ROS_INFO("First point achieved");
        marker.action = visualization_msgs::Marker::DELETE;
		break;	
	}

	ros::spinOnce();
	r.sleep();
  }
  
  sleep(5);
  
  while (ros::ok())
  { 
	//Generate the new marker
    visualization_msgs::Marker New_marker;

    New_marker.header.frame_id = "map";
    New_marker.header.stamp = ros::Time::now();

    New_marker.ns = "basic_shape_2";
    New_marker.id = 2;

    New_marker.type = shape;

	New_marker.action = visualization_msgs::Marker::ADD;

	// Set the new pose of the marker.  
	New_marker.pose.position.x = -4.5;
	New_marker.pose.position.y = 1.0;
	New_marker.pose.position.z = 0.0;
	New_marker.pose.orientation.x = 0.0;
	New_marker.pose.orientation.y = 0.0;
	New_marker.pose.orientation.z = 0.0;
	New_marker.pose.orientation.w = 0.5;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	New_marker.scale.x = 0.2;
	New_marker.scale.y = 0.2;
	New_marker.scale.z = 0.2;

	// Set the color -- be sure to set alpha to something non-zero!
	New_marker.color.r = 0.0f;
	New_marker.color.g = 0.0f;
	New_marker.color.b = 1.0f;
	New_marker.color.a = 1.0;  
	
	double th_x =  fabs(fabs(New_marker.pose.position.x) - fabs(odom.x_position));
	double th_y =  fabs(fabs(New_marker.pose.position.y) - fabs(odom.y_position));
	double th_z =  fabs(fabs(New_marker.pose.position.z) - fabs(odom.z_position));
	double th_x_o =  fabs(fabs(New_marker.pose.orientation.x) - fabs(odom.x_orientation));
	double th_y_o =  fabs(fabs(New_marker.pose.orientation.y) - fabs(odom.y_orientation));
	double th_z_o =  fabs(fabs(New_marker.pose.orientation.z) - fabs(odom.z_orientation));
	double th_w_o =  fabs(fabs(New_marker.pose.orientation.w) - fabs(odom.w_orientation));
	double module_2 = sqrt(pow(New_marker.pose.position.x-odom.x_position,2) + pow(New_marker.pose.position.y-odom.y_position,2));
	
	bool condition = (th_x < 0.4) && (th_y < 1.0) && (odom.velocity < 1e-3);
	
	ROS_INFO("th_x: [%f]", th_x);
	ROS_INFO("th_y: [%f]", th_y);
	ROS_INFO("module_2: [%f]", module_2);
													   
	if ((condition == true) && (module_2 < 1.0)){	
	    ROS_INFO("Second point achieved");	
		marker_pub.publish(New_marker);
	}

    ros::spinOnce();	
    r.sleep();
  }
}
