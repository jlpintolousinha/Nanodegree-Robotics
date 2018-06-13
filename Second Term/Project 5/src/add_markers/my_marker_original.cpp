#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odometry_values; 
	
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

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
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
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(5.0);  //this line keeps the marker alive for only 5 seconds. 
    //odometry_values = n.subscribe("/odom", 1000, callback);
       
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
    
	sleep(5);
	r.sleep();
	break;
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
	New_marker.color.r = 1.0f;
	New_marker.color.g = 0.0f;
	New_marker.color.b = 0.0f;
	New_marker.color.a = 1.0;  

	marker_pub.publish(New_marker);
		
    r.sleep();
  }
}
