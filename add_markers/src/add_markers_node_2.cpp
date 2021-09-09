#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

float pickUpPose[6] = {7.5,-2.85, 0.707, 0.0 , 0.0, 0.707}; // rotation to 90 deg (1.57 rad)
float dropOffPose[6] = {0.0, 7.6, 0.0, 0.0, 0.0, 1.0}; // rotation to 180 deg (3.14 rad) 
bool pick_up_marker = true;
bool drop_off_marker = false;
bool object_picked = false;

// This callback function continuously executes and reads the image data
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
 
    float dist_pick_up = sqrt(pow((msg->pose.pose.position.x - pickUpPose[0]),2) + pow((msg->pose.pose.position.y - pickUpPose[1]),2));
    float dist_drop_off = sqrt(pow((msg->pose.pose.position.x - dropOffPose[0]),2) + pow((msg->pose.pose.position.y -  dropOffPose[1]),2));

    if ( dist_pick_up < 0.2 && !object_picked){
        ROS_INFO("The robot riched the object pick up zone");
        object_picked = true;
        pick_up_marker = false;
        drop_off_marker = false;
    }
    else if (dist_drop_off < 0.2 && object_picked){
        ROS_INFO("The robot riched the object dropped off zone");
        object_picked = false;
        drop_off_marker = true;
        pick_up_marker = false;
    }
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_markers_2");
    ros::NodeHandle n;
    ros::Rate r(5);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Subscribe to /odom topic to read the current robot position
    ros::Subscriber sub1 = n.subscribe("odom", 1, odom_callback);

     
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;
 
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
 
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
 
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    while (ros::ok()){
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  

        while (marker_pub.getNumSubscribers() < 1){
            if (!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            //sleep(5);
        }

        if (pick_up_marker){
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = 7.5;
            marker.pose.position.y = -2.85;
            marker.pose.position.z = 0;
            marker.lifetime = ros::Duration(0.2);
            marker_pub.publish(marker);
        }
        //else if (!pick_up_reached){
        //    ROS_INFO("PICK UP REACHED");
        //    marker.action = visualization_msgs::Marker::DELETE;
        //    marker_pub.publish(marker);
        //}
        if (drop_off_marker){
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = 0.0;
            marker.pose.position.y = 7.6;
            marker.pose.position.z = 0;
            marker.lifetime = ros::Duration(0.2);
            marker_pub.publish(marker);
        }
        ros::spinOnce();
        r.sleep();
    }
}