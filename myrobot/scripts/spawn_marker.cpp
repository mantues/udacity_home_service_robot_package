// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
// %EndTag(INCLUDES)%

class markerCallBack
{
private:
  ros::NodeHandle nh;
  ros::Publisher marker_pulisher;
  ros::Subscriber sub_com;

public:
  void callback(const std_msgs::Float32MultiArray &msg_sub);

  markerCallBack()
  {
    sub_com = nh.subscribe("/spawn_marker", 10, &markerCallBack::callback, this);
    marker_pulisher = nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);
  }
};

void markerCallBack::callback(const std_msgs::Float32MultiArray &msg_sub)
{
    ROS_INFO("SPAWN [%f, %f, %f]!!!!!\n", msg_sub.data[0], msg_sub.data[1], msg_sub.data[2]);
    double sx = msg_sub.data[0], sy = msg_sub.data[1], st = msg_sub.data[2];
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
// %EndTag(MARKER_INIT)%

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
// %Tag(NS_ID)%
    marker.ns = "basic_shapes";
    marker.id = 0;
// %EndTag(NS_ID)%

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
// %Tag(TYPE)%
    uint32_t shape = visualization_msgs::Marker::CUBE;
    marker.type = shape;
// %EndTag(TYPE)%

    // Set the marker action.  Options are ADD and DELETE
// %Tag(ACTION)%
    marker.action = visualization_msgs::Marker::ADD;
// %EndTag(ACTION)%

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
// %Tag(POSE)%
    marker.pose.position.x = sx;
    marker.pose.position.y = sy;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
// %EndTag(POSE)%

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
// %Tag(SCALE)%
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
// %EndTag(SCALE)%

    // Set the color -- be sure to set alpha to something non-zero!
// %Tag(COLOR)%
    marker.color.r = 0.5f;
    marker.color.g = 1.0f;
    marker.color.b = 0.5f;
    marker.color.a = 1.0;
// %EndTag(COLOR)%

// %Tag(LIFETIME)%
    marker.lifetime = ros::Duration(st);
// %EndTag(LIFETIME)%

    // Publish the marker
// %Tag(PUBLISH)%
    marker_pulisher.publish(marker);

    //Spawn_model.Publish(marker);
// %EndTag(PUBLISH)%

}


// %Tag(INIT)%
int main( int argc, char** argv )
{
  ros::init(argc, argv, "msgs_spawn_marker");

  markerCallBack markerballback;
  ros::spin();
  // %EndTag(SLEEP_END)%
}
// %EndTag(FULLTEXT)%
