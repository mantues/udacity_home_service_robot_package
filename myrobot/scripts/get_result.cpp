// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>  // add goal navi
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <std_msgs/Float32MultiArray.h>
// %EndTag(INCLUDES)%

//Grobal
int position = 0;

// Class navCallBack
class navCallBack
{
  private:
      // Publisher and Subscriber
      ros::NodeHandle nh;
      ros::Publisher pub_com;
      ros::Subscriber sub_com;

  public:
      //callback function
      void callback(const move_base_msgs::MoveBaseActionResult &msg_sub);
  navCallBack()
  {
      // Subscribe result of move_base
      sub_com = nh.subscribe("/move_base/result", 10, &navCallBack::callback, this);
      // Publish get_result
      pub_com = nh.advertise<std_msgs::String>("/get_result", 1);
      // Publish marker
      pub_msg = nh.advertise<std_msgs::Float32MultiArray>("/spawn_marker", 1);
  }
};

// Callback function
void navCallBack::callback(const move_base_msgs::MoveBaseActionResult &msg_sub)
{
    ROS_INFO("Get result Message!!!!!\n");
    std_msgs::String pub_msg;
  	pub_msg.data = "Target reached!!";
	  pub_com.publish(pub_msg);

    if (position==0) {
        double sx = 3.16, sy = -4.78, st = 5.0;
        std_msgs::Float32MultiArray pub_data;
        pub_data.data.resize(3);
	      pub_data.data[0] = sx;
	      pub_data.data[1] = sy;
	      pub_data.data[2] = st;
	      pub_msg.publish(pub_data);
	      position++;
    }
    else if (position==1) {
        double sx = 2.19, sy = 1.8, st = 0.0;
        std_msgs::Float32MultiArray pub_data;
        pub_data.data.resize(3);
	      pub_data.data[0] = sx;
	      pub_data.data[1] = sy;
	      pub_data.data[2] = st;
	      pub_msg.publish(pub_data);
        position++;
    }
}

// Main function
int main(int argc, char **argv)
{
    // Initialize node
    ros::init(argc, argv, "get_result");
    // Create instance
    navCallBack navcallback;
    // Ros spin
    ros::spin();
    return 0;
}
