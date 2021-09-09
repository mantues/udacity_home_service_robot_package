#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>  // add goal navi
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

class navCallBack
{
private:
  ros::NodeHandle nh;
  ros::Publisher pub_com;
  ros::Subscriber sub_com;

public:
  void callback(const move_base_msgs::MoveBaseActionResult &msg_sub);

  navCallBack()
  {
    sub_com = nh.subscribe("/move_base/result", 10, &navCallBack::callback, this);
    pub_com = nh.advertise<std_msgs::String>("/get_result", 1);
  }
};

void navCallBack::callback(const move_base_msgs::MoveBaseActionResult &msg_sub)
{
    ROS_INFO("Get result Message!!!!!\n");
    std_msgs::String pub_msg;
	pub_msg.data = "Target reached!!";
	pub_com.publish(pub_msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_result");
    //send_goal(3.16, -4.78, 0.22, 0.97);    // add goal navi
    //goal_ob(1.13, -0.04, 0, 1.0);    // add goal navi
    navCallBack navcallback;
    ros::spin();
    return 0;
}
