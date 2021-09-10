#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>  // add goal navi
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <std_msgs/Float32MultiArray.h>

//Grobal
int position = 0;
 
class navCallBack
{
private:
  ros::NodeHandle nh;
  ros::Publisher pub_com;
  ros::Publisher pub_msg;
  ros::Subscriber sub_com;

public:
  void callback(const move_base_msgs::MoveBaseActionResult &msg_sub);
  void publisher(void);
  
  navCallBack()
  {
    sub_com = nh.subscribe("/move_base/result", 10, &navCallBack::callback, this);
    pub_com = nh.advertise<std_msgs::String>("/get_result", 1);
    pub_msg = nh.advertise<std_msgs::Float32MultiArray>("/spawn_marker", 1);
  }
};

void navCallBack::publisher(void)
{
    double sx = 3.16, sy = -4.78, st = 5.0;
    std_msgs::Float32MultiArray pub_data;
    pub_data.data.resize(3);
    pub_data.data[0] = sx;
	pub_data.data[1] = sy;
	pub_data.data[2] = st;
	pub_msg.publish(pub_data);
    
}
void navCallBack::callback(const move_base_msgs::MoveBaseActionResult &msg_sub)
{
    ROS_INFO("Get result Message!!!!!\n");
    //std_msgs::String pub_msg;
	//pub_msg.data = "Target reached!!";
	//pub_com.publish(pub_msg);
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_result");
    //send_goal(3.16, -4.78, 0.22, 0.97);    // add goal navi
    //goal_ob(1.13, -0.04, 0, 1.0);    // add goal navi
    navCallBack navcallback;
    ros::spin();
    return 0;
}
