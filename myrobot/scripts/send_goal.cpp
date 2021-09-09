#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>  // add goal navi
#include <std_msgs/String.h>

// Global
int target_pos = 0;
int robot_state = 0; // 0: idle 1:moving 2:goal

class GoalCallBack
{
private:
  ros::NodeHandle nh;
  ros::Publisher pub_goal;
  ros::Subscriber sub_status;
  ros::Publisher pub_msg;

public:
  void callback(actionlib_msgs::GoalStatusArray &status);
  void send_goal(double px, double py, double pz, double ow);
  void send_msg(void);

  GoalCallBack()
  {
    sub_status = nh.subscribe("/move_base/status", 10, &GoalCallBack::callback, this);
    pub_goal = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    pub_msg = nh.advertise<std_msgs::String>("/get_result", 1);
  }
};

void GoalCallBack::send_msg(void)
{
    ROS_INFO("Get result Message!!!!!\n");
    std_msgs::String pub_data;
	pub_data.data = "Target reached!!";
	pub_msg.publish(pub_data);
}


void GoalCallBack::send_goal(double px, double py, double pz, double ow){
    ros::Time time = ros::Time::now();
    geometry_msgs::PoseStamped goal_point;
 
    goal_point.pose.position.x = px;
    goal_point.pose.position.y = py;
    goal_point.pose.position.z =  pz;
    goal_point.pose.orientation.w = ow;
    goal_point.header.stamp = time;
    goal_point.header.frame_id = "map";
 
    pub_goal.publish(goal_point);
 
}

void GoalCallBack::callback(actionlib_msgs::GoalStatusArray &status)
{
    ROS_INFO("Get Status!!!!!\n");
    int status_id = 0;
    //uint8 PENDING         = 0  
    //uint8 ACTIVE          = 1 
    //uint8 PREEMPTED       = 2
    //uint8 SUCCEEDED       = 3
    //uint8 ABORTED         = 4
    //uint8 REJECTED        = 5
    //uint8 PREEMPTING      = 6
    //uint8 RECALLING       = 7
    //uint8 RECALLED        = 8
    //uint8 LOST            = 9
    
    if (!status->status_list.empty()){
    actionlib_msgs::GoalStatus goalStatus = status->status_list[0];
    status_id = goalStatus.status;
    }

    ROS_INFO("status=%d\n",status_id);
    ROS_INFO("robot_state=%d\n",robot_state);
    if(status_id==1){
    //myrobot is moving
        ROS_INFO("Target = %d MOVING!!\n",target_pos);
        robot_state = 1;
    }

    if((status_id==3)||(status_id==0)){
    //goal reached or goal reached and waiting
    
        if(status_id==3){
            if(robot_state!=1){
                return;
            }
            if(target_pos==1) { // reached target1
                //pub_com.publish(pub_msg);
                ROS_INFO("Target reached!!!!!\n");
                target_pos++;
                GoalCallBack::goalcallback::send_goal(3.16, -4.78, 0.22, 0.97);    // move to goal2
            }
            /*else if(target_pos==2) {    // reached target2
                target_pos++;
                //send_goal(2.19, 1.80, 0.97, 0.20);    // send goal
                GoalCallBack::send_goal(3.66, -4.78, 0.22, 0.97);    // move to goal2
            }
            else if(target_pos==3) {
                target_pos++;
                Goal goal_ob(-8.2, 0.01, 0.98, 0.99);    // send goal
            }

            else if(target_pos==4){
                target_pos++;
                Goal goal_ob(-9.6, -5.21, 0.03, -0.99);    // send goal
            }
        
            else if(target_pos==5){
                target_pos++;
                Goal goal_ob(-3.5, -4.64, 0.0, 1.0);    // send goal
            }
            else if(target_pos==6){
                ROS_INFO("Final Goal reached!!!!!\n");
            }*/
            
        }
        else {
            ROS_INFO("Idle!\n");
            robot_state = 0;
        }

    }
    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_move_base_goal");
    GoalCallBack goalcallback;

    //ros::Subscriber move_base_status_sub;
    target_pos = 1;
    //send_goal(3.66, -4.78, 0.22, 0.97);    // add goal navi

    ros::spin();
    return 0;
}
