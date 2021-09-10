#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>  // add goal navi
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

// Global
int target_pos = 0;
int robot_state = 0; // 0: idle 1:moving 2:goal
int spawn = 0;

class GoalCallBack
{
private:
  ros::NodeHandle nh;
  ros::Publisher pub_goal;
  ros::Subscriber sub_status;
  ros::Publisher pub_msg;

public:
  void callback(const actionlib_msgs::GoalStatusArray::ConstPtr &status);

  GoalCallBack()
  {
    sub_status = nh.subscribe("/move_base/status", 10, &GoalCallBack::callback, this);
    pub_goal = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    pub_msg = nh.advertise<std_msgs::Float32MultiArray>("/spawn_marker", 1);
  }
};

void GoalCallBack::callback(const actionlib_msgs::GoalStatusArray::ConstPtr &status)
{
    ROS_INFO("Get Status!!!!!\n");
    if (target_pos==0) {
        ros::Duration(0.5).sleep();
        double sx = 3.16, sy = -4.78, st = 0.0;
        std_msgs::Float32MultiArray pub_data;
        pub_data.data.resize(3);
	    pub_data.data[0] = sx;
	    pub_data.data[1] = sy;
	    pub_data.data[2] = st;
	    pub_msg.publish(pub_data);
	}    
    
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
    if (target_pos==0) {
        ROS_INFO("Moving to goal1!!!!!\n");
        
        target_pos++;
        ros::Duration(0.5).sleep();
        
        double px=3.16, py=-4.78, pz=0.22, ow=0.97;    // move to goal2
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
                
                ros::Duration(5).sleep();
                                
                double px=2.19, py=1.8, pz=0.97, ow=0.03;    // move to goal2
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
            else if(target_pos==2) {    // reached target2
                target_pos++;
                //send_goal(2.19, 1.80, 0.97, 0.20);    // send goal
                //pub_com.publish(pub_msg);
                ROS_INFO("MISSION COMPLETED!!!!!\n");
            }
            /*else if(target_pos==3) {
                target_pos++;
                Goal goal_ob(-8.2, 0.01, 0.98, 0.0);    // send goal
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

    //ros::Subscriber move_base_status_sub;
    target_pos = 0;
    GoalCallBack goalcallback;
    //send_goal(3.66, -4.78, 0.22, 0.97);    // add goal navi

    ros::spin();
    return 0;
}
