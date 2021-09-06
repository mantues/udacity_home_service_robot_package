#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>  // add goal navi
#include <std_msgs/String.h>

// Global
int target_pos = 0;
int robot_state = 0; // 0: idle 1:moving 2:goal

class Goal {
public:
    Goal(double px, double py, double pz, double ow);
    ~Goal();
 
private:
    ros::Publisher pub;
    ros::NodeHandle nh;
};

Goal::Goal(double px, double py, double pz, double ow){
    pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
 
    ros::Rate one_sec(1);
    one_sec.sleep();
     
    ros::Time time = ros::Time::now();
    geometry_msgs::PoseStamped goal_point;
 
    goal_point.pose.position.x = px;
    goal_point.pose.position.y = py;
    goal_point.pose.position.z =  pz;
    goal_point.pose.orientation.w = ow;
    goal_point.header.stamp = time;
    goal_point.header.frame_id = "map";
 
    pub.publish(goal_point);
 
}
 
Goal::~Goal(){
 
}

class Command {
public:
    Command();
    void Publish(void);
    ~Command();
    
private:
    ros::Publisher pub_com;
    ros::NodeHandle n;
    std_msgs::String pub_msg;
};

Command::Command()
{
    pub_com = n.advertise<std_msgs::String>("/get_result", 1000);
	pub_msg.data = "Target reached!!";
	pub_com.publish(pub_msg);
}
void Command::Publish(void)
{
	pub_msg.data = "message";
	pub_com.publish(pub_msg);
}


Command::~Command(){
 
}


void navStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr &status)
{

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
            robot_state = 2;
            ROS_INFO("Target = %d Goal reached!!!!!\n",target_pos);
            if(target_pos==1) { // reached target1
                Command Command;
                Command.Publish();
                ROS_INFO("Target reached!!!!!\n");
                target_pos++;
                Goal goal_ob(3.16, -4.78, 0.22, 0.97);    // move to goal2
            }
            /*else if(target_pos==2) {    // reached target2
                target_pos++;
                Goal goal_ob(2.19, 1.80, 0.97, 0.20);    // send goal
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
    ros::NodeHandle nnh;
    Command Command;
    /*int i = 0;
    while(i<1000){
        Command.Publish();
        i=i+1;
        std::cout << i << std::endl;
    }*/
    //ros::Subscriber switch_sub;

    ros::Subscriber move_base_status_sub;
    move_base_status_sub = nnh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 10, &navStatusCallBack);
    target_pos = 1;
    Goal goal_ob(-3.5, -4.64, 0.0, 1.0);    // add goal navi
    //goal_ob(1.13, -0.04, 0, 1.0);    // add goal navi
    Command.Publish();
    ros::spin();

    return 0;
}
