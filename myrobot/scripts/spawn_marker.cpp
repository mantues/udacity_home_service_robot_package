/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
// %EndTag(INCLUDES)%

class Spawn_model {
public:
    Spawn_model(visualization_msgs::Marker msg);
    ~Spawn_model();
    
private:
    ros::Publisher marker_pub;
    ros::NodeHandle n;
};

Spawn_model::Spawn_model(visualization_msgs::Marker msg)
{
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100);
	marker_pub.publish(msg);
}

Spawn_model::~Spawn_model(){
 
}

void Model(std_msgs::String msg){
    ros::NodeHandle nh;
    ros::Rate r(1);
    ros::Publisher marker_pulisher;
    marker_pulisher = nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    ROS_INFO("SPAWN MARKER!!!!!\n");
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
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
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
    marker.lifetime = ros::Duration();
// %EndTag(LIFETIME)%

    // Publish the marker
// %Tag(PUBLISH)%
    marker_pulisher.publish(marker);
    Spawn_model spawn_model_ob();
    //Spawn_model.Publish(marker);
// %EndTag(PUBLISH)%
    r.sleep();
}


// %Tag(INIT)%
int main( int argc, char** argv )
{
  ros::init(argc, argv, "msgs_spawn_marker");
  ros::NodeHandle n;

// %EndTag(INIT)%
  ros::Subscriber status_sub;
  status_sub = n.subscribe<std_msgs::String>("/get_result", 10, &Model);

  ros::spin();
  // %EndTag(SLEEP_END)%
}
// %EndTag(FULLTEXT)%
