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
#include <geometry_msgs/PoseWithCovariance.h> 
#include "nav_msgs/Odometry.h"
#include <cmath>


bool poseAchieved = false;
geometry_msgs::Pose goalPose;

void poseCallback(const nav_msgs::Odometry::ConstPtr &msg){
  const geometry_msgs::Pose p = msg->pose.pose;
  const float positionErr = sqrt(pow(p.position.x-goalPose.position.x, 2) + pow(p.position.y-goalPose.position.y,2));
  if(positionErr < 0.3){
    poseAchieved = true;
  }else{
   ROS_INFO("Position Error: %f\n", positionErr);
  }
}

visualization_msgs::Marker createMarker(double *goalPos){
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "add_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = goalPos[0];
  marker.pose.position.y = goalPos[1];
  marker.pose.position.z = goalPos[2];
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 0;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  goalPose = marker.pose;
  return marker;
}

void publishMarker(ros::NodeHandle &n, visualization_msgs::Marker &marker,bool pickup ){

  poseAchieved = false;
 
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      break;
    }
    ROS_WARN_ONCE("Warning: Create a subscriber to the marker");
    ros::Duration(1).sleep();
  }
 
  ros::Subscriber bot_pose_subscriber = n.subscribe("odom", 1, poseCallback);

  //if(pickup){
  //ROS_INFO("Publishing pickup marker.");
  //marker_pub.publish(marker);}
  
  while(!poseAchieved && ros::ok()){
    ros::spinOnce();
    if(pickup)
    marker_pub.publish(marker); 
    ros::Duration(1).sleep();
  }

 if(!pickup){
  ROS_INFO("Publishing dropoff marker.");
  marker_pub.publish(marker);}
 else{
 ROS_INFO("deleting pickup marker.");
 marker.action = visualization_msgs::Marker::DELETE;
 marker_pub.publish(marker); 
}

}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
 
 
  double pickUpGoal[3]  = {2.90, 3.7,0.4};
  double dropOffGoal[3] = {-0.09, -4.31,0.4};
  
  ROS_INFO("Publishing pick up marker.");
  visualization_msgs::Marker marker1 = createMarker(pickUpGoal);
  publishMarker(n,marker1,true);

  if(poseAchieved){
    ROS_INFO("Reached pick up pose!");
  }else{
    ROS_INFO("Failed to reach the pick up pose.");
  }

  ROS_INFO("Picking up object");
  sleep(5);

  visualization_msgs::Marker marker2 = createMarker(dropOffGoal);
  publishMarker(n,marker2,false);  
  ROS_INFO("Task Finished");

}

