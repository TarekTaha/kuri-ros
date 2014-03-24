/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include "phantom_omni/PhantomButtonEvent.h"
#include <teleoperation/HapticTeleopConfig.h>
#include <dynamic_reconfigure/server.h>

class HapticTeleop
{
	public:
      HapticTeleop(ros::NodeHandle& n);
      void masterJointsCallbackModeJoints(const sensor_msgs::JointState::ConstPtr& joint_states);
      void masterJointsCallbackModeCartesian(const sensor_msgs::JointState::ConstPtr& joint_states);

      void slaveButtonCallback(const phantom_omni::PhantomButtonEvent::ConstPtr& button);
      void paramsCallback(teleoperation::HapticTeleopConfig &config, uint32_t level);

	private:
      double slave_velocity_min_x,   slave_velocity_max_x,   slave_velocity_len_x;
      double slave_velocity_min_y,   slave_velocity_max_y,   slave_velocity_len_y;
      double slave_velocity_min_z,   slave_velocity_max_z,   slave_velocity_len_z;
      double slave_velocity_min_yaw, slave_velocity_max_yaw, slave_velocity_len_yaw;

      double master_joint_min_x,   master_joint_max_x,   master_joint_len_x;
      double master_joint_min_y,   master_joint_max_y,   master_joint_len_y;
      double master_joint_min_z,   master_joint_max_z,   master_joint_len_z;
      double master_joint_min_yaw, master_joint_max_yaw, master_joint_len_yaw;
      double master_min_x, master_max_x, master_len_x;
      double master_min_y, master_max_y, master_len_y;
      double master_min_z, master_max_z, master_len_z;
      double master_min_yaw, master_max_yaw, master_len_yaw;

      double slave_velocity_scale_x;
      double slave_velocity_scale_y;
      double slave_velocity_scale_z;
      double slave_velocity_scale_yaw;
      tf::TransformListener listener;
      tf::StampedTransform transform;

      int mode_;
      ros::NodeHandle n_,n_priv_;
	  ros::Publisher vel_pub_;
      //ros::Publisher joint_state_sub_;
  	  ros::Subscriber omni_sub_,button_sub_;
	  boost::mutex publish_mutex_;
      bool linear_button_pressed, angular_button_pressed;

      dynamic_reconfigure::Server<teleoperation::HapticTeleopConfig> param_server;
      dynamic_reconfigure::Server<teleoperation::HapticTeleopConfig>::CallbackType param_callback_type;
};

HapticTeleop::HapticTeleop(ros::NodeHandle& n):
  n_(n),
  n_priv_("~"),
  linear_button_pressed(false),
  angular_button_pressed(false)
{
  // Master side params

  // Joint space
  n_priv_.param<double>("master_joint_min_x", master_joint_min_x, 0.0);
  n_priv_.param<double>("master_joint_max_x", master_joint_max_x, 1.77);
  n_priv_.param<double>("master_joint_min_y", master_joint_min_y, 0.0);
  n_priv_.param<double>("master_joint_max_y", master_joint_max_y, 0.0);
  n_priv_.param<double>("master_joint_min_z", master_joint_min_z, -.82);
  n_priv_.param<double>("master_joint_max_z", master_joint_max_z, 1.27);
  n_priv_.param<double>("master_joint_min_yaw", master_joint_min_yaw, -1.013);
  n_priv_.param<double>("master_joint_max_yaw", master_joint_max_yaw, 0.956);
  master_joint_len_x=fabs(master_joint_max_x-master_joint_min_x);
  master_joint_len_y=fabs(master_joint_max_y-master_joint_min_y);
  master_joint_len_z=fabs(master_joint_max_z-master_joint_min_z);
  master_joint_len_yaw=fabs(master_joint_max_yaw-master_joint_min_yaw);

  // Cartesian space
  n_priv_.param<double>("master_min_x", master_min_x, 0.0);
  n_priv_.param<double>("master_max_x", master_max_x, 1.77);
  n_priv_.param<double>("master_min_y", master_min_y, 0.0);
  n_priv_.param<double>("master_max_y", master_max_y, 0.0);
  n_priv_.param<double>("master_min_z", master_min_z, -.82);
  n_priv_.param<double>("master_max_z", master_max_z, 1.27);
  n_priv_.param<double>("master_min_yaw", master_min_yaw, -.82);
  n_priv_.param<double>("master_max_yaw", master_max_yaw, 1.27);
  master_len_x=fabs(master_max_x-master_min_x);
  master_len_y=fabs(master_max_y-master_min_y);
  master_len_z=fabs(master_max_z-master_min_z);
  master_len_yaw=fabs(master_max_yaw-master_min_yaw);

  // Slave side params
  n_priv_.param<double>("slave_velocity_min_x", slave_velocity_min_x, -0.5);
  n_priv_.param<double>("slave_velocity_max_x", slave_velocity_max_x,  0.5);
  n_priv_.param<double>("slave_velocity_min_y", slave_velocity_min_y, -0.5);
  n_priv_.param<double>("slave_velocity_max_y", slave_velocity_max_y,  0.5);
  n_priv_.param<double>("slave_velocity_min_z", slave_velocity_min_z, -0.5);
  n_priv_.param<double>("slave_velocity_max_z", slave_velocity_max_z,  0.5);
  n_priv_.param<double>("slave_velocity_min_yaw", slave_velocity_min_yaw, -0.5);
  n_priv_.param<double>("slave_velocity_max_yaw", slave_velocity_max_yaw,  0.5);
  slave_velocity_len_x=fabs(slave_velocity_max_x-slave_velocity_min_x);
  slave_velocity_len_y=fabs(slave_velocity_max_y-slave_velocity_min_y);
  slave_velocity_len_z=fabs(slave_velocity_max_z-slave_velocity_min_z);
  slave_velocity_len_yaw=fabs(slave_velocity_max_yaw-slave_velocity_min_yaw);


  n_priv_.param<int>("mode", mode_, 1);

  // Master-Slave scaling
  slave_velocity_scale_yaw=slave_velocity_len_yaw/master_joint_len_yaw;
  if(mode_==1)
  {
    ROS_INFO("Master joint mode");
    slave_velocity_scale_x=slave_velocity_len_x/master_joint_len_x;
    slave_velocity_scale_y=slave_velocity_len_y/master_joint_len_y;
    slave_velocity_scale_z=slave_velocity_len_z/master_joint_len_z;
    omni_sub_ = n_.subscribe<sensor_msgs::JointState>("/omni1_joint_states", 2, &HapticTeleop::masterJointsCallbackModeJoints, this);
  }
  else
  {
    ROS_INFO("Master cartesian mode");
    slave_velocity_scale_x=slave_velocity_len_x/master_len_x;
    slave_velocity_scale_y=slave_velocity_len_y/master_len_y;
    slave_velocity_scale_z=slave_velocity_len_z/master_len_z;
    slave_velocity_scale_yaw=slave_velocity_len_yaw/master_len_yaw;
    omni_sub_ = n_.subscribe<sensor_msgs::JointState>("/omni1_joint_states", 2, &HapticTeleop::masterJointsCallbackModeCartesian, this);
  }

  param_callback_type = boost::bind(&HapticTeleop::paramsCallback, this, _1, _2);
  param_server.setCallback(param_callback_type);

  vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  button_sub_ = n_.subscribe<phantom_omni::PhantomButtonEvent>("/omni1_button", 2, &HapticTeleop::slaveButtonCallback, this);
}

void HapticTeleop::slaveButtonCallback(const phantom_omni::PhantomButtonEvent::ConstPtr& button)
{
   if(button->grey_button==1)
     linear_button_pressed=true;
   else
     linear_button_pressed=false;


   if(button->white_button==1)
     angular_button_pressed=true;
   else
     angular_button_pressed=false;
}

// Joints only mode
void HapticTeleop::masterJointsCallbackModeJoints(const sensor_msgs::JointState::ConstPtr& joint_states)
{ 
  geometry_msgs::Twist vel;
  if(linear_button_pressed)
  {
      // Elbow controls linear x speed
      double x_master_joint=joint_states->position[2];
      if(x_master_joint<master_joint_min_x)
      {
          x_master_joint=master_joint_min_x;
      }
      else if(x_master_joint>master_joint_max_x)
      {
          x_master_joint=master_joint_max_x;
      }

      vel.linear.x  = -((x_master_joint-master_joint_min_x)  *slave_velocity_scale_x  +slave_velocity_min_x);

      // Elbow controls linear z speed
      //vel.linear.z = (joint_states->position[2]-master_joint_min_z)*slave_velocity_scale_z+slave_velocity_min_z;
  }
  if(angular_button_pressed)
  {
      // Wrist3 controls angular speed
      double yaw_master_joint=joint_states->position[5];
      if(yaw_master_joint<master_joint_min_yaw)
      {
          yaw_master_joint=master_joint_min_yaw;
      }
      else if(yaw_master_joint>master_joint_max_yaw)
      {
          yaw_master_joint=master_joint_max_yaw;
      }
      vel.angular.z =  ((yaw_master_joint-master_joint_min_yaw)*slave_velocity_scale_yaw+slave_velocity_min_yaw);
  }
  vel_pub_.publish(vel);
}

// Cartesian mode
void HapticTeleop::masterJointsCallbackModeCartesian(const sensor_msgs::JointState::ConstPtr& joint_states)
{
  geometry_msgs::Twist vel;

  if(linear_button_pressed)
  {
      try
      {
          listener.lookupTransform("/base", "/wrist2",ros::Time(0), transform);
      }
          catch (tf::TransformException ex)
      {
              ROS_ERROR("%s",ex.what());
      }

      double x_master=transform.getOrigin().x();
      if(x_master<master_min_x)
      {
          x_master=master_min_x;
      }
      else if(x_master>master_max_x)
      {
          x_master=master_max_x;
      }
      vel.linear.x = -((x_master-master_min_x)*slave_velocity_scale_x+slave_velocity_min_x);

      double y_master=transform.getOrigin().y();
      if(y_master<master_min_y)
      {
          y_master=master_min_y;
      }
      else if(y_master>master_max_y)
      {
          y_master=master_max_y;
      }
      vel.linear.y = -((y_master-master_min_y)*slave_velocity_scale_y+slave_velocity_min_y);

      double z_master=transform.getOrigin().z();
      if(z_master<master_min_z)
      {
          z_master=master_min_z;
      }
      else if(z_master>master_max_z)
      {
          z_master=master_max_z;
      }
      vel.linear.z =  ((z_master-master_min_z)*slave_velocity_scale_z+slave_velocity_min_z);
  }
  if(angular_button_pressed)
  {
      // Wrist3 controls angular speed
      double yaw_master=joint_states->position[5];

      if(yaw_master<master_min_yaw)
      {
          yaw_master=master_min_yaw;
      }
      else if(yaw_master>master_max_yaw)
      {
          yaw_master=master_max_yaw;
      }

      // Wrist3 controls angular speed
      vel.angular.z = ((yaw_master-master_min_yaw)*slave_velocity_scale_yaw+slave_velocity_min_yaw);
  }
  vel_pub_.publish(vel);
}

void HapticTeleop::paramsCallback(teleoperation::HapticTeleopConfig &config, uint32_t level)
{
    ROS_DEBUG_STREAM("Haptic teleoperation reconfigure Request ->"
                     << " master_joint_min_x:" << config.master_joint_min_x
                     << " master_joint_max_x:" << config.master_joint_max_x
                     << " master_joint_min_y:" << config.master_joint_min_y
                     << " master_joint_max_y:" << config.master_joint_max_y
                     << " master_joint_min_z:" << config.master_joint_min_z
                     << " master_joint_max_z:" << config.master_joint_max_z
                     << " master_joint_min_yaw:" << config.master_joint_min_yaw
                     << " master_joint_max_yaw:" << config.master_joint_max_yaw
                     << " slave_velocity_min_x:" << config.slave_velocity_min_x
                     << " slave_velocity_max_x:" << config.slave_velocity_max_x
                     << " slave_velocity_min_y:" << config.slave_velocity_min_y
                     << " slave_velocity_max_y:" << config.slave_velocity_max_y
                     << " slave_velocity_min_z:" << config.slave_velocity_min_z
                     << " slave_velocity_max_z:" << config.slave_velocity_max_z
                     << " slave_velocity_min_yaw:" << config.slave_velocity_min_yaw
                     << " slave_velocity_max_yaw:" << config.slave_velocity_max_yaw
                     );

    master_min_x=config.master_min_x;
    master_max_x=config.master_max_x;
    master_min_y=config.master_min_y;
    master_max_y=config.master_max_y;
    master_min_z=config.master_min_z;
    master_max_z=config.master_max_z;
    master_min_yaw=config.master_min_yaw;
    master_max_yaw=config.master_max_yaw;

    master_joint_min_x=config.master_joint_min_x;
    master_joint_max_x=config.master_joint_max_x;
    master_joint_min_y=config.master_joint_min_y;
    master_joint_max_y=config.master_joint_max_y;
    master_joint_min_z=config.master_joint_min_z;
    master_joint_max_z=config.master_joint_max_z;
    master_joint_min_yaw=config.master_joint_min_yaw;
    master_joint_max_yaw=config.master_joint_max_yaw;

    master_joint_len_x=fabs(master_joint_max_x-master_joint_min_x);
    master_joint_len_y=fabs(master_joint_max_y-master_joint_min_y);
    master_joint_len_z=fabs(master_joint_max_z-master_joint_min_z);
    master_joint_len_yaw=fabs(master_joint_max_yaw-master_joint_min_yaw);

    slave_velocity_min_x=config.slave_velocity_min_x;
    slave_velocity_max_x=config.slave_velocity_max_x;
    slave_velocity_min_y=config.slave_velocity_min_y;
    slave_velocity_max_y=config.slave_velocity_max_y;
    slave_velocity_min_z=config.slave_velocity_min_z;
    slave_velocity_max_z=config.slave_velocity_max_z;
    slave_velocity_min_yaw=config.slave_velocity_min_yaw;
    slave_velocity_max_yaw=config.slave_velocity_max_yaw;

    slave_velocity_len_x=fabs(slave_velocity_max_x-slave_velocity_min_x);
    slave_velocity_len_y=fabs(slave_velocity_max_y-slave_velocity_min_y);
    slave_velocity_len_z=fabs(slave_velocity_max_z-slave_velocity_min_z);
    slave_velocity_len_yaw=fabs(slave_velocity_max_yaw-slave_velocity_min_yaw);

  slave_velocity_scale_yaw=slave_velocity_len_yaw/master_joint_len_yaw;
  // Master-Slave scaling
  if(mode_==1)
  {
    ROS_INFO("Master joint mode");
    slave_velocity_scale_x=slave_velocity_len_x/master_joint_len_x;
    slave_velocity_scale_y=slave_velocity_len_y/master_joint_len_y;
    slave_velocity_scale_z=slave_velocity_len_z/master_joint_len_z;
    omni_sub_ = n_.subscribe<sensor_msgs::JointState>("/omni1_joint_states", 2, &HapticTeleop::masterJointsCallbackModeJoints, this);
  }
  else
  {
    ROS_INFO("Master cartesian mode");
    slave_velocity_scale_x=slave_velocity_len_x/master_len_x;
    slave_velocity_scale_y=slave_velocity_len_y/master_len_y;
    slave_velocity_scale_z=slave_velocity_len_z/master_len_z;
    omni_sub_ = n_.subscribe<sensor_msgs::JointState>("/omni1_joint_states", 2, &HapticTeleop::masterJointsCallbackModeCartesian, this);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "omni_teleop");

  ros::NodeHandle n;

  HapticTeleop haptic_teleop(n);
  ros::spin();
      
  return(0);
}




