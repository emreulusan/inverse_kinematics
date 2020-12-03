/*
  impl. file: src/Inverse_Kinematics.cpp
*/
#ifndef INV_KINEMATICS_
#define INV_KINEMATICS_

#include<iostream>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <sensor_msgs/JointState.h>
#include <math.h>

namespace inv_kinematics{

  class inv_kinematics{
  public:

  	enum 
  	{
  		
  	};

    float height_x, y_step, sh_step;
    float l_upper_leg, l_bottom_leg;
    float PI;
    float height_r, height_s;
    float theta1, theta2, theta3;
    float temp_height;

    std_msgs::Float32MultiArray joint_theta;
    
    std_msgs::Float32MultiArray joint_positions(float height_x_t, float y_step_t, float sh_step_t);
    void set_param(float l_upper_leg_t, float l_bottom_leg_t);
    void print_param();
    void find_height();
    float Theta1(float t_theta2);
    float Theta2();
    float calculate_height(float a_degree);
    float Theta3();
    
  };
}

#endif