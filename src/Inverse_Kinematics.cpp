/*
github.com/emreulusan
*/

#include"Inverse_Kinematics_h.h"

namespace inv_kinematics{

std_msgs::Float32MultiArray inv_kinematics::joint_positions(float height_x_t, float y_step_t, float sh_step_t){
	/*
	data[0] -->joint1
	data[1] -->joint2
	data[2] -->shoulder joint*/
	
	joint_theta.data.clear();
	
	height_x = height_x_t;
	y_step = y_step_t;
	sh_step = sh_step_t;
	
	find_height();
	Theta2();
	Theta1(theta2);
	Theta3();

	joint_theta.data.push_back(theta1);
	joint_theta.data.push_back(theta2);
	joint_theta.data.push_back(theta3);

	return joint_theta;
}

void inv_kinematics::set_param(float l_upper_leg_t, float l_bottom_leg_t){
	
	l_bottom_leg=l_bottom_leg_t;
	l_upper_leg=l_upper_leg_t;
	PI=3.141592;		
}

void inv_kinematics::print_param(){
	
	std::cout<<"l_bottom_leg:	"<<l_bottom_leg<<"\n";
	std::cout<<"l_upper_leg:	"<<l_upper_leg<<"\n";
	std::cout<<"height_x:	"<<height_x<<"\n";
	std::cout<<"height_r:	"<<height_r<<"\n";
	std::cout<<"height_s:	"<<height_s<<"\n";
	std::cout<<"joints:		"<<joint_theta;
}

void inv_kinematics::find_height(){

	height_r = sqrt((pow(height_x,2) + pow(y_step,2)));
	height_s = sqrt((pow(height_r,2) + pow(sh_step,2)));
}

float inv_kinematics::Theta1(float t_theta2){

	theta1= atan(y_step/height_s) - atan((l_bottom_leg*sin(t_theta2))
								/(l_upper_leg+l_bottom_leg*cos(t_theta2)));

	return theta1;
}

float inv_kinematics::Theta2(){

	theta2= acos((pow(height_s,2)+pow(y_step,2)-pow(l_upper_leg,2)-pow(l_bottom_leg,2))
								/(2*l_upper_leg*l_bottom_leg));

	return theta2;
}

float inv_kinematics::calculate_height(float a_degree){

	temp_height = pow(l_upper_leg,2) + pow(l_bottom_leg,2) - (2*l_bottom_leg*l_upper_leg*cos(a_degree));
	temp_height = sqrt(temp_height);

	return temp_height;
}

float inv_kinematics::Theta3(){
	
	theta3 = atan(sh_step/height_r);

	return theta3;
}

}//namespace
