#include"Inverse_Kinematics_h.h"

int main(int argc, char const *argv[])
{
	inv_kinematics::inv_kinematics robot;
	
	std_msgs::Float32MultiArray results;

	robot.set_param(200,200);
	

	results = robot.joint_positions(282,30,50);
	
	robot.print_param();

	float ye = robot.calculate_height((3.1415/2));
	std::cout<<ye<<"\n";


	return 0;
}