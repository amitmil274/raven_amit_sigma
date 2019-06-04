#include "Sigma.h"

int main(int argc, char **argv)
{
	
	Sigma sigma;

	// initialize the system
	sigma.initial(argc,argv); 
  	
	// start the console_thread and ros_thread
	sigma.start_thread(); 
	
	// trigger ROS publish and subscribe update
	ros::spin();

	// join the console_thread and ros_thread
	sigma.join_thread();	

  	exit(1);
}
