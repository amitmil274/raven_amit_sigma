#ifndef SIGMA_H_
#define SIGMA_H_

#include "/usr/include/dhdc.h"				//AMIT
#include "/usr/include/drdc.h"				//AMIT
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <iomanip>
#include <ros/ros.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <sstream>
#include <pthread.h>
#include <termios.h>
#include <queue>
#include "std_msgs/Float32.h"
#include "haptic_device.h"
using namespace sigma_ros;
using namespace std;

class Sigma
{
	private:
		int PUB_COUNT;
		int SUB_COUNT;
		bool SHOW_STATUS;
		bool RECEIVED_FIRST;
		bool PAUSE;

		pthread_t console_thread;
		pthread_t ros_thread;
		pthread_t gravity_thread;
		pthread_t haptic_thread;
		std_msgs::Float32 force_sensor_data;
		ros::Publisher sigma_publisher;
		ros::Subscriber sigma_subscriber;
		bool msg_new = false;
		bool msg_proc = false;
		double gripForce;
	public:
		Sigma();		// constructor
		void initial(int, char**);	// initialization and console display
		void init_sys();
		bool init_ros(int, char**);
		void init_words();
		bool menu_words(bool);
		void final_words();

		void start_thread();		// thread management
		void join_thread();
		void *console_process(void);
		void *ros_process(void);
		static void *static_console_process(void*);
		static void *static_ros_process(void*);

		void publish_sigma();			 // ROS publish
		void callback_sigma(std_msgs::Float32 msg); // ROS subscribe

		void output_STATUS();		// show ROS and raven state
		void output_PUBinfo();
		void output_SUBinfo();

		int getKey();

		void init_haptic();			//AMIT
		void *gravity_process(void); 	//AMIT
		void *haptic_process(void); 	//AMIT
		static void *static_haptic_process(void*);
		static void *static_gravity_process(void*);
		void publish_haptic_msg(double px, double py, double pz, double gripper);

}; //end of class definition

#endif
