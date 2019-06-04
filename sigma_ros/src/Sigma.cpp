#include "Sigma.h"
#include "/usr/include/dhdc.h"				//AMIT
#include "/usr/include/drdc.h"	

int *ID;
int  Run = 0;
Sigma::Sigma()
{

}

void* Sigma::gravity_process(void)
{
  // retrieve the device index as argument
  //int id = *((int*)arg);
int id =0;
  // try and open requested dgevice
  ID[id] = dhdOpenID (id);
    printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
  if (ID[id] < 0) {
    printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
    Run = -1000;
    return NULL;
  }
 double oldGripForce = 0;
  // report that we are ready
  Run++;

  // enable force
  dhdEnableForce (DHD_ON, ID[id]);

  // haptic loop
while (ros::ok()) {
if(!msg_new)
	{
//	if (dhdSetForceAndTorqueAndGripperForce (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, ID[id]) < DHD_NO_ERROR)
//		{
//		printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
//		Run = 0;
//		}
	}
else
	{
	msg_new = false;
//	msg_proc = true;
	if (dhdSetForceAndTorqueAndGripperForce (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, gripForce*2, ID[id]) < DHD_NO_ERROR);
//		{
//		printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
//		Run = 0;
//		}	
//	msg_proc = false;
//	oldGripForce = gripForce;
	}
   printf ("[%d] %s:  grip (%+0.03f) m  |  freq [%0.02f kHz]\r", id, dhdGetSystemName(ID[0]), gripForce, dhdGetComFreq (ID[id]));
}
// close the connection
dhdClose (ID[id]);
// report that we are done
Run--;
return NULL;
}

void* Sigma::haptic_process(void){
int id=0;
double px=0;
double py=0;
double pz=0;
double gripper=0;
double prevx=0;
double prevy=0;
double prevz=0;
double prevgripper=0;
dhdGetPosition (&px, &py, &pz, ID[id]);
    dhdGetGripperAngleRad(&gripper,ID[id]);
while (ros::ok()) {
prevx=px;
prevy=py;
prevz=pz;
prevgripper=gripper;
    // display some info on the currently selected device
    dhdGetPosition (&px, &py, &pz, ID[id]);
    dhdGetGripperAngleRad(&gripper,ID[id]);
   // printf ("[%d] %s:  p (%+0.03f %+0.03f %+0.03f) m  |  freq [%0.02f kHz]\r", id, dhdGetSystemName(ID[0]), px, py, pz, dhdGetComFreq (ID[id]));
			publish_haptic_msg(px,py,pz,gripper);
    }


  // happily exit
  printf ("\ndone HAPtiC\n");
  return 0;
  dhdClose (ID[id]);
}
void Sigma::callback_sigma(std_msgs::Float32 msg) 
{
	// (1) save the updated raven_state 
	if(!msg_proc)
	{
		msg_new = true;
		gripForce = msg.data;
	}
//	dhdSetForceAndGripperForce(0,0,0,msg.data,0);
//	std::cout<<msg.data<<"\r";
	// (2) update recieved data count
	SUB_COUNT ++;
}
void Sigma::initial(int argc, char** argv)
{
	init_sys();
	if(!init_ros(argc,argv))
  	{
     		ROS_ERROR("Fail to initialize ROS. Exiting!");
		exit(1);
  	}
	init_haptic();
}
void Sigma::init_sys()  
{
	this->PUB_COUNT = 0;
	this->SUB_COUNT = 0;	
}
void Sigma::init_haptic()
{
// globals




int    count;


  // message
  int major, minor, release, revision;
  dhdGetSDKVersion (&major, &minor, &release, &revision);
  printf ("\n");
  printf ("Force Dimension - Multi-threaded Gravity Compensation Example %d.%d.%d.%d\n", major, minor, release, revision);
  printf ("(C) 2001-2018 Force Dimension\n");
  printf ("All Rights Reserved.\n\n");

  // check for devices (limit to 10 devices)
  //count = MIN(10, dhdGetDeviceCount ());
count = dhdGetDeviceCount();
  if (count < 1) {
    printf ("error: no device detected\n");
    dhdSleep (2.0);
    return;
  }

  // allocate resources and start threads
  printf ("starting %d threads\n\n", count);
  ID = new int[count];
  for (int i=0; i<count; i++ ) {
    ID[i] = i;
   // dhdStartThread (GravityThread, &(ID[i]), DHD_THREAD_PRIORITY_HIGH);
  }

  // wait for all threads to start (each thread increments the 'Run' variable by 1 when ready)
  //while (Run >= 0 && Run < count) dhdSleep (0.1);
  if (Run < 0) {
    printf ("error: thread launch failed\n");
    return;
  }

  // identify each device
  for (int i=0;i <count; i++) printf ("[%d] %s device detected\n", i, dhdGetSystemName(ID[i]));
  printf ("\n");

  // display instructions
  printf ("press 'q' to quit\n");
  printf ("      [0..9] to select display device\n\n");

  // UI thread (default priority)
  
}

bool Sigma::init_ros(int argc, char** argv) 
{
	//initialize ROS
	ros::init(argc, argv, "autocircle_generator");

	static ros::NodeHandle n;

	sigma_subscriber = n.subscribe("chatter",1,&Sigma::callback_sigma,this);

	sigma_publisher = n.advertise<haptic_device>("haptic_msg", 1);
	return true;
}
void Sigma::start_thread()
{
	pthread_create(&gravity_thread,NULL,Sigma::static_gravity_process,this);
	pthread_create(&haptic_thread,NULL,Sigma::static_haptic_process,this);

}
void Sigma::join_thread()
{
	pthread_join(gravity_thread,NULL);
	pthread_join(haptic_thread,NULL);
}

void * Sigma::static_gravity_process(void* classRef)
{
	return ((Sigma *)classRef)->gravity_process();
}
void * Sigma::static_haptic_process(void* classRef)
{
	return ((Sigma *)classRef)->haptic_process();
}
void Sigma::publish_haptic_msg(double px,double py,double pz,double gripper)
{
	static ros::Rate loop_rate(1000);
	static haptic_device msg_haptic_msg;	
	// (1) wrap up the new command	
	msg_haptic_msg.position[0] = px;
	msg_haptic_msg.position[1] = py;
	msg_haptic_msg.position[2] = pz;
	msg_haptic_msg.gripper = gripper;

	// (2) send new command
	sigma_publisher.publish(msg_haptic_msg);
	ros::spinOnce();

	//(3) prepare for next publish
	loop_rate.sleep();
	//PUB_COUNT ++;
}

