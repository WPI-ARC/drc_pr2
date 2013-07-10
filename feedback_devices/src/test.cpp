#include <ros/ros.h>
#include <feedback_devices/tacta_belt.h>

#define REPLY_SIZE 8
#define TIMEOUT 1000

#define TACTA_BELT_ADDRESS 0x2A
#define TACTA_BELT_ID 0x00;
#define TACTA_BELT_MAX 0xFF;
#define TACTA_BELT_SET_OUTPUT 0x56
#define TACTA_BELT_QRY_OUTPUT 0x76
#define TACTA_BELT_ENB_OUTPUT 0x50
#define TACTA_BELT_DSB_OUTPUT 0x70
#define TACTA_BELT_BCAST 0xFF

#define TACTA_BELT_STATE_UPDATE 1
#define TACTA_BELT_MAX_TRY 3



// Create a ROS publisher and suscriber for the tacta_belt
int main(int argc, char** argv)
{

  ros::init(argc, argv, "tacta_test");
  ros::NodeHandle n;
	ros::Publisher pub;
  ros::Rate r(10);

  //Advertise the two publishers, one for the commands and one for the gui
	pub = n.advertise<feedback_devices::tacta_belt>("/feedback_devices/tacta_belt/input", 1);
  
  feedback_devices::tacta_belt myBelt;
  myBelt.motors = 16;
  int index = 0;

  index = 8;

  myBelt.values[0] = 0;
  myBelt.values[1] = 0;
  myBelt.values[2] = 0;
  myBelt.values[3] = 0;
  myBelt.values[4] = 0;
  myBelt.values[5] = 0;
  myBelt.values[6] = 0;
  myBelt.values[7] = 0;

  while(ros::ok())
  {
  
    ROS_INFO("Sending New Msg");
    for(int i = 8; i < myBelt.motors; i++){

      myBelt.values[i] = 0;

      if (i == index){
        myBelt.values[i] = 1;
      }      

    }

    index++;

    if (index >= myBelt.motors){
      index = 8;
    }
  
    pub.publish(myBelt);

    ros::spinOnce();
    r.sleep();
  }   
}
