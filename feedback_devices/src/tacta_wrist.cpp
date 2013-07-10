#include <ros/ros.h>
#include <feedback_devices/tacta_wrist.h>
#include <cereal_port/CerealPort.h>

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

//Global Variables for use with the TACTA_Belt
cereal::CerealPort device;
char data[6];



//Callback for use with the belts msg
void wrist_cb (feedback_devices::tacta_wrist wrist){

  ROS_INFO("--------------------");

  //Top
  data[0] = TACTA_BELT_ADDRESS;
  data[1] = TACTA_BELT_SET_OUTPUT;
  data[2] = TACTA_BELT_ID;
  data[3] = 0;
  data[4] = wrist.top;
  data[5] = data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4];
  device.write(data, 6);


  //Bottom
  data[0] = TACTA_BELT_ADDRESS;
  data[1] = TACTA_BELT_SET_OUTPUT;
  data[2] = TACTA_BELT_ID;
  data[3] = 1;
  data[4] = wrist.bottom;
  data[5] = data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4];
  device.write(data, 6);


  //Left
  data[0] = TACTA_BELT_ADDRESS;
  data[1] = TACTA_BELT_SET_OUTPUT;
  data[2] = TACTA_BELT_ID;
  data[3] = 2;
  data[4] = wrist.left;
  data[5] = data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4];
  device.write(data, 6);


  //Right
  data[0] = TACTA_BELT_ADDRESS;
  data[1] = TACTA_BELT_SET_OUTPUT;
  data[2] = TACTA_BELT_ID;
  data[3] = 3;
  data[4] = wrist.right;
  data[5] = data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4];
  device.write(data, 6);


  //Front
  data[0] = TACTA_BELT_ADDRESS;
  data[1] = TACTA_BELT_SET_OUTPUT;
  data[2] = TACTA_BELT_ID;
  data[3] = 4;
  data[4] = wrist.front;
  data[5] = data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4];
  device.write(data, 6);


  //Back
  data[0] = TACTA_BELT_ADDRESS;
  data[1] = TACTA_BELT_SET_OUTPUT;
  data[2] = TACTA_BELT_ID;
  data[3] = 5;
  data[4] = wrist.back;
  data[5] = data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4];
  device.write(data, 6);

}



// Create a ROS publisher and suscriber for the tacta_belt
int main(int argc, char** argv)
{

  ros::init(argc, argv, "tacta_wrist");
  ros::NodeHandle n;
	ros::Publisher state;
  ros::Rate r(TACTA_BELT_STATE_UPDATE);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = n.subscribe ("/feedback_devices/tacta_wrist/input", 1, wrist_cb);
  
  // Change the next line according to your port name and baud rate
  try{ device.open("/dev/ttyUSB0", 19200); }
  catch(cereal::Exception& e)
  {
      ROS_FATAL("Failed to open the serial port!!!");
      ROS_BREAK();
  }
  ROS_INFO("The serial port is opened.");
  
  //On Init Enable B-Cast on All Channels
  data[0] = TACTA_BELT_ADDRESS;
  data[1] = TACTA_BELT_ENB_OUTPUT;
  data[2] = TACTA_BELT_BCAST;
  data[3] = TACTA_BELT_BCAST;
  data[4] = data[0] ^ data[1] ^ data[2] ^ data[3];
  device.write(data, 5);

  ros::spin();

}

