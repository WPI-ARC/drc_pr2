
#include <ros/ros.h>
#include <feedback_devices/tacta_box.h>
#include <cereal_port/CerealPort.h>
#include <geometry_msgs/Point.h>

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

#define CHANNELS 16
#define SIZE 100

int active[CHANNELS] = {0};
int output[CHANNELS][SIZE] = {{0}};

//Global Variables for use with the TACTA_Device
cereal::CerealPort device;
char data_amp[6];
char data_freq[6];

//Callback for use with the belts msg
void box_cb (feedback_devices::tacta_box box){

    for (int channel = 0; channel < CHANNELS; channel++){

        if (box.active[channel] == feedback_devices::tacta_box::ACTIVE) {

            active[channel] = feedback_devices::tacta_box::ACTIVE;

            if (box.freq[channel] > 0 && box.freq[channel] < 255){

                int uptime = (int)(100 / (box.freq[channel] * 2));

                int pos = 0;

                //Create the matrix for the rh_rf
                for (int i = 1; i <= box.freq[channel]; i++){
                    for (int j = 0; j < 100 / box.freq[channel]; j++){

                        pos = j + ((100 / box.freq[channel])*(i-1));

                        if (j < uptime) { output[channel][pos] = (int)box.amp_max[channel]; }
                        else output[channel][pos] = (int)box.amp_min[channel];
                    }
                }

            } else if (box.freq[channel] == feedback_devices::tacta_box::FREQ_ON) {

                for (int j = 0; j < 100; j++){
                    output[channel][j] = box.amp_max[channel];
                }

            } else if (box.freq[channel] == feedback_devices::tacta_box::FREQ_OFF) {

                for (int j = 0; j < 100; j++){
                    output[channel][j] = 0;
                }
            }

        } else { active[channel] = feedback_devices::tacta_box::INACTIVE; }

    } //End of each output
} //End of Function

void tacta_output(void) {

    static int loc = 0;

    for (int channel = 0; channel < CHANNELS; channel++){

        if (active[channel] == feedback_devices::tacta_box::ACTIVE) {

            data_amp[0] = TACTA_BELT_ADDRESS;
            data_amp[1] = TACTA_BELT_SET_OUTPUT;
            data_amp[2] = TACTA_BELT_ID;
            data_amp[3] = channel;
            data_amp[4] = output[channel][loc];
            data_amp[5] = data_amp[0] ^ data_amp[1] ^ data_amp[2] ^ data_amp[3] ^ data_amp[4];
            device.write(data_amp, 6);

        }
    }

    loc++;

    if (loc >= 100) { loc = 0; }

}

// Create a ROS publisher and suscriber for the tacta_belt
int main(int argc, char** argv)
{

  ros::init(argc, argv, "tacta_box");
  ros::NodeHandle n;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = n.subscribe ("/feedback_devices/tacta_box/input", 1, box_cb);

  // Change the next line according to your port name and baud rate
  try{ device.open("/dev/ttyUSB0", 19200); }
  catch(cereal::Exception& e)
  {
      ROS_FATAL("Failed to open the serial port!!!");
      ROS_BREAK();
  }
  ROS_INFO("The serial port has been opened.");

  //On Init Enable B-Cast on All Channels
  data_amp[0] = TACTA_BELT_ADDRESS;
  data_amp[1] = TACTA_BELT_ENB_OUTPUT;
  data_amp[2] = TACTA_BELT_BCAST;
  data_amp[3] = TACTA_BELT_BCAST;
  data_amp[4] = data_amp[0] ^ data_amp[1] ^ data_amp[2] ^ data_amp[3];
  device.write(data_amp, 5);

  ros::Rate r(100);

  while(ros::ok()){

      tacta_output();

      ros::spinOnce();

      r.sleep();

  }

  ros::spin();

}

