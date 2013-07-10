
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

#define RH_RF 0
#define RH_LF 1
#define LH_RF 2
#define LH_LF 3

//Global Variables for use with the TACTA_Device
cereal::CerealPort device;
char data_amp[6];
char data_freq[6];

//Right Hand output struct
int rh_rf_output[100];
int rh_lf_output[100];

//Left Hand output struct
int lh_rf_output[100];
int lh_lf_output[100];

//Callback for use with the belts msg
void grippers_cb (feedback_devices::tacta_box grippers){

    ROS_INFO("{  [%d, %d] @ %d    ||    [%d, %d] @ %d  }         {  [%d, %d] @ %d    ||    [%d, %d] @ %d  }", grippers.amp_min[LH_LF], grippers.amp_max[LH_LF], grippers.freq[LH_LF],
                                                                                                              grippers.amp_min[LH_RF], grippers.amp_max[LH_RF], grippers.freq[LH_RF],
                                                                                                              grippers.amp_min[RH_LF], grippers.amp_max[RH_LF], grippers.freq[RH_LF],
                                                                                                              grippers.amp_min[RH_RF], grippers.amp_max[RH_RF], grippers.freq[RH_RF]);

//    //Reset all of the output matrixies for the hands
//    for (int i = 0; i < 100; i++){
//        rh_rf_output[i] = 0;
//        rh_lf_output[i] = 0;
//        lh_rf_output[i] = 0;
//        lh_lf_output[i] = 0;
//    }

    int pos = 0;

    if (grippers.freq[RH_RF] > 0 && grippers.freq[RH_RF] < 255){
        int rh_rf_uptime = (int)(100 / (grippers.freq[RH_RF] * 2));

        //Create the matrix for the rh_rf
        for (int i = 1; i <= grippers.freq[RH_RF]; i++){
            for (int j = 0; j < 100 / grippers.freq[RH_RF]; j++){

                pos = j + ((100 / grippers.freq[RH_RF])*(i-1));

                if (j < rh_rf_uptime) { rh_rf_output[pos] = grippers.amp_max[RH_RF]; }
                else rh_rf_output[pos] = grippers.amp_min[RH_RF];
            }
        }
    } else if (grippers.freq[RH_RF] == feedback_devices::tacta_box::FREQ_ON) {
        for (int j = 0; j < 100; j++){
            rh_rf_output[j] = grippers.amp_max[RH_RF];
        }

    } else if (grippers.freq[RH_RF] == feedback_devices::tacta_box::FREQ_OFF) {
        for (int j = 0; j < 100; j++){
            rh_rf_output[j] = 0;
        }
    }

    if (grippers.freq[RH_LF] > 0 && grippers.freq[RH_LF] < 255){
        int rh_lf_uptime = (int)(100 / (grippers.freq[RH_LF] * 2));

        //Create the matrix for the rh_lf
        for (int i = 1; i <= grippers.freq[RH_LF]; i++){
            for (int j = 0; j < 100 / grippers.freq[RH_LF]; j++){

                pos = j + ((100 / grippers.freq[RH_LF])*(i-1));

                if (j < rh_lf_uptime) { rh_lf_output[pos] = grippers.amp_max[RH_LF]; }
                else rh_lf_output[pos] = grippers.amp_min[RH_LF];
            }
        }
    } else if (grippers.freq[RH_LF] == feedback_devices::tacta_box::FREQ_ON) {
        for (int j = 0; j < 100; j++){
            rh_lf_output[j] = grippers.amp_max[RH_LF];
        }

    } else if (grippers.freq[RH_LF] == feedback_devices::tacta_box::FREQ_OFF) {
        for (int j = 0; j < 100; j++){
            rh_lf_output[j] = 0;
        }
    }

    if (grippers.freq[LH_RF] > 0 && grippers.freq[LH_RF] < 255){
        int lh_rf_uptime = (int)(100 / (grippers.freq[LH_RF] * 2));

        //Create the matrix for the lh_rf
        for (int i = 1; i <= grippers.freq[LH_RF]; i++){
            for (int j = 0; j < 100 / grippers.freq[LH_RF]; j++){

                pos = j + ((100 / grippers.freq[LH_RF])*(i-1));

                if (j < lh_rf_uptime) { lh_rf_output[pos] = grippers.amp_max[LH_RF]; }
                else lh_rf_output[pos] = grippers.amp_min[LH_RF];
            }
        }
    } else if (grippers.freq[LH_RF] == feedback_devices::tacta_box::FREQ_ON) {
        for (int j = 0; j < 100; j++){
            lh_rf_output[j] = grippers.amp_max[LH_RF];
        }

    } else if (grippers.freq[LH_RF] == feedback_devices::tacta_box::FREQ_OFF) {
        for (int j = 0; j < 100; j++){
            lh_rf_output[j] = 0;
        }
    }

    if (grippers.freq[LH_LF] > 0 && grippers.freq[LH_LF] < 255){
        int lh_lf_uptime = (int)(100 / (grippers.freq[LH_LF] * 2));

        //Create the matrix for the lh_lf
        for (int i = 1; i <= grippers.freq[LH_LF]; i++){
            for (int j = 0; j < 100 / grippers.freq[LH_LF]; j++){

                pos = j + ((100 / grippers.freq[LH_LF])*(i-1));

                if (j < lh_lf_uptime) { lh_lf_output[pos] = grippers.amp_max[LH_LF]; }
                else lh_lf_output[pos] = grippers.amp_min[LH_LF];
            }
        }
    } else if (grippers.freq[LH_LF] == feedback_devices::tacta_box::FREQ_ON) {
        for (int j = 0; j < 100; j++){
            lh_lf_output[j] = grippers.amp_max[LH_LF];
        }

    } else if (grippers.freq[LH_LF] == feedback_devices::tacta_box::FREQ_OFF) {
        for (int j = 0; j < 100; j++){
            lh_lf_output[j] = 0;
        }
    }
}


int loc = 0;

void tacta_output(void) {

//    int lf_lf = lh_lf_output[loc];
//    int lf_rf = lh_rf_output[loc];
//    int rf_lf = rh_lf_output[loc];
//    int rf_rf = rh_rf_output[loc];


//    ROS_WARN("SENDING [%d, %d, %d, %d]", lf_lf, lf_rf, rf_lf, rf_rf);

    data_amp[0] = TACTA_BELT_ADDRESS;
    data_amp[1] = TACTA_BELT_SET_OUTPUT;
    data_amp[2] = TACTA_BELT_ID;
    data_amp[3] = RH_RF;
    data_amp[4] = rh_rf_output[loc];
    data_amp[5] = data_amp[0] ^ data_amp[1] ^ data_amp[2] ^ data_amp[3] ^ data_amp[4];
    device.write(data_amp, 6);

    data_amp[0] = TACTA_BELT_ADDRESS;
    data_amp[1] = TACTA_BELT_SET_OUTPUT;
    data_amp[2] = TACTA_BELT_ID;
    data_amp[3] = RH_LF;
    data_amp[4] = rh_lf_output[loc];
    data_amp[5] = data_amp[0] ^ data_amp[1] ^ data_amp[2] ^ data_amp[3] ^ data_amp[4];
    device.write(data_amp, 6);


    data_amp[0] = TACTA_BELT_ADDRESS;
    data_amp[1] = TACTA_BELT_SET_OUTPUT;
    data_amp[2] = TACTA_BELT_ID;
    data_amp[3] = LH_RF;
    data_amp[4] = lh_rf_output[loc];
    data_amp[5] = data_amp[0] ^ data_amp[1] ^ data_amp[2] ^ data_amp[3] ^ data_amp[4];
    device.write(data_amp, 6);

    data_amp[0] = TACTA_BELT_ADDRESS;
    data_amp[1] = TACTA_BELT_SET_OUTPUT;
    data_amp[2] = TACTA_BELT_ID;
    data_amp[3] = LH_LF;
    data_amp[4] = lh_lf_output[loc];
    data_amp[5] = data_amp[0] ^ data_amp[1] ^ data_amp[2] ^ data_amp[3] ^ data_amp[4];
    device.write(data_amp, 6);

    loc++;

    if (loc >= 100) { loc = 0; }

}

// Create a ROS publisher and suscriber for the tacta_belt
int main(int argc, char** argv)
{

  ros::init(argc, argv, "tacta_fingers");
  ros::NodeHandle n;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = n.subscribe ("/feedback_devices/tacta_grippers/input", 1, grippers_cb);

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

  for (int i = 0; i < 100; i++){
      //Right Hand output struct
      rh_rf_output[i] = 0;
      rh_lf_output[i] = 0;

      //Left Hand output struct
      lh_rf_output[i] = 0;
      lh_lf_output[i] = 0;
  }

  while(ros::ok()){

      tacta_output();

      ros::spinOnce();

      r.sleep();

  }

  ros::spin();

}

