#include "ros/ros.h"
//#include "feedback_devices/tacta_pr2_grippers.h"
#include "feedback_devices/tacta_box.h"
#include "pr2_controllers_msgs/JointControllerState.h"
#include "pr2_msgs/PressureState.h"
#include <iostream>
#include <fstream>




#define RH_RF 0
#define RH_LF 1
#define LH_RF 2
#define LH_LF 3

void printPressureLog(int hand);
int derivative_magic(int hand, int finger);

ros::Publisher pub_tacta_grippers;
//feedback_devices::tacta_pr2_grippers grippers;
feedback_devices::tacta_box grippers;

enum hand {RIGHT, LEFT};

enum output_mode {ACTIVE, PASSIVE};
int currentMode = ACTIVE;

void r_gripper_state_cb(pr2_controllers_msgs::JointControllerState input){

    if (currentMode == ACTIVE) {

    } else if (currentMode == PASSIVE) {

        if (input.process_value >= 0 && input.process_value < .016){
            grippers.freq[RH_RF] = feedback_devices::tacta_box::FREQ_5HZ;
            grippers.freq[RH_LF] = feedback_devices::tacta_box::FREQ_5HZ;
        } else if (input.process_value >= .016 && input.process_value < .032){
            grippers.freq[RH_RF] = feedback_devices::tacta_box::FREQ_4HZ;
            grippers.freq[RH_LF] = feedback_devices::tacta_box::FREQ_4HZ;
        } else if (input.process_value >= .032 && input.process_value < .048){
            grippers.freq[RH_RF] = feedback_devices::tacta_box::FREQ_3HZ;
            grippers.freq[RH_LF] = feedback_devices::tacta_box::FREQ_3HZ;
        } else if (input.process_value >= .048 && input.process_value < .064){
            grippers.freq[RH_RF] = feedback_devices::tacta_box::FREQ_2HZ;
            grippers.freq[RH_LF] = feedback_devices::tacta_box::FREQ_2HZ;
        } else if (input.process_value >= .064 && input.process_value < .080){
            grippers.freq[RH_RF] = feedback_devices::tacta_box::FREQ_1HZ;
            grippers.freq[RH_LF] = feedback_devices::tacta_box::FREQ_1HZ;
        } else if (input.process_value >= .08) {
            grippers.freq[RH_RF] = feedback_devices::tacta_box::FREQ_OFF;
            grippers.freq[RH_LF] = feedback_devices::tacta_box::FREQ_OFF;
        }

        grippers.amp_max[RH_RF] = feedback_devices::tacta_box::AMP_FULL;
        grippers.amp_max[RH_LF] = feedback_devices::tacta_box::AMP_FULL;

        grippers.amp_min[RH_RF] = feedback_devices::tacta_box::AMP_MED;
        grippers.amp_min[RH_LF] = feedback_devices::tacta_box::AMP_MED;

    }

}

void l_gripper_state_cb(pr2_controllers_msgs::JointControllerState input){

    if (currentMode == ACTIVE) {

    } else if (currentMode == PASSIVE) {

        if (input.process_value >= 0 && input.process_value < .016){
            grippers.freq[LH_RF] = feedback_devices::tacta_box::FREQ_5HZ;
            grippers.freq[LH_LF] = feedback_devices::tacta_box::FREQ_5HZ;
        } else if (input.process_value >= .016 && input.process_value < .032){
            grippers.freq[LH_RF] = feedback_devices::tacta_box::FREQ_4HZ;
            grippers.freq[LH_LF] = feedback_devices::tacta_box::FREQ_4HZ;
        } else if (input.process_value >= .032 && input.process_value < .048){
            grippers.freq[LH_RF] = feedback_devices::tacta_box::FREQ_3HZ;
            grippers.freq[LH_LF] = feedback_devices::tacta_box::FREQ_3HZ;
        } else if (input.process_value >= .048 && input.process_value < .064){
            grippers.freq[LH_RF] = feedback_devices::tacta_box::FREQ_2HZ;
            grippers.freq[LH_LF] = feedback_devices::tacta_box::FREQ_2HZ;
        } else if (input.process_value >= .064 && input.process_value < .080){
            grippers.freq[LH_RF] = feedback_devices::tacta_box::FREQ_1HZ;
            grippers.freq[LH_LF] = feedback_devices::tacta_box::FREQ_1HZ;
        } else if (input.process_value >= .08) {
            grippers.freq[LH_RF] = feedback_devices::tacta_box::FREQ_OFF;
            grippers.freq[LH_LF] = feedback_devices::tacta_box::FREQ_OFF;
        }

        grippers.amp_max[LH_RF] = feedback_devices::tacta_box::AMP_FULL;
        grippers.amp_max[LH_LF] = feedback_devices::tacta_box::AMP_FULL;

        grippers.amp_min[LH_RF] = feedback_devices::tacta_box::AMP_MED;
        grippers.amp_min[LH_LF] = feedback_devices::tacta_box::AMP_MED;

    }

}

int previous_grip_quality[6];
int grip_quality[6];

#define SENSORS 22
#define AMP_THRESHOLD 2000
#define FREQ_THRESHOLD 75
#define MOVE_AVERAGE_SIZE 5

//Variables for performing calibration
bool rh_calibrated = false;
int rh_calibration_time = 2; //Seconds

//rh_pointers for the moving buffers that will house the data
int rh_pointer = 0;


//Variables for performing calibration on the hands / fingers
int rh_calibration_count = 0;
int rh_rf_calibration_sum[SENSORS] = { 0 };
int rh_lf_calibration_sum[SENSORS] = { 0 };
int rh_rf_calibration[SENSORS] = { 0 };
int rh_lf_calibration[SENSORS] = { 0 };

//The moving average
int rh_rf_moving_average[SENSORS][MOVE_AVERAGE_SIZE];
int rh_lf_moving_average[SENSORS][MOVE_AVERAGE_SIZE];

//The current value of the sensors so that the dirivative can be found
int rh_rf_current[SENSORS] = { 0 };
int rh_lf_current[SENSORS] = { 0 };

int rh_rf_prev_current[SENSORS] = { 0 };
int rh_lf_prev_current[SENSORS] = { 0 };

int rh_rf_deriv[SENSORS] = { 0 };
int rh_lf_deriv[SENSORS] = { 0 };

void r_gripper_pressure_cb(pr2_msgs::PressureState input){

    printPressureLog(RIGHT);

    //If the gripper has been rh_calibrated then this code can just be run normally
    if (rh_calibrated) {

        //Keep track of the moving average for both the left and right fingertip
        for (int i = 0; i < SENSORS; i++){
            rh_rf_moving_average[i][rh_pointer] = input.r_finger_tip[i] - rh_rf_calibration[i];
            rh_lf_moving_average[i][rh_pointer] = input.l_finger_tip[i] - rh_lf_calibration[i];
        }
        rh_pointer++;
        if (rh_pointer >= MOVE_AVERAGE_SIZE) rh_pointer = 0;

        //Determine the current value for each of the sensors based off the moving average
        for (int i = 0; i < SENSORS; i++){

            //Sum the current state of the moving average
            for (int j = 0; j < MOVE_AVERAGE_SIZE; j++){
                rh_rf_current[i] += rh_rf_moving_average[i][j];
                rh_lf_current[i] += rh_lf_moving_average[i][j];
            }

            //Find the average or the "actual" current value
            rh_rf_current[i] = rh_rf_current[i] / MOVE_AVERAGE_SIZE;
            rh_lf_current[i] = rh_lf_current[i] / MOVE_AVERAGE_SIZE;
        }

        //Determine what the derivatives are
        for (int i = 0; i < SENSORS; i++){
            rh_rf_deriv[i] = rh_rf_current[i] - rh_rf_prev_current[i];
            rh_lf_deriv[i] = rh_lf_current[i] - rh_lf_prev_current[i];
        }

        //Generate output based off of ACTIVE mode
        if (currentMode == ACTIVE) {

            int rf_max_val = 0;
            int lf_max_val = 0;

            //RIGHT-----------------------------------------------------------------------------------

                    //Find the max value of the current output
                    for (int i = 0; i < SENSORS; i++) {
                        if (rh_rf_current[i] > rf_max_val){
                            rf_max_val = rh_rf_current[i];
                        }
                    }

                    //Make sure that the max val is above the noise threshold
                    if (rf_max_val > AMP_THRESHOLD) {

                        if (rf_max_val > 0 && rf_max_val <= 4000) grippers.amp_max[RH_RF] = 77;
                        if (rf_max_val > 4001 && rf_max_val <= 6000) grippers.amp_max[RH_RF] = 153;
                        if (rf_max_val > 8001 && rf_max_val <= 12000) grippers.amp_max[RH_RF] = 255;

                        //Try and figure out if something is slipping
                        int rf_result = derivative_magic(RIGHT, RIGHT);

                        //We're not slipping
                        if (rf_result == 1) {

                            grippers.freq[RH_RF] = feedback_devices::tacta_box::FREQ_ON;

                        //We are slipping
                        } else if (rf_result == -1) {

                            grippers.freq[RH_RF] = feedback_devices::tacta_box::FREQ_5HZ;

                        //No change
                        } else if (rf_result == 0) {

                            grippers.freq[RH_RF] = feedback_devices::tacta_box::FREQ_ON;

                        }

                    } else {
                        grippers.freq[RH_RF] = feedback_devices::tacta_box::FREQ_OFF;
                    }

            //LEFT-----------------------------------------------------------------------------------

                    //Find the max value of the current output
                    for (int i = 0; i < SENSORS; i++) {
                        if (rh_lf_current[i] > lf_max_val){
                            lf_max_val = rh_lf_current[i];
                        }
                    }

                    //Make sure that the max val is above the noise threshold
                    if (lf_max_val > AMP_THRESHOLD) {

                        if (lf_max_val > 0 && lf_max_val <= 4000) grippers.amp_max[RH_LF] = 77;
                        if (lf_max_val > 4001 && lf_max_val <= 6000) grippers.amp_max[RH_LF] = 153;
                        if (lf_max_val > 8001 && lf_max_val <= 12000) grippers.amp_max[RH_LF] = 255;


                        //Try and figure out if something is slipping
                        int lf_result = derivative_magic(RIGHT, LEFT);

                        //We're not slipping
                        if (lf_result == 1) {

                            grippers.freq[RH_LF] = feedback_devices::tacta_box::FREQ_ON;

                        //We are slipping
                        } else if (lf_result == -1) {

                            grippers.freq[RH_LF] = feedback_devices::tacta_box::FREQ_5HZ;

                        //No change
                        } else if (lf_result == 0) {

                            grippers.freq[RH_LF] = feedback_devices::tacta_box::FREQ_ON;

                        }

                    } else {
                        grippers.freq[RH_LF] = feedback_devices::tacta_box::FREQ_OFF;
                    }

        //Generate output based off of PASSIVE mode
        } else if (currentMode == PASSIVE){

        }


        //Save the current state as the previous current state
        for (int i = 0; i < SENSORS; i++){
            rh_rf_prev_current[i] = rh_rf_current[i];
            rh_lf_prev_current[i] = rh_lf_current[i];
        }


    //If the gripper has not been rh_calibrated then the values must be zero'd
    } else {

        // We're calibrating by taking the average of the sensor data, this will sum them
        for (int i = 0; i < SENSORS; i++){
            rh_rf_calibration_sum[i] += input.r_finger_tip[i];
            rh_lf_calibration_sum[i] += input.l_finger_tip[i];
        }

        rh_calibration_count++;

        if ( rh_calibration_count >= rh_calibration_time * 10 ){
            for (int i = 0; i < SENSORS; i++){
                rh_rf_calibration[i] = rh_rf_calibration_sum[i] / rh_calibration_count;
                rh_lf_calibration[i] = rh_lf_calibration_sum[i] / rh_calibration_count;
            }
            rh_calibrated = true;
        }
    }

}


//Variables for performing calibration
bool lh_calibrated = false;
int lh_calibration_time = 1; //Seconds

//lh_pointers for the moving buffers that will house the data
int lh_pointer = 0;


//Variables for performing calibration on the hands / fingers
int lh_calibration_count = 0;
int lh_rf_calibration_sum[SENSORS] = { 0 };
int lh_lf_calibration_sum[SENSORS] = { 0 };
int lh_rf_calibration[SENSORS] = { 0 };
int lh_lf_calibration[SENSORS] = { 0 };

//The moving average
int lh_rf_moving_average[SENSORS][MOVE_AVERAGE_SIZE];
int lh_lf_moving_average[SENSORS][MOVE_AVERAGE_SIZE];

//The current value of the sensors so that the dirivative can be found
int lh_rf_current[SENSORS] = { 0 };
int lh_lf_current[SENSORS] = { 0 };

int lh_rf_prev_current[SENSORS] = { 0 };
int lh_lf_prev_current[SENSORS] = { 0 };

int lh_rf_deriv[SENSORS] = { 0 };
int lh_lf_deriv[SENSORS] = { 0 };

void l_gripper_pressure_cb(pr2_msgs::PressureState input){

    printPressureLog(LEFT);

    //If the gripper has been lh_calibrated then this code can just be run normally
    if (lh_calibrated) {

        //Keep track of the moving average for both the left and right fingertip
        for (int i = 0; i < SENSORS; i++){
            lh_rf_moving_average[i][lh_pointer] = input.r_finger_tip[i] - lh_rf_calibration[i];
            lh_lf_moving_average[i][lh_pointer] = input.l_finger_tip[i] - lh_lf_calibration[i];
        }
        lh_pointer++;
        if (lh_pointer >= MOVE_AVERAGE_SIZE) lh_pointer = 0;

        //Determine the current value for each of the sensors based off the moving average
        for (int i = 0; i < SENSORS; i++){

            //Sum the current state of the moving average
            for (int j = 0; j < MOVE_AVERAGE_SIZE; j++){
                lh_rf_current[i] += lh_rf_moving_average[i][j];
                lh_lf_current[i] += lh_lf_moving_average[i][j];
            }

            //Find the average or the "actual" current value
            lh_rf_current[i] = lh_rf_current[i] / MOVE_AVERAGE_SIZE;
            lh_lf_current[i] = lh_lf_current[i] / MOVE_AVERAGE_SIZE;
        }

        //Determine what the derivatives are
        for (int i = 0; i < SENSORS; i++){
            lh_rf_deriv[i] = lh_rf_current[i] - lh_rf_prev_current[i];
            lh_lf_deriv[i] = lh_lf_current[i] - lh_lf_prev_current[i];
        }

        //Generate output based off of ACTIVE mode
        if (currentMode == ACTIVE) {

            int rf_max_val = 0;
            int lf_max_val = 0;

            //RIGHT-----------------------------------------------------------------------------------

                    //Find the max value of the current output
                    for (int i = 0; i < SENSORS; i++) {
                        if (lh_rf_current[i] > rf_max_val){
                            rf_max_val = lh_rf_current[i];
                        }
                    }

                    //Make sure that the max val is above the noise threshold
                    if (rf_max_val > AMP_THRESHOLD) {

                        if (rf_max_val > 0 && rf_max_val <= 4000) grippers.amp_max[LH_RF] = 77;
                        if (rf_max_val > 4001 && rf_max_val <= 6000) grippers.amp_max[LH_RF] = 153;
                        if (rf_max_val > 8001 && rf_max_val <= 12000) grippers.amp_max[LH_RF] = 255;

                        //Try and figure out if something is slipping
                        int rf_result = derivative_magic(LEFT, RIGHT);

                        //We're not slipping
                        if (rf_result == 1) {

                            grippers.freq[LH_RF] = feedback_devices::tacta_box::FREQ_ON;

                        //We are slipping
                        } else if (rf_result == -1) {

                            grippers.freq[LH_RF] = feedback_devices::tacta_box::FREQ_5HZ;

                        //No change
                        } else if (rf_result == 0) {

                            grippers.freq[LH_RF] = feedback_devices::tacta_box::FREQ_ON;

                        }

                    } else {
                        grippers.freq[LH_RF] = feedback_devices::tacta_box::FREQ_OFF;
                    }

            //LEFT-----------------------------------------------------------------------------------

                    //Find the max value of the current output
                    for (int i = 0; i < SENSORS; i++) {
                        if (lh_lf_current[i] > lf_max_val){
                            lf_max_val = lh_lf_current[i];
                        }
                    }

                    //Make sure that the max val is above the noise threshold
                    if (lf_max_val > AMP_THRESHOLD) {

                        if (lf_max_val > 0 && lf_max_val <= 4000) grippers.amp_max[LH_LF] = 77;
                        if (lf_max_val > 4001 && lf_max_val <= 6000) grippers.amp_max[LH_LF] = 153;
                        if (lf_max_val > 8001 && lf_max_val <= 12000) grippers.amp_max[LH_LF] = 255;

                        //Try and figure out if something is slipping
                        int lf_result = derivative_magic(LEFT, LEFT);

                        //We're not slipping
                        if (lf_result == 1) {

                            grippers.freq[LH_LF] = feedback_devices::tacta_box::FREQ_ON;

                        //We are slipping
                        } else if (lf_result == -1) {

                            grippers.freq[LH_LF] = feedback_devices::tacta_box::FREQ_5HZ;

                        //No change
                        } else if (lf_result == 0) {

                            grippers.freq[LH_LF] = feedback_devices::tacta_box::FREQ_ON;

                        }

                    } else {
                        grippers.freq[LH_LF] = feedback_devices::tacta_box::FREQ_OFF;
                    }

        //Generate output based off of PASSIVE mode
        } else if (currentMode == PASSIVE){

        }


        //Save the current state as the previous current state
        for (int i = 0; i < SENSORS; i++){
            lh_rf_prev_current[i] = lh_rf_current[i];
            lh_lf_prev_current[i] = lh_lf_current[i];
        }


    //If the gripper has not been lh_calibrated then the values must be zero'd
    } else {

        // We're calibrating by taking the average of the sensor data, this will sum them
        for (int i = 0; i < SENSORS; i++){
            lh_rf_calibration_sum[i] += input.r_finger_tip[i];
            lh_lf_calibration_sum[i] += input.l_finger_tip[i];
        }

        lh_calibration_count++;

        if ( lh_calibration_count >= lh_calibration_time * 10 ){
            for (int i = 0; i < SENSORS; i++){
                lh_rf_calibration[i] = lh_rf_calibration_sum[i] / lh_calibration_count;
                lh_lf_calibration[i] = lh_lf_calibration_sum[i] / lh_calibration_count;
            }
            lh_calibrated = true;
        }
    }

}

int derivative_magic(int hand, int finger){

    int min = 30000, max = -30000;

    //If we are looking for the right hand
    if (hand == RIGHT){

        //Right finger on the right hand
        if (finger == RIGHT){

            //Look through the derivative for each hand
            for (int i = 0; i < SENSORS; i++){
                if (rh_rf_deriv[i] < min) min = rh_rf_deriv[i];
                if (rh_rf_deriv[i] > max) max = rh_rf_deriv[i];
            }

        //Left finger on the right hand
        } else if (finger == LEFT){

            //Look through the derivative for each hand
            for (int i = 0; i < SENSORS; i++){
                if (rh_lf_deriv[i] < min) min = rh_lf_deriv[i];
                if (rh_lf_deriv[i] > max) max = rh_lf_deriv[i];
            }

        }

    //If we are looking for the left hand
    } else if (hand == LEFT) {

        //Right finger on the left hand
        if (finger == RIGHT){

            //Look through the derivative for each hand
            for (int i = 0; i < SENSORS; i++){
                if (lh_rf_deriv[i] < min) min = lh_rf_deriv[i];
                if (lh_rf_deriv[i] > max) max = lh_rf_deriv[i];
            }

        //Left finger on the left hand
        } else if (finger == LEFT){

            //Look through the derivative for each hand
            for (int i = 0; i < SENSORS; i++){
                if (lh_lf_deriv[i] < min) min = lh_lf_deriv[i];
                if (lh_lf_deriv[i] > max) max = lh_lf_deriv[i];
            }

        }

    }

    //Determine the output

    //If both of the numbers are within the noise range don't do anything
    if (abs(min) < FREQ_THRESHOLD && abs(max) < FREQ_THRESHOLD) return 0;

    //If the numbers have different signs, return that we're slipping
    if ((min < 0 && abs(min) > FREQ_THRESHOLD) && (max > 0 && abs(max) > FREQ_THRESHOLD)) return -1;

    //Else return that we're not slipping
    return 1;

}

std::ofstream rh_rf_data, rh_lf_data, rh_rf_der, rh_lf_der;
std::ofstream lh_rf_data, lh_lf_data, lh_rf_der, lh_lf_der;
int rh_count = 0, lh_count = 0;
void printPressureLog(int hand){

    if (hand == RIGHT){

        if (rh_count == 0){
            rh_rf_data.open ("rh_rf_pressure.csv");
            rh_lf_data.open ("rh_lf_pressure.csv");
            rh_rf_der.open("rh_rf_derivative.csv");
            rh_lf_der.open("rh_lf_derivative.csv");
        }

        for (int i = 0; i < SENSORS; i++){

            rh_rf_data << rh_rf_current[i] << ",";
            rh_lf_data << rh_lf_current[i] << ",";
            rh_rf_der << rh_rf_deriv[i] << ",";
            rh_lf_der << rh_lf_deriv[i] << ",";

        }

        rh_count++;
        rh_rf_data << "\n";
        rh_lf_data << "\n";
        rh_rf_der << "\n";
        rh_lf_der << "\n";

    } else {

        if (lh_count == 0){
            lh_rf_data.open ("lh_rf_pressure.csv");
            lh_lf_data.open ("lh_lf_pressure.csv");
            lh_rf_der.open("lh_rf_derivative.csv");
            lh_lf_der.open("lh_lf_derivative.csv");
        }

        for (int i = 0; i < SENSORS; i++){

            lh_rf_data << lh_rf_current[i] << ",";
            lh_lf_data << lh_lf_current[i] << ",";
            lh_rf_der << lh_rf_deriv[i] << ",";
            lh_lf_der << lh_lf_deriv[i] << ",";

        }

        lh_count++;
        lh_rf_data << "\n";
        lh_lf_data << "\n";
        lh_rf_der << "\n";
        lh_lf_der << "\n";


    }
}

void send_feedback(){
    pub_tacta_grippers.publish(grippers);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "proximity");

  //Setup everything for the publishes
  ros::NodeHandle nh;

  //Advertise the two publishers, one for the commands and one for the gui
  pub_tacta_grippers = nh.advertise<feedback_devices::tacta_box>("/feedback_devices/tacta_grippers/input", 1);

  //Create all the subscribers that are needed
  ros::Subscriber sub_r_gripper_state = nh.subscribe("/r_gripper_controller/state",1, r_gripper_state_cb);
  ros::Subscriber sub_l_gripper_state = nh.subscribe("/l_gripper_controller/state",1, l_gripper_state_cb);
  ros::Subscriber sub_r_gripper_pressure = nh.subscribe("/pressure/r_gripper_motor", 1, r_gripper_pressure_cb);
  ros::Subscriber sub_l_gripper_pressure = nh.subscribe("/pressure/l_gripper_motor", 1, l_gripper_pressure_cb);

  grippers.freq[RH_RF] = feedback_devices::tacta_box::FREQ_OFF;
  grippers.freq[RH_LF] = feedback_devices::tacta_box::FREQ_OFF;
  grippers.freq[LH_RF] = feedback_devices::tacta_box::FREQ_OFF;
  grippers.freq[LH_LF] = feedback_devices::tacta_box::FREQ_OFF;

  grippers.amp_max[RH_RF] = feedback_devices::tacta_box::AMP_MED;
  grippers.amp_max[RH_LF] = feedback_devices::tacta_box::AMP_MED;
  grippers.amp_max[LH_RF] = feedback_devices::tacta_box::AMP_MED;
  grippers.amp_max[LH_LF] = feedback_devices::tacta_box::AMP_MED;

  grippers.amp_min[RH_RF] = feedback_devices::tacta_box::AMP_OFF;
  grippers.amp_min[RH_LF] = feedback_devices::tacta_box::AMP_OFF;
  grippers.amp_min[LH_RF] = feedback_devices::tacta_box::AMP_OFF;
  grippers.amp_min[LH_LF] = feedback_devices::tacta_box::AMP_OFF;

  ros::Rate r(10);

  while(ros::ok()){
      send_feedback();
      ros::spinOnce();
      r.sleep();
  }

  //Spin Forever
  ros::spin();

  return 0;

}

