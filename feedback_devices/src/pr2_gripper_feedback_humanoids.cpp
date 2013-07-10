#include "ros/ros.h"
#include "feedback_devices/tacta_hydra.h"
#include "pr2_controllers_msgs/JointControllerState.h"
#include "pr2_controllers_msgs/Pr2GripperCommand.h"
#include "pr2_msgs/PressureState.h"
#include "std_msgs/Int16.h"
#include <iostream>
#include <fstream>

//======================== DEFINED CONSTANTS ==================================

//Robot Specific Constants
#define HANDS 2
#define FINGERS 2
#define SENSORS 22

//Fingertip Sensor Specific Constants
#define SENSOR_START 7
#define RAW_TO_NEWT 1600 //Determined from PR2 Code and NOT VERIFIED
#define MAX_FORCE_NEWT 40
#define FMIN .25
#define CRUSH_LIMIT .01
#define OUTPUT_MIN 0

//Kuchenbecker Constatns
#define FLIMIT 1 //Newtons that defines contact
#define TOUCH_READINGS 1 //Number of readings to take immediately after touching an object

#define CALIBRATION_READINGS 3
#define FREQ_THRESHOLD 75

//======================== TYPE DEFINES AND ENUMS =============================

enum hand_t {LEFT_HAND, RIGHT_HAND};
enum finger_t {LEFT_FINGER, RIGHT_FINGER};
enum state_t {UNCALIBRATED, OPEN, TOUCH, LOAD, CRUSH};
enum auto_state_t {AUTO_WAIT, AUTO_CLOSE, AUTO_LOAD, AUTO_LIFT, AUTO_OPEN};

//======================== FUNCTION PROTOTYPES ================================

void r_gripper_pressure_cb(pr2_msgs::PressureState input);
void l_gripper_pressure_cb(pr2_msgs::PressureState input);
void calculate_pressure(pr2_msgs::PressureState input, hand_t hand);

void r_gripper_state_cb(pr2_controllers_msgs::JointControllerState input);
void l_gripper_state_cb(pr2_controllers_msgs::JointControllerState input);
void calculate_gripper_state(pr2_controllers_msgs::JointControllerState input, hand_t hand);

bool detect_slip(hand_t hand);

void printPressureLog(int hand);

//======================== GLOBAL ARRAYS ======================================

int state[HANDS] = {UNCALIBRATED};
int state_prev[HANDS] = {UNCALIBRATED};
double contact[HANDS] = {0};
int auto_state[HANDS] = {AUTO_WAIT};

double force[HANDS] = {0};
double gripper[HANDS] = {0};
double gripper_prev [HANDS] = {0};

double hardness[HANDS] = {0};
int touch_num[HANDS] = {0};

int current_value[HANDS][FINGERS][SENSORS] = {{{0}}};
int prev_current_value[HANDS][FINGERS][SENSORS] = {{{0}}};

int calibration_value[HANDS][FINGERS][SENSORS] = {{{0}}};
int calibration_sum[HANDS][FINGERS][SENSORS] = {{{0}}};
int calibration_count[HANDS] = {0};

ros::Publisher pub_tacta_hydra;
ros::Publisher pub_r_gripper_command;
feedback_devices::tacta_hydra hydra;

//======================== FUNCTIONS ==========================================

void r_gripper_state_cb(pr2_controllers_msgs::JointControllerState input){

    calculate_gripper_state(input, RIGHT_HAND);

}

void l_gripper_state_cb(pr2_controllers_msgs::JointControllerState input){

    calculate_gripper_state(input, LEFT_HAND);

}

void calculate_gripper_state(pr2_controllers_msgs::JointControllerState input, hand_t hand){

    gripper_prev[hand] = gripper[hand];
    gripper[hand] = input.process_value;

    //Check what state the hand is in
    if (state[hand] == TOUCH || state[hand] == LOAD) {

        //Check if we already have a value
        if (contact[hand] == 0) {

            contact[hand] = input.process_value;

            //ROS_INFO("        Contact Made w/ Gripper at : %f", contact[hand]);

        }

        //Check to make sure the difference between the two isn't too large
        if ((contact[hand] - input.process_value) > CRUSH_LIMIT) {
            state_prev[hand] = state[hand];
            state[hand] = CRUSH;

            ROS_INFO("        Object Crushed when Gripper Passed : %f w/ a force of %4.2f", input.process_value, force[hand]);

        }
    }
    else if ((contact[hand] - input.process_value) <= CRUSH_LIMIT && state[hand] == CRUSH) {

        state[hand] = state_prev[hand];

        //ROS_INFO("        Switching back to state : %d", state_prev[hand]);

    }

}

void r_gripper_pressure_cb(pr2_msgs::PressureState input){

    calculate_pressure(input, RIGHT_HAND);
}


void l_gripper_pressure_cb(pr2_msgs::PressureState input){

//    calculate_pressure(input, LEFT_HAND);
}

void calculate_pressure(pr2_msgs::PressureState input, hand_t hand){

    //ROS_INFO("================================================================");
    //ROS_INFO("The States are: 0-Uncalibrated, 1-Open, 2-Touch, 3-Load, 4-Crush");
    //ROS_INFO("Entering Pressure ==> Hand: %d, State: %d", hand, state[hand]);

    int rf_adc = 0;
    int lf_adc = 0;

    //Remove the calibration value and calculate the overal pressure
    for (int sensor = SENSOR_START; sensor < SENSORS; sensor++){

        //Remove the calibration value
        current_value[hand][LEFT_FINGER][sensor] = input.l_finger_tip[sensor] - calibration_value[hand][LEFT_FINGER][sensor];
        current_value[hand][RIGHT_FINGER][sensor] = input.r_finger_tip[sensor] - calibration_value[hand][RIGHT_FINGER][sensor];

        //Make sure the values don't go negative
        if (current_value[hand][LEFT_FINGER][sensor] < 0) current_value[hand][LEFT_FINGER][sensor] = 0;
        if (current_value[hand][RIGHT_FINGER][sensor] < 0) current_value[hand][RIGHT_FINGER][sensor] = 0;

        //Sum the pressure
        rf_adc += current_value[hand][LEFT_FINGER][sensor];
        lf_adc += current_value[hand][RIGHT_FINGER][sensor];

        //Sum the pressure
        rf_adc += current_value[hand][LEFT_FINGER][sensor];
        lf_adc += current_value[hand][RIGHT_FINGER][sensor];
    }

    float right_newton = (float)rf_adc / (float)RAW_TO_NEWT;
    float left_newton = (float)lf_adc / (float)RAW_TO_NEWT;
    float total_newton = .5 * (right_newton + left_newton);
    force[hand] = total_newton;

    if (total_newton > MAX_FORCE_NEWT) total_newton = MAX_FORCE_NEWT;

//    ROS_INFO("    Current Force: L( %d ), R( %d ), Total( %f )", lf_adc, rf_adc, .5 * (rf_adc + lf_adc));
//    ROS_INFO("    Current Force: L( %4.2f N), R( %4.2f N), Total( %4.2f N)", left_newton, right_newton, total_newton);




    //If the state of the hand is OPEN
    if (state[hand] == OPEN) {

        //ROS_INFO("    Entering State OPEN");

        hydra.amp[hand] = OUTPUT_MIN;
        hydra.freq[hand] = feedback_devices::tacta_hydra::FREQ_ON;

        if (left_newton >= FLIMIT && right_newton >= FLIMIT) {

            //ROS_INFO("    Contact Made ( %f N) ( %f N), Transitioning OPEN ==> TOUCH", left_newton, right_newton);
            hardness[hand] = 0;
            state[hand] = TOUCH;

        }
    }



    //If the state of the hand is TOUCH
    if (state[hand] == TOUCH) {

        ROS_INFO("    Entering State TOUCH");

        //Increment the number of times we've touched the object
        touch_num[hand]++;

        ROS_INFO("    %d of %d Touch Readings Recorded", touch_num[hand], TOUCH_READINGS);

        //Calculate the max force seen during this time
        if (total_newton > hardness[hand]) hardness[hand] = total_newton;

        //If we have touched the object enough times, move to the next state
        if (touch_num[hand] >= TOUCH_READINGS) {

            ROS_INFO("    All Readings Taken, Hardness ( %4.2f )", hardness[hand]);
            touch_num[hand] = 0;
            state[hand] = LOAD;

        }

        //TODO Make sure that this doesn't cause problems!
        if (right_newton < FLIMIT || left_newton < FLIMIT) {

            //ROS_INFO("    Too little force on sensors, Switching from TOUCH ==> OPEN\n");

            touch_num[hand] = 0;
            hardness[hand] = 0;
            contact[hand] = 0;
            state[hand] = OPEN;
        }

    }



    //If the gripper has been rh_calibrated then this code can just be run normally
    if (state[hand] == LOAD) {

        //ROS_INFO("    Entering State LOAD");

        hydra.amp[hand] = OUTPUT_MIN + (total_newton / MAX_FORCE_NEWT) * (255 - OUTPUT_MIN);

        if(detect_slip(hand)){ //Slip is Occuring

            hydra.freq[hand] = feedback_devices::tacta_hydra::FREQ_SLIP;

        } else { //Slip is not Occuring

            hydra.freq[hand] = feedback_devices::tacta_hydra::FREQ_ON;

        }

        //Save the current state as the previous current state
        for (int sensor = SENSOR_START; sensor < SENSORS; sensor++){
            prev_current_value[hand][LEFT_FINGER][sensor] = current_value[hand][LEFT_FINGER][sensor];
            prev_current_value[hand][RIGHT_FINGER][sensor] = current_value[hand][RIGHT_FINGER][sensor];
        }

        //TODO Make sure that this doesn't cause problems!
        if (right_newton < FLIMIT || left_newton < FLIMIT) {

            //ROS_INFO("    Too little force on sensors, Switching from LOAD ==> OPEN");

            //Save the current state as the previous current state
            for (int sensor = SENSOR_START; sensor < SENSORS; sensor++){
                prev_current_value[hand][LEFT_FINGER][sensor] = 0;
                prev_current_value[hand][RIGHT_FINGER][sensor] = 0;
            }

            contact[hand] = 0;
            state[hand] = OPEN;
        }

    } //End State == LOAD


    if (state[hand] == CRUSH) {

        //ROS_INFO("    Entering State CRUSH");

        hydra.amp[hand] = 255;
        hydra.freq[hand] = feedback_devices::tacta_hydra::FREQ_CRUSH;

    }


    if (state[hand] == UNCALIBRATED) {

        //ROS_INFO("    Entering State UNCALIBRATED");

        // We're calibrating by taking the average of the sensor data, this will sum them
        for (int sensor = SENSOR_START; sensor < SENSORS; sensor++){
            calibration_sum[hand][LEFT_FINGER][sensor] += input.l_finger_tip[sensor];
            calibration_sum[hand][RIGHT_FINGER][sensor] += input.r_finger_tip[sensor];
        }

        calibration_count[hand]++;

        //ROS_INFO("    %d of %d Calibration Messages Recieved", calibration_count[hand], CALIBRATION_READINGS);

        if ( calibration_count[hand] >= CALIBRATION_READINGS){
            for (int sensor = SENSOR_START; sensor < SENSORS; sensor++){
                calibration_value[hand][LEFT_FINGER][sensor] = calibration_sum[hand][LEFT_FINGER][sensor] / calibration_count[hand];
                calibration_value[hand][RIGHT_FINGER][sensor] = calibration_sum[hand][RIGHT_FINGER][sensor] / calibration_count[hand];
            }
            state[hand] = OPEN;
        }
    }

}


bool detect_slip(hand_t hand){

    int min = 30000, max = -30000;

    for (int sensor = SENSOR_START; sensor < SENSORS; sensor++){

        int l_deriv = current_value[hand][LEFT_FINGER][sensor] - prev_current_value[hand][LEFT_FINGER][sensor];
        int r_deriv = current_value[hand][RIGHT_FINGER][sensor] - prev_current_value[hand][RIGHT_FINGER][sensor];

        if (l_deriv < min) min = l_deriv;
        if (l_deriv > max) max = l_deriv;

        if (r_deriv < min) min = r_deriv;
        if (r_deriv > max) max = r_deriv;

    }

    //Determine the output

    //If both of the numbers are within the noise range don't do anything
    if (abs(min) < FREQ_THRESHOLD && abs(max) < FREQ_THRESHOLD) return false;

    //If the numbers have different signs, return that we're slipping
    if ((min < 0 && abs(min) > FREQ_THRESHOLD) && (max > 0 && abs(max) > FREQ_THRESHOLD)) return true;

    //Else return that we're not slipping
    return false;

}

//std::ofstream rh_rf_data, rh_lf_data, rh_rf_der, rh_lf_der;
//std::ofstream lh_rf_data, lh_lf_data, lh_rf_der, lh_lf_der;
//int rh_count = 0, lh_count = 0;
//void printPressureLog(int hand){

//    if (hand == RIGHT){

//        if (rh_count == 0){
//            rh_rf_data.open ("rh_rf_pressure.csv");
//            rh_lf_data.open ("rh_lf_pressure.csv");
//            rh_rf_der.open("rh_rf_derivative.csv");
//            rh_lf_der.open("rh_lf_derivative.csv");
//        }

//        for (int i = 0; i < SENSORS; i++){

//            rh_rf_data << rh_rf_current[i] << ",";
//            rh_lf_data << rh_lf_current[i] << ",";
//            rh_rf_der << rh_rf_deriv[i] << ",";
//            rh_lf_der << rh_lf_deriv[i] << ",";

//        }

//        rh_count++;
//        rh_rf_data << "\n";
//        rh_lf_data << "\n";
//        rh_rf_der << "\n";
//        rh_lf_der << "\n";

//    } else {

//        if (lh_count == 0){
//            lh_rf_data.open ("lh_rf_pressure.csv");
//            lh_lf_data.open ("lh_lf_pressure.csv");
//            lh_rf_der.open("lh_rf_derivative.csv");
//            lh_lf_der.open("lh_lf_derivative.csv");
//        }

//        for (int i = 0; i < SENSORS; i++){

//            lh_rf_data << lh_rf_current[i] << ",";
//            lh_lf_data << lh_lf_current[i] << ",";
//            lh_rf_der << lh_rf_deriv[i] << ",";
//            lh_lf_der << lh_lf_deriv[i] << ",";

//        }

//        lh_count++;
//        lh_rf_data << "\n";
//        lh_lf_data << "\n";
//        lh_rf_der << "\n";
//        lh_lf_der << "\n";


//    }
//}

#define BUTTON_1 1
#define BUTTON_2 2
#define BUTTON_3 3
#define BUTTON_4 4
#define BUTTON_START 5
#define BUTTON_STICK 6
#define BUTTON_FRONT 0


double currentForce = 1;
double force_e = .1;
double force_d = .25;
double gripper_d = .001;
double gripper_d_fine = .0001;
bool closed = true;

int count = 0;

void start_force_servo(void){

    if (auto_state[RIGHT_HAND] == AUTO_WAIT) {

        auto_state[RIGHT_HAND] = AUTO_CLOSE;

    } else if (auto_state[RIGHT_HAND] == AUTO_LIFT) {

        auto_state[RIGHT_HAND] = AUTO_CLOSE;

    }

}

void run_crush_test(void) {

    int hand = RIGHT_HAND;

    count++;

    if (count >= 3) {
        count = 0;

            //If this loop was just activate it will be in the auto_close state
            if (auto_state[hand] == AUTO_CLOSE) {

                //If we're determined that we're done, then print the force used
                //We're done if the force is within an epsilon of the desired force
                //And both of the grippers are making contact with the object

                //Start by checking if both of the finger tips are touching the object
                if (contact[hand] != 0) {

                    //Check if the force is within some epsilon
                    if (gripper[hand] <= (contact[hand] - .01)) {

                        //Print the result
                        ROS_INFO("Task Completed Force: (%4.2f / %4.2f) @ Hardness: %4.2f", force[hand], currentForce, hardness[hand]);

                        //Move to the next state
                        auto_state[hand] = AUTO_LIFT;

                    //If there's not enough force yet, then keep closing the hand
                    } else if (gripper[hand] > (contact[hand] - .01)) {

                        //If we are here then the force was not enough and we need to close the gripper more
                        pr2_controllers_msgs::Pr2GripperCommand gripper_command;
                        gripper_command.max_effort = 100;

                        //Check what the delta should be for the hand
                        if (gripper[hand] == gripper_prev[hand]) gripper_d += .001;
                        else gripper_d -= .001;
                        if (gripper_d <= 0) gripper_d = 0;

                        //Move the gripper based off of where
                        gripper_command.position = gripper[hand] - gripper_d;
                        if (gripper_command.position < 0) gripper_command.position = 0;
                        if (gripper_command.position > .08) gripper_command.position = .08;

                        ROS_INFO("CRUSHING(fine) with Force = %4.2f / %4.2f, Commanding: %4.4f w/ %4.4f", force[hand], currentForce, gripper_command.position, gripper_d);

                        //Publish the gripper command to close the gripper a bit more
                        pub_r_gripper_command.publish(gripper_command);
                    }

                //Both of the fingers are not touching the object since we are not in TOUCH or LOAD state
                //So the gripper needs to be told to continue closing until that happens
                } else {

                    //If we are here then the force was not enough and we need to close the gripper more
                    pr2_controllers_msgs::Pr2GripperCommand gripper_command;
                    gripper_command.max_effort = 100;

                    //Check what the delta should be for the hand
                    if (gripper[hand] == gripper_prev[hand]) gripper_d += .001;
                    else gripper_d = .001;
                    if (gripper_d <= 0) gripper_d = 0;

                    //Move the gripper based off of where
                    gripper_command.position = gripper[hand] - gripper_d;
                    if (gripper_command.position < 0) gripper_command.position = 0;
                    if (gripper_command.position > .08) gripper_command.position = .08;

                    ROS_INFO("CRUSHING with Force = %4.2f / %4.2f, Commanding: %4.4f w/ %4.4f", force[hand], currentForce, gripper_command.position, gripper_d);

                    //Publish the gripper command to close the gripper a bit more
                    pub_r_gripper_command.publish(gripper_command);

                }

            } else if (auto_state[hand] == AUTO_LIFT) {

                //LIFT THE HAND HERE!

                //Transition to the next state
                //auto_state[hand] = AUTO_OPEN;

            } else if (auto_state[hand] == AUTO_OPEN) {

                //If we are here then the force was not enough and we need to close the gripper more
                pr2_controllers_msgs::Pr2GripperCommand gripper_command;
                gripper_command.max_effort = 100;
                gripper_command.position = .08;
                pub_r_gripper_command.publish(gripper_command);

                currentForce += force_d;

                auto_state[hand] = AUTO_WAIT;

            }
        }

}

void r_button_press (std_msgs::Int16 input){

    if (input.data == BUTTON_1) currentForce = 1;
    else if (input.data == BUTTON_2) currentForce = 2;
    else if (input.data == BUTTON_3) currentForce = 3;
    else if (input.data == BUTTON_4) currentForce = 4;
    else if (input.data == BUTTON_START) currentForce += force_d;
    else if (input.data == BUTTON_STICK){

        ROS_INFO("Current Force: %4.2f", force[RIGHT_HAND]);

        //auto_state[RIGHT_HAND] = AUTO_CLOSE;
        //start_force_servo();

    }

}

void forceServo(){

    int hand = RIGHT_HAND;

    count++;

    if (count >= 3) {
        count = 0;

            //If this loop was just activate it will be in the auto_close state
            if (auto_state[hand] == AUTO_CLOSE) {

                //If we're determined that we're done, then print the force used
                //We're done if the force is within an epsilon of the desired force
                //And both of the grippers are making contact with the object

                //Start by checking if both of the finger tips are touching the object
                if (state[hand] == TOUCH || state[hand] == LOAD) {

                    //Check if the force is within some epsilon
                    if (force[hand] <= currentForce + force_e && force[hand] >= currentForce - force_e) {

                        //Print the result
                        ROS_INFO("Task Completed Force: (%4.2f / %4.2f) @ Hardness: %4.2f", force[hand], currentForce, hardness[hand]);

                        //Move to the next state
                        auto_state[hand] = AUTO_LIFT;

                    //If there's not enough force yet, then keep closing the hand
                    } else if (force[hand] < currentForce) {

                        //If we are here then the force was not enough and we need to close the gripper more
                        pr2_controllers_msgs::Pr2GripperCommand gripper_command;
                        gripper_command.max_effort = 100;

                        if (closed == false) {
                            closed = true;
                            gripper_d_fine = 0;
                        }

                        //Check what the delta should be for the hand
                        if (gripper[hand] == gripper_prev[hand]) gripper_d_fine += .0001;
                        else gripper_d_fine -= .0001;
                        if (gripper_d_fine <= 0) gripper_d_fine = 0;

                        //Move the gripper based off of where
                        gripper_command.position = gripper[hand] - gripper_d_fine;

                        ROS_INFO("CLOSING(fine) with Force = %4.2f / %4.2f, Commanding: %4.4f w/ %4.4f", force[hand], currentForce, gripper_command.position, gripper_d_fine);

                        //Publish the gripper command to close the gripper a bit more
                        pub_r_gripper_command.publish(gripper_command);

                    //There is too much force, open the gripper
                    } else if (force[hand] > currentForce) {

                        //If we are here then the force was not enough and we need to close the gripper more
                        pr2_controllers_msgs::Pr2GripperCommand gripper_command;
                        gripper_command.max_effort = 100;

                        if (closed == true) {
                            closed = false;
                            gripper_d_fine = 0;
                        }

                        //Check what the delta should be for the hand
                        if (gripper[hand] == gripper_prev[hand]) gripper_d_fine += .0001;
                        else gripper_d_fine -= .0001;
                        if (gripper_d_fine <= 0) gripper_d_fine = 0;

                        //Move the gripper based off of where
                        gripper_command.position = gripper[hand] + gripper_d_fine;

                        ROS_INFO("OPENING(fine) with Force = %4.2f / %4.2f, Commanding: %4.4f w/ %4.4f", force[hand], currentForce, gripper_command.position, gripper_d_fine);

                        //Publish the gripper command to close the gripper a bit more
                        pub_r_gripper_command.publish(gripper_command);

                    }

                //Both of the fingers are not touching the object since we are not in TOUCH or LOAD state
                //So the gripper needs to be told to continue closing until that happens
                } else {

                    //If we are here then the force was not enough and we need to close the gripper more
                    pr2_controllers_msgs::Pr2GripperCommand gripper_command;
                    gripper_command.max_effort = 100;

                    //Check what the delta should be for the hand
                    if (gripper[hand] == gripper_prev[hand]) gripper_d += .001;
                    else gripper_d = .001;
                    if (gripper_d <= 0) gripper_d = 0;

                    //Move the gripper based off of where
                    gripper_command.position = gripper[hand] - gripper_d;

                    ROS_INFO("CLOSING with Force = %4.2f / %4.2f, Commanding: %4.4f w/ %4.4f", force[hand], currentForce, gripper_command.position, gripper_d);

                    //Publish the gripper command to close the gripper a bit more
                    pub_r_gripper_command.publish(gripper_command);

                }

            } else if (auto_state[hand] == AUTO_LIFT) {

                //LIFT THE HAND HERE!

                //Transition to the next state
                //auto_state[hand] = AUTO_OPEN;

            } else if (auto_state[hand] == AUTO_OPEN) {

                //If we are here then the force was not enough and we need to close the gripper more
                pr2_controllers_msgs::Pr2GripperCommand gripper_command;
                gripper_command.max_effort = 100;
                gripper_command.position = .08;
                pub_r_gripper_command.publish(gripper_command);

                currentForce += force_d;

                auto_state[hand] = AUTO_WAIT;

            }
        }



}

void send_feedback(){
    pub_tacta_hydra.publish(hydra);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pr2_grippers_humanoids");

  //Setup everything for the publishes
  ros::NodeHandle nh;

  //Advertise the two publishers, one for the commands and one for the gui
  pub_tacta_hydra = nh.advertise<feedback_devices::tacta_hydra>("/feedback_devices/tacta_hydra/input", 1);
  pub_r_gripper_command = nh.advertise<pr2_controllers_msgs::Pr2GripperCommand>("r_gripper_controller/command", 1);

  //Create all the subscribers that are needed
  ros::Subscriber sub_r_gripper_state = nh.subscribe("/r_gripper_controller/state",1, r_gripper_state_cb);
  ros::Subscriber sub_l_gripper_state = nh.subscribe("/l_gripper_controller/state",1, l_gripper_state_cb);
  ros::Subscriber sub_r_gripper_pressure = nh.subscribe("/pressure/r_gripper_motor", 1, r_gripper_pressure_cb);
  ros::Subscriber sub_l_gripper_pressure = nh.subscribe("/pressure/l_gripper_motor", 1, l_gripper_pressure_cb);
  ros::Subscriber sub_r_button_press = nh.subscribe("pr2_hydra_dev/right_paddle/button_press", 1, r_button_press);

  hydra.freq[LEFT_HAND] = feedback_devices::tacta_hydra::FREQ_OFF;
  hydra.freq[RIGHT_HAND] = feedback_devices::tacta_hydra::FREQ_OFF;

  ros::Rate r(30);

  while(ros::ok()){
      send_feedback();
      //forceServo();
      run_crush_test();
      ros::spinOnce();
      r.sleep();
  }

  //Spin Forever
  ros::spin();

  return 0;

}
