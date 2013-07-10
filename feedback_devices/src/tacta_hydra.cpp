#include "ros/ros.h"
#include "feedback_devices/tacta_box.h"
#include "feedback_devices/tacta_hydra.h"

ros::Publisher pub_tacta_box;

void hydra_cb(feedback_devices::tacta_hydra input){

    feedback_devices::tacta_box box;

//    box.active[0] = feedback_devices::tacta_box::ACTIVE;
//    box.freq[0] = input.freq[feedback_devices::tacta_hydra::LEFT];
//    box.amp_max[0] = input.amp[feedback_devices::tacta_hydra::LEFT];
//    box.amp_min[0] = input.amp[feedback_devices::tacta_hydra::LEFT];

    box.active[3] = feedback_devices::tacta_box::ACTIVE;
    box.freq[3] = input.freq[feedback_devices::tacta_hydra::RIGHT];
    box.amp_max[3] = input.amp[feedback_devices::tacta_hydra::RIGHT];
    box.amp_min[3] = 0;

    pub_tacta_box.publish(box);

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "tacta_hydra");

  //Setup everything for the publishes
  ros::NodeHandle nh;

  //Advertise the two publishers, one for the commands and one for the gui
  pub_tacta_box = nh.advertise<feedback_devices::tacta_box>("/feedback_devices/tacta_box/input", 1);

  //Create all the subscribers that are needed
  ros::Subscriber sub_tacta_hydra = nh.subscribe("/feedback_devices/tacta_hydra/input",1, hydra_cb);

  //Spin Forever
  ros::spin();

  return 0;

}

