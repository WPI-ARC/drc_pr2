hubo-ach-sim
============

simulator of hubo called OpenHUBO using OpenRave that takes commands from hubo-ach and put them on the virtual hubo

Notes from Bener Suay (benersuay@wpi.edu, December 2012):
======================

To run the executable

1. If you haven't made / sourced openHubo, for do that:
  
  $ roscd wpi_drc/openHubo/
  $ source ./setup-plugins.sh

2. Now go back to simulator and run it (note the argument -m lets you choose the model name. rlhuboplus_mit is the model we changed the home joint values for at mit. The model directory is hard coded to be wpi_drc/openHubo/huboplus):

  $ roscd wpi_drc/wpi_openrave/hubo/hubo-ach-sim/
  $ ./hubo-ach-openhubo -g -m rlhuboplus_mit

3. The following command would run the simulator to listen to ref-filtered channel:

  $ ./hubo-ach-openhubo -g -f -m rlhuboplus_mit



 


