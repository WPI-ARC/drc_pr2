/** \example hubo-ach-openhubo.cpp
    \author Daniel M. Lofaro

    Loads OpenHUBO model and updates the joint values via what is avaliable on ach.
    If there is a colisiion a flag is posted.

    Usage:
    \verbatim
    hubo-ach-openhubo [-g] [-v] [-V]
    \endverbatim

    - \b -g - Shows the robot in the QTCoin viewer.
    - \b -v - verbose mode and prints out colision state only
    - \b -V = Extra Verbose mode prints out colision state and location of colision

    Example:
    \verbatim
    hubo-ach-openhubo  -g
    \endverbatim

*/
#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <iostream>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>


/* ACH */
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <pthread.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include "ach.h"



/* Hubo Const */
#include "hubo.h"

// ach message type
//typedef struct hubo huboOpen[1];

// ach channels
ach_channel_t chan_num;

// Timing info
#define NSEC_PER_SEC    1000000000
//#define interval = 500000000; // 2hz (0.5 sec)
//#define interval = 10000000; // 100 hz (0.01 sec)

using namespace OpenRAVE;
using namespace std;


static inline void tsnorm(struct timespec *ts){

  while (ts->tv_nsec >= NSEC_PER_SEC) {
    ts->tv_nsec -= NSEC_PER_SEC;
    ts->tv_sec++;
  }
}


void SetViewer(EnvironmentBasePtr penv, const string& viewername)
{
  ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
  BOOST_ASSERT(!!viewer);

  // attach it to the environment:
  penv->AddViewer(viewer);

  // finally call the viewer's infinite loop (this is why a separate thread is needed)
  bool showgui = true;
  viewer->main(showgui);
}

int main(int argc, char ** argv)
{

  //int interval = 1000000000; // 1hz (1.0 sec)
  //int interval = 500000000; // 2hz (0.5 sec)
  //int interval = 10000000; // 100 hz (0.01 sec)
  int interval =   40000000; // 25 hz (0.04 sec)

  // At this point an explanation is necessary for the following arrays
  //
  // jointNames is the order makeTraj python code saves the joint values in plain text format
  string jointNames[] = {"RHY", "RHR", "RHP", "RKP", "RAP", "RAR", "LHY", "LHR", "LHP", "LKP", "LAP", "LAR", "RSP", "RSR", "RSY", "REP", "RWY", "RWR", "RWP", "LSP", "LSR", "LSY", "LEP", "LWY", "LWR", "LWP", "NKY", "NK1", "NK2", "HPY", "rightThumbKnuckle1", "rightIndexKnuckle1", "rightMiddleKnuckle1", "rightRingKnuckle1", "rightPinkyKnuckle1", "leftThumbKnuckle1", "leftIndexKnuckle1", "leftMiddleKnuckle1", "leftRingKnuckle1", "leftPinkyKnuckle1"};
  
  // openRAVE Joint Indices keeps the mapping between the order given above and hubo's joint index in openRAVE. (E.g. RHY's index in openRAVE is 1, RHR's is 3 and so on). A -1 means that the joint name does not exist in openRAVE but it exists on the real robot. (E.g. RWR)
  int openRAVEJointIndices[] = {1, 3, 5, 7, 9, 11, 2, 4, 6, 8, 10, 12, 13, 15, 17, 19, 21, -1, 23, 14, 16, 18, 20, 22, -1, 24, -1, -1, -1, 0, 41, 29, 32, 35, 38, 56, 44, 47, 50, 53};

  // hubo_ref Joint Indices keeps the mapping between openRAVE joint Indices and the order hubo-read-trajectory writes in ach channel. (E.g. RHY is the 26th element of H.ref[] array).
  int hubo_refJointIndices[] = {26, 27, 28, 29, 30, 31, 19, 20, 21, 22, 23, 24, 11, 12, 13, 14, 15, 16, 17, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 0, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41};

  /* Joint Numbers/Index values */	
  string viewername = "qtcoin";
  string scenefilename = "../../../openHubo/huboplus/";
  RaveInitialize(true); // start openrave core
  EnvironmentBasePtr penv = RaveCreateEnvironment(); // create the main environment
	
  int i = 1;
  bool vflag = false; // verbose mode flag
  bool Vflag = false; // extra verbose mode flag
  bool cflag = false; // self colision flag
  bool fflag = false; // ref vs. ref-filter channel flag
  /* For Viewer */
  if(argc <= i ){
    printf("Loading Headless\n");	
  }
  while(argc > i) {
    if(strcmp(argv[i], "-g") == 0) {
      //RaveSetDebugLevel(Level_Debug);
      boost::thread thviewer(boost::bind(SetViewer,penv,viewername));
    }
    else {
      printf("Loading Headless\n");
    }

		
    if(strcmp(argv[i], "-v") == 0) {
      vflag = true;
      printf("Entering Verbose Mode");
    }
    else {
      vflag = false;
    }
		
    if(strcmp(argv[i], "-V") == 0) {
      Vflag = true;
      printf("Entering Extra Verbose Mode");
    }

    if(strcmp(argv[i], "-f") == 0) {
      fflag = true;
      printf("listening to filtered channel.");
    }
		
    if(strcmp(argv[i], "-m") == 0) {
      i++;
      string hubomodel;
      hubomodel = argv[i];
      scenefilename.append(hubomodel);
      scenefilename.append(".robot.xml");
      cout<<"Loading model "<<scenefilename<<endl;
    }
    i++;
  }

  vector<dReal> vsetvalues;
  // parse the command line options
    
  // load the scene
  if( !penv->Load(scenefilename) ) {
    return 2;
  }

  // Wait until the robot model appears
  usleep(1000000);
	
  /* timing */
  struct timespec t;
	
  /* hubo ACH Channel */
  struct hubo_ref H;
  memset( &H,   0, sizeof(H));
  int r;
	
  if(fflag){
    r = ach_open(&chan_num, HUBO_CHAN_REF_FILTER_NAME, NULL);
  }
  else{
    r = ach_open(&chan_num, HUBO_CHAN_REF_NAME, NULL);
  }

  size_t fs;


  /* read first set of data */
  r = ach_get( &chan_num, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
  assert( sizeof(H) == fs );

  int contactpoints = 0;
  bool runflag = true;
  while(runflag) {
  {
    // lock the environment to prevent data from changing
    EnvironmentMutex::scoped_lock lock(penv->GetMutex());
    
    //vector<KinBodyPtr> vbodies;
    vector<RobotBasePtr> vbodies;
    //penv->GetBodies(vbodies);
    penv->GetRobots(vbodies);
    // get the first body
    if( vbodies.size() == 0 ) {
      RAVELOG_ERROR("no bodies loaded\n");
      return -3;
    }
    
    //KinBodyPtr pbody = vbodies.at(0);
    RobotBasePtr pbody = vbodies.at(0);
    vector<dReal> values;
    pbody->GetDOFValues(values);
    
    // set new values
    for(int i = 0; i < (int)vsetvalues.size() && i < (int)values.size(); ++i) {
      values[i] = vsetvalues[i];
    }
    
    pbody->Enable(true);
    //pbody->SetVisible(true);
    //CollisionReportPtr report(new CollisionReport());
    //bool runflag = true;
    //while(runflag) {
	
      /* Wait until next shot */
      clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);

      /* Get updated joint info here */
      r = ach_get( &chan_num, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
      assert( sizeof(H) == fs );

      
      /* set all joints from ach */
      int len = (int)(sizeof(openRAVEJointIndices)/sizeof(openRAVEJointIndices[0]));

      /* set all joints from ach */
      for( int ii = 0; ii < (int)values.size() ; ii++ ) {
	values[ii] = 0.0;
      }

      for( int ii = 0; ii < len; ii++){
	if(openRAVEJointIndices[ii] != -1){
	  values[openRAVEJointIndices[ii]] = H.ref[hubo_refJointIndices[ii]];
	}
      }

      //		values[RSY] = -1.0;
      //		values[REB] = 1.0;
      pbody->SetDOFValues(values,true);

      // penv->GetCollisionChecker()->SetCollisionOptions(CO_Contacts);
      // if( pbody->CheckSelfCollision(report) ) {
      // 	cflag = true;
      // if(vflag | Vflag){
      // 		RAVELOG_INFO("body not in collision\n");
      // 	}
      // 	if(Vflag) {   
      // 	 	contactpoints = (int)report->contacts.size();
      // 		stringstream ss;
      // 		ss << "body in self-collision "
      // 		<< (!!report->plink1 ? report->plink1->GetName() : "") << ":"
      // 		<< (!!report->plink2 ? report->plink2->GetName() : "") << " at "
      // 		<< contactpoints << "contacts" << endl;
      // 		for(int i = 0; i < contactpoints; ++i) {
      // 			CollisionReport::CONTACT& c = report->contacts[i];
      // 			ss << "contact" << i << ": pos=("
      // 			<< c.pos.x << ", " << c.pos.y << ", " << c.pos.z << "), norm=("
      // 			<< c.norm.x << ", " << c.norm.y << ", " << c.norm.z << ")" << endl;
      // 		}

      // 	RAVELOG_INFOA(ss.str());
      // 	}
      // }
      // else {
      // 	cflag = false;
      // 	if(vflag | Vflag) {
      // 		RAVELOG_INFO("body not in collision\n");
      // 	}
      // }
      // get the transformations of all the links
      vector<Transform> vlinktransforms;
      pbody->GetLinkTransformations(vlinktransforms);
      //boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
      //		boost::this_thread::sleep(boost::posix_time::milliseconds(1));
      t.tv_nsec+=interval;
      tsnorm(&t);
      
      //		runflag = false;
      //pbody->Enable(true);
      //pbody->SetVisible(true);
      
      usleep(10000);
      
      pbody->SimulationStep(0.01);
      penv->StepSimulation(0.01);
  } 
  }
  pause();
  RaveDestroy(); // destroy
  return contactpoints;
}
