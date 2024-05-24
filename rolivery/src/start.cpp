#include "ros/ros.h"
#include "libs/delirobot.h"
#include "urjc_utils/PickUpSrv.h"
#include "urjc_utils/DeliverSrv.h"
#include "urjc_utils/OrderLine.h"
#include <sstream>

int main(int argc, char *argv[]) {  
//Initializing node management
  ros::init(argc, argv, "start");
  ros::NodeHandle nh;

  Delirobot delirobot = Delirobot(&nh);
  delirobot.startWorking();
  return 0;
}