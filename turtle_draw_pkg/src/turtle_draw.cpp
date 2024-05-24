#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "shape_drawer.h"
#include <sstream>

//Argc: number of strings, Argv: argument vector
int main(int argc, char *argv[]) {
//Smart pointer, check C++ documentation
  std::unique_ptr<ShapeDrawer> drawer;
  
//Initializing turtle node
  ros::init(argc, argv, "turtle_draw");

  if (argc < 2) {
    std::cerr << "Missing shape parameter" << std::endl;
  }
//Chooses which class is going to be used
  if (std::string(argv[1]) == "circle") {
    drawer = std::unique_ptr<ShapeDrawer>(new CircleDrawer());
  } else if (std::string(argv[1]) == "square") {
    drawer = std::unique_ptr<ShapeDrawer>(new SquareDrawer());
  } else if (std::string(argv[1]) == "triangle") {
    drawer = std::unique_ptr<ShapeDrawer>(new TriangleDrawer());
  } else if (std::string(argv[1]) == "pentagon") {
    drawer = std::unique_ptr<ShapeDrawer>(new PentagonDrawer());
  } else {
  //If user didnt choose a proper shape, sends an error
    std::cerr << "Shape is invalid, use: triangle|square|pentagon|circle" << std::endl;
    return 1;
  }
//Starts drawing
  drawer->draw();

  return 0;
}