#ifndef SHAPE_DRAWER_H
#define SHAPE_DRAWER_H
#include "ros/ros.h"

//Class headers for turtle_draw
class ShapeDrawer {
    protected:
        ros::NodeHandle node;
        ros::Publisher turtlesim_pub;
    public:
        ShapeDrawer();
        virtual void draw() = 0;
};
//Each class is a different shape
class CircleDrawer : public ShapeDrawer {
    public:
        void draw();
};
class SquareDrawer : public ShapeDrawer {
    public:
        void draw();
};

class TriangleDrawer : public ShapeDrawer {
    public:
        void draw();
};
class PentagonDrawer : public ShapeDrawer {
    public:
        void draw();
};

#endif