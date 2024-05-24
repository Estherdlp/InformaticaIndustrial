#ifndef ROLIVERY_DELIROBOT_H
#define ROLIVERY_DELIROBOT_H

#include "ros/ros.h"
#include "data.h"

class Delirobot{
    public:
        Delirobot(ros::NodeHandle *nh);         //Node management
        void startWorking();
    private:
    //Manages services and ads for picking, delivering and moving.
        ros::ServiceClient pickUpClient;
        ros::ServiceClient deliverClient;
        ros::NodeHandle nh;
        ros::Publisher posePub;
        std::vector<int> orders;
        std::vector<pickup_line_t> orderProducts;
        std::vector<pickup_line_t> pickedProducts;

        void retrievePendingOrders();           //Reads from DB pending orders
        void retrieveOrderLines(int orderId);   //Reads from DB lines from each order
        void attendOrder();                     //Attends order
        void pickupOrderLine(pickup_line_t pickupLine);     //Manages movement and picking
        void goTo(float x, float y);            //Moves robot
        bool pickupProduct(int productId, int units);   //Pick products
        void startDeliveryOrder();              //Starts return to deliver products
        bool deliverOrder();                    //Deliver products
};

#endif