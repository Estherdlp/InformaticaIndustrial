#include <iostream>
#include <cstdlib>
#include <sqlite3.h>
#include <vector>
#include <string>
#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"

#include "urjc_utils/PickUpSrv.h"
#include "urjc_utils/DeliverSrv.h"
#include "urjc_utils/OrderLine.h"

#include "data.h"
#include "delirobot.h"

//Constructor for Robot. It needs a NodeHandle to create all services
Delirobot::Delirobot(ros::NodeHandle *nh) {  
    this->nh = *nh;
    this->posePub = this->nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    this->pickUpClient = this->nh.serviceClient<urjc_utils::PickUpSrv>("pickup");
    this->deliverClient = this->nh.serviceClient<urjc_utils::DeliverSrv>("deliver");
}

//Starts robot to execute all functionalities of picking and delivering
void Delirobot::startWorking(){
    //Starts working
    retrievePendingOrders();

    for (auto orderId : this->orders){
        retrieveOrderLines(orderId);
        if (this->orderProducts.size() > 0) {
            attendOrder();
            this->orderProducts.clear();
        }
    }
}

//Reads from DB pending orders
void Delirobot::retrievePendingOrders(){
    const char* env_db;

    if(env_db = std::getenv("WAREHOUSE_DB"))
        std::cout << "Your database path is: " << env_db << '\n'; 
    
    sqlite3 *db;
    int rc = sqlite3_open(env_db, &db);
    if (rc != SQLITE_OK) {       //Debugging error message if DB not available
        fprintf(stderr, "DB not available: %s.\n", sqlite3_errmsg(db));
    }
    const char *sql = 
    "SELECT DISTINCT(orderId) as orderId FROM \"Order\" ORDER BY orderId ASC";

    sqlite3_stmt *result;

    rc = sqlite3_prepare(db, sql, -1, &result, NULL);
    if (rc != SQLITE_OK) {      //Debugging error message
        fprintf(stderr, "Error executing sql %s: %s.\n", sql, sqlite3_errmsg(db));
    }

    while (sqlite3_step(result) == SQLITE_ROW) {
        
        std::cout 
            << "Pending Order - orderId: " << sqlite3_column_int(result, 0) << std::endl;

        this->orders.push_back(sqlite3_column_int(result, 0));  
    }
}


//Reads from DB lines from order
void Delirobot::retrieveOrderLines(int orderId){
    const char* env_db;

    if(env_db = std::getenv("WAREHOUSE_DB"))
        std::cout << "Your database path is: " << env_db << '\n'; 
    
    sqlite3 *db;
    int rc = sqlite3_open(env_db, &db);
    if (rc != SQLITE_OK) {          //Debugging error message if DB not available
        fprintf(stderr, "DB not available: %s.\n", sqlite3_errmsg(db));
    }

    //Data obtained ordered by productId
    const char *sql = 
    "SELECT o.orderId, pz.xLocation, pz.yLocation, ol.productId , ol.units  from \"Order\" o "
    "JOIN OrderLine ol ON ol.orderId = o.orderId "
    "JOIN Inventory i ON ol.productId  = i.productId "
    "JOIN PickupZone pz ON pz.pickupZoneId  = i.pickupZoneId "
    "WHERE o.orderId = ? ORDER BY ol.productId";

    sqlite3_stmt *result;

    rc = sqlite3_prepare(db, sql, -1, &result, NULL);
    rc = sqlite3_bind_int(result, 1, orderId);  //Added value for first Parameter
    if (rc != SQLITE_OK) {
        fprintf(stderr, "Error al ejecutar el sql %s: %s.\n", sql, sqlite3_errmsg(db));
    }

    while (sqlite3_step(result) == SQLITE_ROW) {
        pickup_line_t order;        
        order.orderId = sqlite3_column_int(result, 0);
        order.x = (float)sqlite3_column_double(result, 1);
        order.y = (float)sqlite3_column_double(result, 2);
        order.productId = sqlite3_column_int(result, 3);
        order.units = sqlite3_column_int(result, 4);

        std::cout 
            << "orderId: " << order.orderId
            << " - productId: " << order.productId
            << " - units " << order.units
            << " - xLocation: " << order.x
            << " - yLocation: " << order.y << std::endl;

        this->orderProducts.push_back(order);  
    }
}


//Start to review and pickup pending orders
void Delirobot::attendOrder() {
    int orderId = -1;

    for (auto pickupLine : this->orderProducts){
        if (orderId != -1 && orderId != pickupLine.orderId){
            ROS_INFO("Delivery Orders");
            startDeliveryOrder();
            ROS_INFO("Clear pickedProducts");
            this->pickedProducts.clear();
        }

        orderId = pickupLine.orderId;
        pickupOrderLine(pickupLine);
        this->pickedProducts.push_back(pickupLine);
    }
    
    if (orderId != -1) {
        ROS_INFO("Delivery Orders");
        startDeliveryOrder();
        ROS_INFO("Clear pickedProducts");
        this->pickedProducts.clear();
    }

}

//Pickup line items
void Delirobot::pickupOrderLine(pickup_line_t pickupLine){
    bool ready = false;
    ros::Rate loop_rate(1);

    while(!ready){

        //Go to pickupLine coordinates
        goTo(pickupLine.x, pickupLine.y);

        //Call to pickup service
        ready = pickupProduct(pickupLine.productId, pickupLine.units);

        if (ready){
            continue;
        }

        for(int i = 0; i < 10; i++){
            ros::spinOnce();
            loop_rate.sleep();   
        }
        
    }
    ROS_INFO("NEXT Product");
}
 
//Move robot to a position using map loaded previously
void Delirobot::goTo(float x, float y){

    ros::Rate loop_rate(1);
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.w = 1.0;
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;

    this->posePub.publish(goal);
    ros::spinOnce();
    loop_rate.sleep();
}

//Pick up the product
bool Delirobot::pickupProduct(int productId, int units){

    urjc_utils::PickUpSrv srv;
    
    srv.request.order.productId = productId;
    srv.request.order.units = units;
    
    if(this->pickUpClient.call(srv)){
        return srv.response.done;
    }
    return false;
}

//Go back to delivery point and start delivering
void Delirobot::startDeliveryOrder(){
    if (this->pickedProducts.size() > 0){
        bool ready = false;
        ROS_INFO("Trying to deliver order...");
        ros::Rate loop_rate(1);

        while(!ready){

            goTo(0, 0);     //Go back to delivery area

            ready = deliverOrder();

            for(int i = 0; i < 10; i++){
                ros::spinOnce();
                loop_rate.sleep();   
            }
        }
    }
}

//Go back to delivery point and start delivering
bool Delirobot::deliverOrder(){

    urjc_utils::DeliverSrv srv;
    srv.request.orderId = this->pickedProducts[0].orderId;

    for(auto line : this->pickedProducts){
        urjc_utils::OrderLine orderLine;
        orderLine.productId = line.productId;
        orderLine.units = line.units;
        srv.request.lines.push_back(orderLine);
    }
    
    if(this->deliverClient.call(srv)){
        return srv.response.done;
    }
    
    return false;
}