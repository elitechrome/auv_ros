#include "ros/ros.h"
#include "clothoid_msgs/Lane.h"
#include "clothoid_msgs/LightStatus.h"
#include "clothoid_msgs/TrafficSignRecognitionStatus.h"
#include "clothoid_msgs/Pedestrian.h"
#include "clothoid_msgs/VehicleDetectStatus.h"

#include "clothoid_msgs/katech_KCAN.h"

class ClothoidMainNode{
private:
    ros::NodeHandle nh;
    ros::Subscriber lane_sub, light_sub, sign_sub, ped_sub, vehicle_sub;
    ros::Publisher rcg_pub,speed_pub;

public:
    void laneCallback( clothoid_msgs::LaneConstPtr lane_msg){}
    void lightCallback( clothoid_msgs::LightStatusConstPtr light_msg){}
    void signCallback( clothoid_msgs::TrafficSignRecognitionStatusConstPtr sign_msg){}
    void pedestrianCallback( clothoid_msgs::PedestrianConstPtr ped_msg){}
    void vehicleCallback( clothoid_msgs::VehicleDetectStatusConstPtr vehicle_msg){}
    ClothoidMainNode(ros::NodeHandle _nh):nh(_nh){
         lane_sub       = nh.subscribe<clothoid_msgs::Lane>("lane", 1, &ClothoidMainNode::laneCallback, this);
         light_sub      = nh.subscribe<clothoid_msgs::LightStatus>("light", 1, &ClothoidMainNode::lightCallback, this);
         sign_sub       = nh.subscribe<clothoid_msgs::TrafficSignRecognitionStatus>("sign", 1, &ClothoidMainNode::signCallback, this);
         ped_sub        = nh.subscribe<clothoid_msgs::Pedestrian>("pedestrian", 1, &ClothoidMainNode::pedestrianCallback, this);
         vehicle_sub    = nh.subscribe<clothoid_msgs::VehicleDetectStatus>("vehicle", 1, &ClothoidMainNode::vehicleCallback, this);

         rcg_pub        = nh.advertise<clothoid_msgs::katech_KCAN>("recognition_signal",1);
         speed_pub      = nh.advertise<clothoid_msgs::katech_KCAN>("vehicle_speed",1);
    }

};
int main (int argc, char **argv)
{
    ros::init(argc, argv, "clothoid_main");
    ros::NodeHandle nh;
    ClothoidMainNode cn(nh);
    ros::spin();
}
