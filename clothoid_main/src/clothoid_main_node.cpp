#include "ros/ros.h"
#include "clothoid_msgs/Lane.h"
#include "clothoid_msgs/LightStatus.h"
#include "clothoid_msgs/TrafficSign.h"
#include "clothoid_msgs/Pedestrian.h"
#include "clothoid_msgs/VehicleDetectStatus.h"

#include "clothoid_msgs/katech_KCAN.h"

#include <string.h>

const char *byte_to_binary(int x)
{
    static char b[9];
    b[0] = '\0';

    int z;
    for (z = 128; z > 0; z >>= 1)
    {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }

    return b;
}
class ClothoidMainNode{
private:
    ros::NodeHandle nh;
    ros::Subscriber lane_sub, light_sub, sign_sub, ped_sub, vehicle_sub;
    ros::Publisher rcg_pub,speed_pub;

public:
    void laneCallback( const clothoid_msgs::LaneConstPtr &lane_msg){
        ROS_INFO("lane received : LaneDeparture %d, LaneStatus %d, eval %lf",lane_msg.get()->LaneDeparture, lane_msg.get()->LaneStatus, lane_msg.get()->eval);
    }
    void lightCallback( const clothoid_msgs::LightStatusConstPtr &light_msg)
    {
        ROS_INFO("light received : %s",byte_to_binary(light_msg.get()->lightOn));
    }
    void signCallback( const clothoid_msgs::TrafficSignConstPtr &sign_msg)
    {
        ROS_INFO("sign received : Vel %d, Bump %d", sign_msg.get()->TrafficSignBump, sign_msg.get()->TrafficSignBump);
    }
    void pedestrianCallback( const clothoid_msgs::PedestrianConstPtr &ped_msg)
    {
        ROS_INFO("pedestrian received : (%d, %d), w :%d, h: %d", ped_msg.get()->x, ped_msg.get()->y, ped_msg.get()->width, ped_msg.get()->height);
    }
    void vehicleCallback( const clothoid_msgs::VehicleDetectStatusConstPtr &vehicle_msg)
    {
        ROS_INFO("vehicle received");
    }
    ClothoidMainNode(ros::NodeHandle _nh):nh(_nh){
         lane_sub       = nh.subscribe<clothoid_msgs::Lane>("lane", 1, &ClothoidMainNode::laneCallback, this);
         light_sub      = nh.subscribe<clothoid_msgs::LightStatus>("light", 1, &ClothoidMainNode::lightCallback, this);
         sign_sub       = nh.subscribe<clothoid_msgs::TrafficSign>("sign", 1, &ClothoidMainNode::signCallback, this);
         ped_sub        = nh.subscribe<clothoid_msgs::Pedestrian>("pedestrian", 1, &ClothoidMainNode::pedestrianCallback, this);
         vehicle_sub    = nh.subscribe<clothoid_msgs::VehicleDetectStatus>("vehicle", 1, &ClothoidMainNode::vehicleCallback, this);

         rcg_pub        = nh.advertise<clothoid_msgs::katech_KCAN>("recognition_signal",1);
         speed_pub      = nh.advertise<clothoid_msgs::katech_KCAN>("vehicle_speed",1);
         ROS_INFO("clothoid_main has initialized. waiting for messages.");
    }

};
int main (int argc, char **argv)
{
    ros::init(argc, argv, "clothoid_main");
    ros::NodeHandle nh;
    ClothoidMainNode cn(nh);
    ros::spin();
}
