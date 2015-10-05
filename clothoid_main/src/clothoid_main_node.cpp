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
    ros::Timer timer;
    clothoid_msgs::katech_KCAN pub_msg;
    clothoid_msgs::katech_KCAN pub_vel;

public:
    ClothoidMainNode(ros::NodeHandle _nh):nh(_nh){
         lane_sub       = nh.subscribe<clothoid_msgs::Lane>("lane", 1, &ClothoidMainNode::laneCallback, this);
         light_sub      = nh.subscribe<clothoid_msgs::LightStatus>("light", 1, &ClothoidMainNode::lightCallback, this);
         sign_sub       = nh.subscribe<clothoid_msgs::TrafficSign>("sign", 1, &ClothoidMainNode::signCallback, this);
         ped_sub        = nh.subscribe<clothoid_msgs::Pedestrian>("pedestrian", 1, &ClothoidMainNode::pedestrianCallback, this);
         vehicle_sub    = nh.subscribe<clothoid_msgs::VehicleDetectStatus>("vehicle", 1, &ClothoidMainNode::vehicleCallback, this);
         rcg_pub        = nh.advertise<clothoid_msgs::katech_KCAN>("recognition_signal",1);
         speed_pub      = nh.advertise<clothoid_msgs::katech_KCAN>("vehicle_speed",1);
         timer          = nh.createTimer(ros::Duration(0.001), &ClothoidMainNode::timerCallback, this);
         ROS_INFO("clothoid_main has initialized. waiting for messages.");
    }

    void laneCallback( const clothoid_msgs::LaneConstPtr &lane_msg){
        ROS_INFO("lane received : LaneDeparture %d, LaneStatus %d, eval %lf",lane_msg.get()->LaneDeparture, lane_msg.get()->LaneStatus, lane_msg.get()->eval);
        if(lane_msg.get()->LaneDeparture != 1){
            pub_msg.LaneDeparture=1;

        }
        else{
            pub_msg.LaneDeparture=0;
        }
        rcg_pub.publish(pub_msg);
        pub_msg.LaneDeparture=0;
    }
    void lightCallback( const clothoid_msgs::LightStatusConstPtr &light_msg)
    {
        ROS_INFO("light received : %s",byte_to_binary(light_msg.get()->lightOn));
        pub_msg.TrafficLight =0;
        if(light_msg.get()->lightOn == clothoid_msgs::LightStatus::RED ){
            pub_msg.TrafficLight |= clothoid_msgs::LightStatus::RED;
            pub_vel.vehicle_speed=clothoid_msgs::katech_KCAN::VEHICLE_SPEED_0KPH;
        }
        if(light_msg.get()->lightOn == clothoid_msgs::LightStatus::LEFT ){
            pub_msg.TrafficLight |= clothoid_msgs::LightStatus::LEFT;}
        if(light_msg.get()->lightOn == clothoid_msgs::LightStatus::RIGHT ){
            pub_msg.TrafficLight |= clothoid_msgs::LightStatus::RIGHT;}
        if(light_msg.get()->lightOn == clothoid_msgs::LightStatus::GREEN ){
            pub_msg.TrafficLight |= clothoid_msgs::LightStatus::GREEN;}

        speed_pub.publish(pub_vel);
        rcg_pub.publish(pub_msg);
        pub_msg.TrafficLight =0;
    }
    void signCallback( const clothoid_msgs::TrafficSignConstPtr &sign_msg)
    {
        ROS_INFO("sign received : Vel %d, Bump %d", sign_msg.get()->TrafficSignBump, sign_msg.get()->TrafficSignBump);

        if(sign_msg.get()->TrafficSignBump == 1){
            pub_msg.SpeedBump|=1;
            pub_vel.vehicle_speed = clothoid_msgs::katech_KCAN::VEHICLE_SPEED_20KPH;
        }
        switch(sign_msg.get()->TrafficSignVel){
        case clothoid_msgs::TrafficSign::SIGN_VEL20:
            pub_msg.TrafficSign |= 0x0001;
            pub_vel.vehicle_speed = clothoid_msgs::katech_KCAN::VEHICLE_SPEED_20KPH;
            break;
        case clothoid_msgs::TrafficSign::SIGN_VEL30:
            pub_msg.TrafficSign |= 0x0002;
            pub_vel.vehicle_speed = clothoid_msgs::katech_KCAN::VEHICLE_SPEED_30KPH;
            break;
        case clothoid_msgs::TrafficSign::SIGN_VEL40:
            pub_msg.TrafficSign |= 0x0004;
            pub_vel.vehicle_speed = clothoid_msgs::katech_KCAN::VEHICLE_SPEED_40KPH;
            break;
        case clothoid_msgs::TrafficSign::SIGN_VEL50:
            pub_msg.TrafficSign |= 0x0008;
            pub_vel.vehicle_speed = clothoid_msgs::katech_KCAN::VEHICLE_SPEED_50KPH;
            break;
        case clothoid_msgs::TrafficSign::SIGN_VEL60:
            pub_msg.TrafficSign |= 0x0010;
            pub_vel.vehicle_speed = clothoid_msgs::katech_KCAN::VEHICLE_SPEED_50KPH;
            break;
        case clothoid_msgs::TrafficSign::SIGN_VEL70:
            pub_msg.TrafficSign |= 0x0020;
            pub_vel.vehicle_speed = clothoid_msgs::katech_KCAN::VEHICLE_SPEED_50KPH;
            break;
        case clothoid_msgs::TrafficSign::SIGN_VEL80:
            pub_msg.TrafficSign |= 0x0040;
            pub_vel.vehicle_speed = clothoid_msgs::katech_KCAN::VEHICLE_SPEED_50KPH;
            break;
        case clothoid_msgs::TrafficSign::SIGN_VEL90:
            pub_msg.TrafficSign |= 0x0080;
            pub_vel.vehicle_speed = clothoid_msgs::katech_KCAN::VEHICLE_SPEED_50KPH;
            break;
        case clothoid_msgs::TrafficSign::SIGN_VEL100:
            pub_msg.TrafficSign |= 0x0100;
            pub_vel.vehicle_speed = clothoid_msgs::katech_KCAN::VEHICLE_SPEED_50KPH;
            break;
        case clothoid_msgs::TrafficSign::SIGN_VEL110:
            pub_msg.TrafficSign |= 0x0200;
            pub_vel.vehicle_speed = clothoid_msgs::katech_KCAN::VEHICLE_SPEED_50KPH;
            break;
        default:
            break;
        }
        speed_pub.publish(pub_vel);
        rcg_pub.publish(pub_msg);
        pub_msg.TrafficSign = 0;
        pub_msg.SpeedBump &= 2;
    }
    void pedestrianCallback( const clothoid_msgs::PedestrianConstPtr &ped_msg)
    {
        ROS_INFO("pedestrian received : (%d, %d), w :%d, h: %d", ped_msg.get()->x, ped_msg.get()->y, ped_msg.get()->width, ped_msg.get()->height);
        pub_msg.Pedestrian=1;
        rcg_pub.publish(pub_msg);
        pub_msg.Pedestrian=0;
    }
    void vehicleCallback( const clothoid_msgs::VehicleDetectStatusConstPtr &vehicle_msg)
    {
        ROS_INFO("vehicle received");
        pub_msg.PrecedingCar=1;
        rcg_pub.publish(pub_msg);
        pub_msg.PrecedingCar=0;
    }
    void timerCallback(const ros::TimerEvent&)
    {
        //check mission sequences
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
