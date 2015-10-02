#include "ros/ros.h"
#include <clothoid_msgs/katech_KCAN.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <asm/types.h>
#include <unistd.h>
#include <boost/thread.hpp>

#define DWORD  __u32
#define WORD   unsigned short
#define BYTE   unsigned char
#define LPSTR  char*


#include "PCANBasic.h"
#define PCAN_DEVICE		PCAN_USBBUS2

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

class KCANNode{
    ros::NodeHandle nh;
    ros::Publisher kcan_pub;
    ros::Subscriber speed_sub,rcg_sub;
    ros::Timer timer;
    clothoid_msgs::katech_KCAN can_msg;

    TPCANMsg Message;
    TPCANStatus Status;
    int fd;
    fd_set Fds;

public:

    KCANNode(ros::NodeHandle _nh):nh(_nh){
    kcan_pub = nh.advertise<clothoid_msgs::katech_KCAN>("vehicle_status", 1000);
    timer = nh.createTimer(ros::Duration(0.05), &KCANNode::timerCallback,this);
    speed_sub = nh.subscribe<clothoid_msgs::katech_KCAN>("vehicle_speed",1,&KCANNode::speedCallback,this);
    rcg_sub = nh.subscribe<clothoid_msgs::katech_KCAN>("recognition_signal",1, &KCANNode::recognitionCallback, this);
    Status = CAN_Initialize(PCAN_USBBUS1, PCAN_BAUD_500K, 0, 0, 0);
    printf("Initialize CAN: %i\n", (int) Status);


    CAN_GetValue(PCAN_USBBUS1, PCAN_RECEIVE_EVENT, &fd,sizeof(int));

    /* Watch stdin (fd 0) to see when it has input. */
    FD_ZERO(&Fds);
    FD_SET(fd, &Fds);

    boost::thread grab_thread = boost::thread(boost::bind(&KCANNode::publishKCAN, this));
}

    void recognitionCallback(const clothoid_msgs::katech_KCANConstPtr &rcg_msg)
    {
        Message.ID = 0x400; //ID for Recognition Signal
        Message.LEN = 8;
        Message.MSGTYPE = PCAN_MESSAGE_EXTENDED;
        Message.DATA[0]=rcg_msg.get()->Pedestrian<4 | rcg_msg.get()->TrafficLight;
        Message.DATA[1]=rcg_msg.get()->SpeedBump<4 | rcg_msg.get()->LaneDeparture;
        Message.DATA[2]=(rcg_msg.get()->TrafficSign&0x000F)<4 | rcg_msg.get()->PrecedingCar;
        Message.DATA[3]=(rcg_msg.get()->TrafficSign&0xff00)>8 | (rcg_msg.get()->TrafficSign&0x00f0)>4;

        if((Status=CAN_Write(PCAN_DEVICE,&Message)) != PCAN_ERROR_OK){
            ROS_ERROR("CAN Error : Error Sending Recognition Signal.");
            return;
        }
        ROS_DEBUG("ID: %x, CAN DATA[0]: %s", Message.ID, byte_to_binary(Message.DATA[0]));
        ROS_DEBUG("ID: %x, CAN DATA[1]: %s", Message.ID, byte_to_binary(Message.DATA[1]));
        ROS_DEBUG("ID: %x, CAN DATA[2]: %s", Message.ID, byte_to_binary(Message.DATA[2]));
        ROS_DEBUG("ID: %x, CAN DATA[3]: %s", Message.ID, byte_to_binary(Message.DATA[3]));
    }

    void speedCallback(const clothoid_msgs::katech_KCANConstPtr &speed_msg)
    {
        can_msg.vehicle_speed = speed_msg.get()->vehicle_speed;
        ROS_DEBUG("Received Speed :%d [kph]", can_msg.vehicle_speed);

    }
    void timerCallback(const ros::TimerEvent& event)
    {
        Message.ID = 0x420; //ID for Longitudinal control
        Message.LEN = 8;
        Message.MSGTYPE = PCAN_MESSAGE_EXTENDED;
        Message.DATA[0]=can_msg.vehicle_speed;
        switch(can_msg.vehicle_speed){
        case clothoid_msgs::katech_KCAN::VEHICLE_SPEED_0KPH:
        case clothoid_msgs::katech_KCAN::VEHICLE_SPEED_20KPH:
        case clothoid_msgs::katech_KCAN::VEHICLE_SPEED_30KPH:
        case clothoid_msgs::katech_KCAN::VEHICLE_SPEED_40KPH:
        case clothoid_msgs::katech_KCAN::VEHICLE_SPEED_50KPH:
            ROS_DEBUG("Send Speed :%d [kph]", can_msg.vehicle_speed);
            break;
        default:
            ROS_WARN("Vehicle Speed should be 0, 20, 30, 40, 50 [kph]!");
            return;
            break;
        }
        if((Status=CAN_Write(PCAN_DEVICE,&Message)) != PCAN_ERROR_OK){
            ROS_ERROR("CAN Error : Error Sending Vehicle Speed.");
            return;
        }
    }

    void publishKCAN()
    {
        while (select(fd+1, &Fds, NULL, NULL, NULL) > 0) {
            Status = CAN_Read(PCAN_USBBUS1, &Message, NULL);
            if (Status != PCAN_ERROR_OK) {
                printf("Error 0x%x\n", (int) Status);
                break;
            }

    //		printf("  - R ID:%4x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x %02x %02x\n",
    //				(int) Message.ID, (int) Message.LEN, (int) Message.DATA[0],
    //				(int) Message.DATA[1], (int) Message.DATA[2],
    //				(int) Message.DATA[3], (int) Message.DATA[4],
    //				(int) Message.DATA[5], (int) Message.DATA[6],
    //				(int) Message.DATA[7]);
            switch((int) Message.ID){
            case 0x200://for longitudinal_status
                can_msg.CurrentVehicleSpeed = (int) Message.DATA[2];
                can_msg.CurrentVehicleAcceleration = (int)(Message.DATA[0]|(Message.DATA[1]<8))*0.01-10.23;
                ROS_DEBUG("CurrentVehicleSpeed : %d, CurrentVehicleAcc : %f", can_msg.CurrentVehicleSpeed, can_msg.CurrentVehicleAcceleration);
                break;
    //        case 0x102://Clothoid vehicle speed
    //            can_msg.CurrentVehicleSpeed = (int) Message.DATA[3];
    //            ROS_INFO("Clothiod CurrentVehicleSpeed : %d", can_msg.CurrentVehicleSpeed);
    //            break;
            default:
                break;
            }
            kcan_pub.publish(can_msg);
        }
    }
};
int main(int argc, char* argv[]) 
{
    ros::init(argc, argv, "katech_pcan_node");
    ros::NodeHandle nh;
    KCANNode kn(nh);
    ros::spin();
	return 0;
}

