#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <clothoid_msgs/LaserROIArray.h>
#include <cv_bridge/cv_bridge.h>

class LaserRoiNode{
private:
    ros::NodeHandle n_;
    ros::Subscriber laser_sub_;
    ros::Publisher roi_pub_;
public:
   LaserRoiNode(ros::NodeHandle n) :
   n_(n)
   {
       laser_sub_ = n_.subscribe<sensor_msgs::LaserScan>("lidar/scan", 1, &LaserRoiNode::scanCallback,this);
       roi_pub_ = n_.advertise<clothoid_msgs::LaserROIArray>("lidar/roi",10);
   }

   void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
   {
       float ang_res = scan_in.get()->angle_increment;
       float ang_min= scan_in.get()->angle_min;
       std::vector<float> ranges = scan_in.get()->ranges;
       ////////////////////////////////////////////////////////
       ///Do Clustering!
       /// input : ranges
       /// output : start/end points of each cluster
       ////////////////////////////////////////////////////////


       cv::Mat V1 = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
       cv::Mat U1 = (cv::Mat_<double>(3, 1) << 0, 0, 1);

       // Intrinsic
       double fx = 283.11208;
       double fy = 283.11208;
       double cx = 320;
       double cy = 240;
       cv::Mat intrinsic = (cv::Mat_<double>(3, 3) <<
                            fx,  0, cx,
                             0, fy, cy,
                             0,  0,  1
                        );

        // Extrinsic
       cv::Mat extrinsic = (cv::Mat_<double>(3, 4) <<
                             1,  0,  0, 0,
                             0,  1,  0, 0,
                             0,  0,  1, 0);

       U1 = intrinsic * extrinsic * V1;
       U1/=U1.at<double>(2, 0);

   }

};

int main(int argc, char** argv)
{

 ros::init(argc, argv, "laser_roi_node");
 ros::NodeHandle n;
 LaserRoiNode lsp(n);
 ROS_INFO("laser_roi_node initialized.\n");
 ros::spin();

 return 0;
}
