#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Vision_main.h>
#include <LaneDetection.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
        //cv::imshow("view", img);

        //Control Panel
        cv::namedWindow("Control", CV_WINDOW_AUTOSIZE);
        resizeWindow("Control", 480, 400);
        //Operation Mode Switch
        cv::createTrackbar("Operation Mode", "Control", &Operation_Mode, 3);
        //ROI
        cv::createTrackbar("ROI H", "Control", &ROI_offset_h, 480);
        cv::createTrackbar("ROI H Len", "Control", &ROI_h_len, 480);
        //Lane Detection
        cv::createTrackbar("Binary_th", "Control", &Bin_th, 255);
        //Hough_th
        cv::createTrackbar("Hough_th", "Control", &Hough_th, 100);
        //Detection_Mode
        cv::createTrackbar("Detection_Mode", "Control", &Detection_Mode, 4);
        //Mid line
        cv::createTrackbar("Mid line", "Control", &Mid_pos, 1200);
        //Frame No.
        cv::createTrackbar("Frame", "Control", &Frame_No, 300);


        LaneOut Result_lane;
        m_LaneDetectionClass.ParamSet(ROI_offset_h, ROI_h_len, Bin_th, Mid_pos, Hough_th);
        Result_lane = m_LaneDetectionClass.LaneMain(img, Detection_Mode);
        imshow("results", img);

        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_node");
    ros::NodeHandle nh;
    //cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("mono/image_raw", 1, imageCallback);
    //image_transport::Subscriber sub = it.subscribe("mono/image_raw", 1, imageCallback);
    ros::spin();
    cv::destroyWindow("view");
}
