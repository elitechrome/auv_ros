#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
int Cb_MIN = 105;
int Cb_MAX = 138;
int Cr_MIN = 142;
int Cr_MAX = 196;

//for harris
Mat src_gray;  // frame의 gray변환 저장용
Mat frame;     // 영상을 받을 프레임
int thresh = 100;  //2차 배포 DB영상의 thresh
char* source_window = "Source image";
char* corners_window = "Corners detected";

void cornerHarris_demo(Mat temp);

void on_trackbar(int, void*)
{
    //This function gets called whenever a
    // trackbar position is changed
}
void createTrackbars(){
    //create window for trackbars


    namedWindow("Trackbars", 0);
    //create memory to store trackbar name on window
    char TrackbarName[50];
    sprintf(TrackbarName, "Cb_MIN", Cb_MIN);
    sprintf(TrackbarName, "Cb_MAX", Cb_MAX);
    sprintf(TrackbarName, "Cr_MIN", Cr_MIN);
    sprintf(TrackbarName, "Cr_MAX", Cr_MAX);
    //create trackbars and insert them into window
    //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
    //the max value the trackbar can move (eg. H_HIGH),
    //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
    //                                  ---->    ---->     ---->
    createTrackbar("Cb_MIN", "Trackbars", &Cb_MIN, Cb_MAX, on_trackbar);
    createTrackbar("Cb_MAX", "Trackbars", &Cb_MAX, Cb_MAX, on_trackbar);
    createTrackbar("Cr_MIN", "Trackbars", &Cr_MIN, Cr_MAX, on_trackbar);
    createTrackbar("Cr_MAX", "Trackbars", &Cr_MAX, Cr_MAX, on_trackbar);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        //ROS_INFO("aa");
        imshow("view", frame);
        createTrackbars();

        ////////////

        Rect traffic_Rect;
        Mat croppedFaceImage;
        //Mat image, grayimg, cannyimg, cornerimg, circle, gauss, hsvimg,blurout;
        Mat blurout;
        Mat frame_color_ycrcb, frame_color_ycrcb_vel;

        cv::Point p1, p2;

        cv::vector<cv::Mat> rgb;

        p1 = cvPoint(0, 0);
        p2 = cvPoint(frame.cols, frame.rows / 2);

        cv::rectangle(frame, p1, p2, cvScalar(255, 0, 0), 1, 8, 0); // 만족하는 경우 사각형을 그려주게 됨 .
        traffic_Rect = cv::Rect(p1.x, p1.y, p2.x - p1.x, p2.y - p1.y);

        croppedFaceImage = frame(traffic_Rect).clone();



        //	imshow("Searching Area", croppedFaceImage);
        //medianBlur(croppedFaceImage,mid_blur, 3);
        blur(croppedFaceImage, blurout, Size(5, 5));

        //imshow("blurout", blurout);

        frame_color_ycrcb = frame;
        frame_color_ycrcb.copyTo(frame_color_ycrcb_vel);

        Mat YCrCb, mask(frame_color_ycrcb.size(), CV_8U, Scalar(0));
        cvtColor(frame_color_ycrcb, YCrCb, CV_BGR2YCrCb);

        //YCrCb 이미지를 각 채널별로 분리
        vector<Mat> planes;

        split(YCrCb, planes);

        //int nr = frame_color_ycrcb.rows;
        int nr = 180;
        int nc = frame_color_ycrcb.cols;
        int count;

        // 170 < Cr <230, 70 <Cb < 130인 영역만 255로 표시해서 mask 만들기

//        for (int i = 0; i<nr; i++){

//            uchar* Cr = planes[1].ptr<uchar>(i);

//            uchar* Cb = planes[2].ptr<uchar>(i);

//            for (int j = 0; j<nc; j++){


//                if ((Cr_MIN<Cr[j] && Cr[j] <Cr_MAX) && (Cb_MIN<Cb[j] && Cb[j]<Cb_MAX)){

//                    mask.at<uchar>(i, j) = 255;
//                    count ++;
//                }

//            }

//        }
        for (int i = 0; i<nr; i++){

            uchar* Cr = planes[1].ptr<uchar>(i);

            uchar* Cb = planes[2].ptr<uchar>(i);

            for (int j = 200; j<600; j++){


                if ((Cr_MIN<Cr[j] && Cr[j] <Cr_MAX) && (Cb_MIN<Cb[j] && Cb[j]<Cb_MAX)){

                    mask.at<uchar>(i, j) = 255;
                    count ++;
                }

            }

        }
        ROS_INFO("red %d",count);

        if(count >20){

            ROS_INFO("red light");
        }


        //가우시안 블러

        GaussianBlur(mask, mask, Size(3, 3), 2, 2);

        imshow("MASK", mask);

        Mat img_erode;

       // erode(mask, img_erode, Mat(3, 3, CV_8U, Scalar(1)), Point(-1, -1), 1);

       // imshow("img_erode", img_erode);

        ////////////////////////////
//        int r,g,b;
//        int count = 0;
//        for(int y =0; y<180; y++){
//            Vec3b* pixel = mask.ptr<Vec3b>(y);
//            for(int x =0; x<mask.cols; x++){
//                r = pixel[x][2];
//                g = pixel[x][1];
//                b = pixel[x][0];
//                if(r>250 && 160 > g && g > 80 && 160 > b && b >80 ){
//                    count++;
//                    ROS_INFO("upupup");
//                }


//            }
//        }
//        ROS_INFO("red2 : %d",count);
        /////////////////

        cornerHarris_demo(frame);
        //////////
        char c = waitKey(500); //low number means fast video playback

        //high number means slow video playback
        if (c == 32)                // 32 == SPACE key
        {
            while ((c = waitKey(10)) != 32 && c != 27);

        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_sample");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("stereo/left/image_raw", 1, imageCallback);
    ros::spin();
    cv::destroyWindow("view");
}

void cornerHarris_demo(Mat temp) // 이 temp는 image watch에서 확인하기 위한 용도임, 코드내에서 사용되지 않음.
{
    vector<Point2f> corners;   //탐지된 코너 저장용 벡터

    Mat dst, dst_norm, dst_norm_scaled; //각 중간 과정을 저장할 Mat변수

    dst = Mat::zeros(src_gray.size(), CV_32FC1);

    /// Detector parameters
    int blockSize = 5;     //It is the size of neighbourhood considered for corner detection
    int kSize = 3;         //Aperture parameter of Sobel derivative used.
    double k = 0.04;       //Harris detector free parameter in the equation.

    cvtColor(frame, src_gray, CV_BGR2GRAY);

    /// Detecting corners
    cornerHarris(src_gray, dst, blockSize, kSize, k, BORDER_DEFAULT);

    /// Normalizing
    normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());

    convertScaleAbs(dst_norm, dst_norm_scaled);


    // 코너를 찾아 vector에 넣는다.
    for (int j = 0; j < dst_norm.rows; j++)
    {
        for (int i = 0; i < dst_norm.cols; i++)
        {
            if ((int)dst_norm.at<float>(j, i) > thresh)
            {
                //찾을 영역을 지정 y값 j , x값 i
                if (j < 100 && 100< i &&  i < 500)
                {
                    circle(dst_norm_scaled, Point(i, j), 2, Scalar(0), 2, 8, 0);
                    circle(frame, Point(i, j), 2, cvScalar(255, 0, 0), 2, 8, 0);

                    Point2f point = Point2f(i, j);

                    corners.push_back(point);
                }
            }
        }//end for
    }//end for


    Point p1, p2;
    Point topCenter;
    int x_length;
    int y_length;
    int size = corners.size();

    for (int i = 0; i < size - 2; i++)
    {
        for (int j = i + 1; j < size - 1; j++)
        {
            p1.x = corners[i].x;
            p1.y = corners[i].y;

            p2.x = corners[j].x;
            p2.y = corners[j].y;

            if (p1.x >= p2.x){
                x_length = p1.x - p2.x;
                topCenter.x = p2.x + (x_length / 2);
            }
            else{
                x_length = p2.x - p1.x;
                topCenter.x = p1.x + (x_length / 2);
            }

            int checkY;
            if (p1.y > p2.y)
            {
                y_length = p1.y - p2.y;
                topCenter.y = p2.y;
            }
            else if (p1.y < p2.y)
            {
                y_length = p2.y - p1.y;
                topCenter.y = p1.y;
            }

            int tempY = topCenter.y + (y_length / 5);


            if (x_length * 3.0 < y_length && y_length < x_length * 5.0 && x_length > 8 && y_length > 20)
            {
//                int check = recognizeLight(topCenter.x, topCenter.y, y_length, temp);
//                if (check == RED)
//                {
//                    globalCount++;
//                    printf("red %d\n", globalCount);
//                }
            }


        }//end for
    }//end for

    cv::imshow(source_window, frame);

    //찾은 코너 보여주기
    imshow(corners_window, dst_norm_scaled);
}
