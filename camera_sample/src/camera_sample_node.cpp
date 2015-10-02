#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <string>
#include "BlobLabeling.h"
#include <cv.h>
#include <highgui.h>

#include <stack>

#include <vector>





//#define SIMPLE_MATCH

#define CONTOUR_MIN 300
using namespace cv;
using namespace std;

double minVal;
double maxVal;

int count=0;

int thresh = 100;
int max_thresh = 255;
int match_method;
int max_Trackbar = 5;

Point minLoc;
Point maxLoc;
Point matchLoc;


void countnum(Mat img);
/// Global Variables
Mat img;
Mat templ;
Mat result;

Mat img_rgb, img_gray, canny_output, drawing; //»ï°¢Çü
Mat triangle_ROI;
Mat frame_color_ycrcb_tri;
Mat Labeling(Mat *src);



Mat dsd;
cv::Mat Capframe_one;
cv::Mat Canny_image;
//Get the corners from the object
Mat hough_image;
cv::Mat frame_color_ycrcb;
Mat frame_color_ycrcb_vel;





void MatchingMethod(int, void*);
void drawStuff();
void drawAllTriangles(Mat&, const vector< vector<Point> >&);
void readme();
void canny(cv::Mat& img2, cv::Mat& out) {

    // Convert to gray
    cv::cvtColor(img, out, CV_BGR2GRAY);
    // Compute Canny edges
    cv::Canny(out, out, 100, 200);
    // Invert the image
    cv::threshold(out, out, 128, 255, cv::THRESH_BINARY_INV);
}
void OCR_Text();
#ifndef SIMPLE_MATCH
void MatchingMethod(int, void*)
{
    /// Source image to display
    Mat img_display;
    img.copyTo(img_display);
    cvtColor(img,img,CV_YCrCb2BGR);
    int i, j, x, y, key;

    Point tempLoc;

    Mat graySourceImage = Mat(img.rows, img.cols, CV_8UC3);
    Mat grayTemplateImage = Mat(templ.rows, templ.cols, CV_8UC3);
    Mat binarySourceImage = Mat(img.rows, img.cols, CV_8UC3);
    Mat binaryTemplateImage = Mat(templ.rows, templ.cols, CV_8UC3);

    img.copyTo(img_display);

    binarySourceImage=img;
    binaryTemplateImage=templ;
    int templateHeight = templ.rows;
    int templateWidth = templ.cols;
    //printf("img.cols:%d \n,img.rows:%d\n,templ.rows:%d \n,templ.cols:% d\n",img.cols,img.rows,templ.rows,templ.cols);

    float templateScale = 0.5f;

    for (i = 2; i <= 3; i++)
    {
        int tempTemplateHeight = (int)(templateWidth * (i * templateScale));
        int tempTemplateWidth = (int)(templateHeight * (i * templateScale));
        Mat tempBinaryTemplateImage(Size(tempTemplateWidth, tempTemplateHeight), CV_8UC3);

        // W - w + 1, H - h + 1
        int result_cols = img.cols -templ.cols+1 ;
        int result_rows = img.rows -templ.rows+1;

        result.create(result_rows, result_cols, CV_32FC1);

        //resize(binaryTemplateImage, tempBinaryTemplateImage, tempBinaryTemplateImage.size(), 0, 0, CV_INTER_LINEAR);
        int angle_div = 1;
        float degree = 180.0/angle_div;
        Mat rotateBinaryTemplateImage(Size(tempBinaryTemplateImage.cols, tempBinaryTemplateImage.rows), CV_8UC3);

        /*for (j = 0; j <= angle_div; j++)
        {
            for (y = 0; y < tempTemplateHeight; y++){
                for (x = 0; x < tempTemplateWidth; x++){
                    rotateBinaryTemplateImage.data[y * tempTemplateWidth + x] = 255;
                }
            }
            for (y = 0; y < tempTemplateHeight; y++){
                for (x = 0; x < tempTemplateWidth; x++){
                    float radian = (float)j * degree * CV_PI / 180.0f;
                    int scale = y * tempTemplateWidth + x;

                    int rotateY = -sin(radian) * ((float)x - (float)tempTemplateWidth / 2.0f) + cos(radian) * ((float)y - (float)tempTemplateHeight / 2.0f) + tempTemplateHeight / 2;

                    int rotateX = cos(radian) * ((float)x - (float)tempTemplateWidth / 2.0f) + sin(radian) * ((float)y - (float)tempTemplateHeight / 2.0f) + tempTemplateWidth / 2;

                    if (rotateY < tempTemplateHeight && rotateX < tempTemplateWidth && rotateY >= 0 && rotateX >= 0)
                        rotateBinaryTemplateImage.data[scale] = tempBinaryTemplateImage.data[rotateY * tempTemplateWidth + rotateX];
                }
            }


        }*/

        matchTemplate(binarySourceImage, rotateBinaryTemplateImage, result, CV_TM_SQDIFF_NORMED);


    //	normalize(result, result, -1, 1, NORM_MINMAX, -1, Mat());

   minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

        /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
        if (match_method == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED){

          matchLoc = minLoc;
        }
        else{
        matchLoc = maxLoc;
        }

    //        printf("ÅÛÇÃžŽ œºÄÉÀÏ %d%%,¢ªÀÇ ÃÖŽë À¯»çµµ : %f%%\n", (int)(i * 0.5 * 100), (1 - minVal) * 100);

        if (minVal < 0.5){ // 1 - 0.065 = 0.935 : 93.5
            printf("방지표지\n");

         //  rectangle(img_display, matchLoc, Point( matchLoc.x + templ.cols,  matchLoc.y + templ.rows), Scalar(0, 255, 0), 2, 8, 0);
          //rectangle(result, matchLoc, Point( matchLoc.x + templ.cols,  matchLoc.y + templ.rows), Scalar::all(0), 2, 8, 0);


        }


    }
    //imshow(templ_window, templ);

   // imshow("image_window", img_display);
    //imshow("result_window", result);

    return;
}
#endif
#ifdef SIMPLE_MATCH
void MatchingMethod(int, void*)
{
    /// Source image to display
    Mat img_display;
    img.copyTo(img_display);

    /// Create the result matrix
    int result_cols = img.cols - templ.cols + 1;
    int result_rows = img.rows - templ.rows + 1;

    result.create(result_rows, result_cols, CV_32FC1);

    /// Do the Matching and Normalize
    matchTemplate(img, templ, result, match_method);
    normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());

    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;

    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    if (match_method == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED)
    {
        matchLoc = minLoc;
    }
    else
    {
        matchLoc = maxLoc;

    }

    /// Show me what you got
    rectangle(img_display, matchLoc, Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), Scalar::all(0), 2, 8, 0);
    rectangle(result, matchLoc, Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), Scalar::all(0), 2, 8, 0);

    //imshow(templ_window, templ);
    imshow(image_window, img_display);
    imshow(result_window, result);


    return;
}
#endif
void readme()
{
    std::cout << " Usage: ./TemplateMatching <template> <target>" << std::endl;
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat img21 = cv_bridge::toCvShare(msg, "bgr8")->image;
        //cv::imshow("view",img21 );

        frame_color_ycrcb = cv::Mat::zeros(480, 640 ,CV_8UC3);
        ///////

    //Get the corners from the object

         Capframe_one =img21;


         imshow("dsd",Capframe_one);
      //  if(Capframe_one.empty()) break;

       //	Capframe_one=imread("RR.jpg");




            frame_color_ycrcb=Capframe_one;
                frame_color_ycrcb_vel=frame_color_ycrcb.clone();
            frame_color_ycrcb.copyTo(frame_color_ycrcb_tri);
            //YCrCbÀÌ¹ÌÁö·Î º¯°æ

            Mat YCrCb, mask(frame_color_ycrcb.size(), CV_8U, Scalar(0));

            cvtColor(frame_color_ycrcb, YCrCb, CV_BGR2YCrCb);



            //YCrCb ÀÌ¹ÌÁöžŠ °¢ Ã€³Îº°·Î ºÐž®

            vector<Mat> planes;

            split(YCrCb, planes);

            int nr=frame_color_ycrcb.rows;

            int nc=frame_color_ycrcb.cols;



            // 170 < Cr <230, 70 <Cb < 130ÀÎ ¿µ¿ªžž 255·Î Ç¥œÃÇØŒ­ mask žžµé±â

            for(int i=0; i<nr; i++){

                uchar* Cr=planes[1].ptr<uchar>(i);

                uchar* Cb=planes[2].ptr<uchar>(i);

                for(int j=0; j<nc;j++){
                   //if((140<Cr[j] && Cr[j] <240) && (50<Cb[j] && Cb[j]<150) )

                   // if((170<Cr[j] && Cr[j] <250) && (70<Cb[j] && Cb[j]<180) )

                    if((130<Cr[j] && Cr[j] <220) && (40<Cb[j] && Cb[j]<140) )

                        mask.at<uchar>(i, j)=255;

                }

            }

            GaussianBlur(mask, mask, Size(3, 3), 2, 2);

           imshow("MASK", mask);


         img_gray=mask;


        drawStuff();


            vector<Vec3f> circles;

            std::vector<cv::Vec3f> circles2;
            Mat abcd;

                    cvtColor(frame_color_ycrcb_vel, abcd, CV_BGR2GRAY);
     //       HoughCircles(mask, circles, CV_HOUGH_GRADIENT, 2, 1000,20,95,2,50);
     HoughCircles(abcd, circles, CV_HOUGH_GRADIENT, 2,frame_color_ycrcb_vel.rows/4,200,100,2,50);



            std::vector<cv::Vec3f>::iterator itc_R = circles.begin();
            double count_circle=0;
            while (itc_R != circles.end()) {
                count_circle++;
              //  std::cout << count_circle << std::endl;



             ++itc_R;

            }






            for(int i=0; i<circles.size(); i++)
            {

                Point center((circles[i][0]), (circles[i][1]));
                int radius=(circles[i][2]);

                double b=-70;
                Rect ROI_Rect((double)circles[i][0] - (double)circles[i][2]-b/(double)circles[i][2], (double)circles[i][1] - (double)circles[i][2]-b/(double)circles[i][2], (double)circles[i][2] * 2+b*2/(double)circles[i][2], (double)circles[i][2] * 2+b*2/(double)circles[i][2]);

            //	std::cout << "rect_draw" << std::endl;


           // printf("one dectect");
               // circle(frame_color_ycrcb, center, radius, Scalar(0,255,0), 1);




                if( ((circles[i][0] - circles[i][2]-100)>=0 )&&
                    ((circles[i][0] + circles[i][2]+100)<=640)&&
                    ((circles[i][1] + circles[i][2]+100)<=480)&&
                    ((circles[i][1] - circles[i][2]-100)>=0))

                {
                Mat Temp_ROI,circle_ROI,Temp_ROI_vel;



                frame_color_ycrcb_vel(ROI_Rect).copyTo(Temp_ROI_vel);
                cvtColor(Temp_ROI_vel,Temp_ROI_vel,CV_YCrCb2BGR);

                Mat Temp_vel_gray;

                Mat Temp_vel_gray_ROI;


                double c=12;
                double d=1.35;
                    if(((double)circles[i][2])>=35.0){
                    d=1;
                    }

                    if(((double)circles[i][2])<=21.0&&((double)circles[i][2])>=17.0){
                    d=1;
                    }

                Rect ROI_Rect_vel(((double)(circles[i][0])-((double)(circles[i][2]))),
                                  ((double)(circles[i][1])-((double)(circles[i][2]))),
                                  (double)(circles[i][2])*2,
                                  ((double)(circles[i][2]))*2);


        vector<Rect> Rect_Vector;


                if((0<=((double)(circles[i][0])-((double)(circles[i][2])/1.414213)+((double)(circles[i][2])/1.414213)*2)<=640 )&&
                   (0<=((double)(circles[i][0])-((double)(circles[i][2])/1.414213)+((double)(circles[i][2])/1.414213)*2)>=0 )&&
                   ((((double)(circles[i][1])-((double)(circles[i][2])/1.414213))+((double)(circles[i][2])/1.414213)*2)<=480)&&
                   ((((double)(circles[i][1])-((double)(circles[i][2])/1.414213))+((double)(circles[i][2])/1.414213)*2)>=0))

                {

                    Mat vel_ocr;

                     Temp_vel_gray_ROI=frame_color_ycrcb_vel.clone();
                     Mat 	Temp_vel_gray_ROI_2;
                int x, y, x2, y2;
                if (radius > 25){
                    x = cvRound(circles[i][0]) - (radius*0.8); // ¹ÝÁöž§ÀÌ 25ºžŽÙ Å«°æ¿ì
                    y = cvRound(circles[i][1]) - (radius*0.6);
                    if (x > 0 && y > 0){
                        Temp_vel_gray_ROI_2 = Temp_vel_gray_ROI(Rect(x, y, 1.6 * radius, 1.2*radius));
                        imshow("ROI", Temp_vel_gray_ROI_2);

                    }
                }
                else{
                    x = cvRound(circles[i][0]) - (radius*0.8); // ¹ÝÁöž§ÀÌ 25ºžŽÙ Å«°æ¿ì
                    y = cvRound(circles[i][1]) - (radius*0.6);
                    if (x > 0 && y > 0){
                        Temp_vel_gray_ROI_2= Temp_vel_gray_ROI(Rect(x, y, 1.8 * radius, 1.4*radius));
                        imshow("ROI", Temp_vel_gray_ROI_2);
                    }
                }



                if (x > 0 && y > 0){
                    if (2 * radius < 80){
            CBlobLabeling blob;
            //cvtColor(Temp_vel_gray_ROI,Temp_vel_gray_ROI,CV_BGR2GRAY);

            IplImage *out = new IplImage(Temp_vel_gray_ROI_2);
            IplImage * gray=cvCreateImage(cvSize(out->width,out->height),8,1);

            cvCvtColor(out,gray,CV_BGR2GRAY);
            cvThreshold(gray,gray,126,255,CV_THRESH_BINARY_INV);
            //cvThreshold(gray,gray,126,255,CV_THRESH_BINARY|CV_THRESH_OTSU);

            IplImage * labeled=cvCreateImage(cvSize(out->width,out->height),8,3);
            cvCvtColor(gray,labeled,CV_GRAY2BGR);


            blob.SetParam(gray, 25);
            blob.DoLabeling();

                        int x_min = 100;
                        int x_max = 0;
                        int y_min = 100;
                        int y_max = 0;

                    printf("라벨링 개수: %d\n", blob.m_nBlobs);

                    for (int i = 0; i < blob.m_nBlobs; i++)
                    {


                        CvPoint pt1 = cvPoint(blob.m_recBlobs[i].x, blob.m_recBlobs[i].y);
                        CvPoint pt2 = cvPoint(pt1.x + blob.m_recBlobs[i].width, pt1.y + blob.m_recBlobs[i].height);
                            if (x_max < pt2.x) x_max = pt2.x;
                            if (x_min > pt1.x) x_min = pt1.x;
                            if (y_max < pt2.y) y_max = pt2.y;
                            if (y_min > pt1.y) y_min = pt1.y;

                    }
                //	printf("%d : %d %d %d %d\n", blob.m_nBlobs, x_max, x_min, y_max, y_min);
                        // °¢ ·¹ÀÌºí Ç¥œÃ


                        CvPoint pt1 = cvPoint(x_min, y_min);
                        CvPoint pt2 = cvPoint(x_min + (x_max - x_min), y_min + (y_max - y_min));

                        CvScalar color = cvScalar(0, 0, 255);
                        //cvDrawRect(labeled, pt1, pt2, color);
                  Rect rect1(1+pt1.x, 1+pt1.y, pt2.x-1, pt2.y-1);
                   if(     rect1.width >= 0 && rect1.height >= 0 && rect1.x < labeled->width && rect1.y < labeled->height && rect1.x + rect1.width >= (int)(rect1.width > 0) && rect1.y + rect1.height >= (int)(rect1.height > 0)){
                        cvSetImageROI(labeled, rect1);


                        //printf("%d \n", blob.m_nBlobs);
                        Mat labeling(labeled);
                        cvtColor(labeling, labeling, CV_RGB2GRAY);
                        threshold(labeling, labeling, 100, 255, 0);
                        countnum(labeling);
                        //imshow("temp_vel_gray_ROI",Temp_vel_gray_ROI);
                        imshow("labeling", labeling);
                        //waitKey(0);

                        cvReleaseImage(&labeled);
}



                }

}
                }


         }


       }

            imshow("IMAGE", frame_color_ycrcb);




        //////










    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void countnum(Mat img){
    int count[6] = { 0, 0, 0, 0, 0, 0 };
    int j_min = 2;
    /////////////////////////°¡·Î
    for (int j = j_min; j < img.cols - 1; j++){
        if (img.at<uchar>(img.rows / 4, j) != img.at<uchar>(img.rows / 4, j + 1)){
            count[0]++;
        }
    }
    for (int j = j_min; j < img.cols - 1; j++){
        if (img.at<uchar>(img.rows / 2, j) != img.at<uchar>(img.rows / 2, j + 1)){
            count[1]++;
        }
    }

    for (int j = j_min; j < img.cols - 1; j++){
        if (img.at<uchar>((img.rows / 2) + (img.rows / 4), j) != img.at<uchar>((img.rows / 2) + (img.rows / 4), j + 1)){
            count[2]++;
        }
    }

    ////////////////////////// ŒŒ·Î
    for (int j = j_min; j < img.rows - 1; j++){
        if (img.at<uchar>(j, img.cols / 4) != img.at<uchar>(j + 1, img.cols / 4)){
            count[3]++;
        }
    }

    for (int j = j_min; j < img.rows - 1; j++){
        if (img.at<uchar>(j, img.cols / 2) != img.at<uchar>(j + 1, img.cols / 2)){
            count[4]++;
        }
    }

    for (int j = j_min; j < img.rows - 1; j++){
        if (img.at<uchar>(j, (img.cols / 2) + (img.cols / 4)) != img.at<uchar>(j + 1, (img.cols / 2) + (img.cols / 4))){
            count[5]++;
        }
    }

    /*
    printf("°¡·Î ¹Ù²ï ŒýÀÚŽÂ : %d \n", count[0]);
    printf("°¡·Î ¹Ù²ï ŒýÀÚŽÂ : %d \n", count[1]);
    printf("°¡·Î¹Ù²ï ŒýÀÚŽÂ : %d \n", count[2]);
    printf("ŒŒ·Î ¹Ù²ï ŒýÀÚŽÂ : %d \n", count[3]);
    printf("ŒŒ·Î ¹Ù²ï ŒýÀÚŽÂ : %d \n", count[4]);
    printf("ŒŒ·Î ¹Ù²ï ŒýÀÚŽÂ : %d \n", count[5]);*/

    if ((count[0] >= 6 && count[0] <= 8) && (count[1] >= 6 && count[1] <= 7) && (count[2] >= 7 && count[2] <= 8) &&
        (count[3] >= 0 && count[3] <= 0) && (count[4] >= 0 && count[4] <= 1) && (count[5] >= 3 && count[5] <= 3)) printf("속도표지판: 110\n");
    else if ((count[0] >= 8 && count[0] <= 8) && (count[1] >= 9 && count[1] <= 9) && (count[2] >= 9 && count[2] <= 9) &&
        (count[3] >= 2 && count[3] <= 2) && (count[4] >= 3 && count[4] <= 3) && (count[5] >= 3 && count[5] <= 3)) printf("속도표지판: 100\n");
    else if ((count[0] >= 7 && count[0] <= 7) && (count[1] >= 7 && count[1] <= 7) && (count[2] >= 5 && count[2] <= 6) &&
        (count[3] >= 4 && count[3] <= 5) && (count[4] >= 0 && count[4] <= 0) && (count[5] >= 2 && count[5] <= 2)) printf("속도표지판: 90\n");
    else if ((count[0] >= 7 && count[0] <= 7) && (count[1] >= 7 && count[1] <= 7) && (count[2] >= 7 && count[2] <= 7) &&
        (count[3] >= 5 && count[3] <= 5) && (count[4] >= 2 && count[4] <= 2) && (count[5] >= 3 && count[5] <= 3)) printf("속도표지판: 80\n");
    else if ((count[0] >= 6 && count[0] <= 6) && (count[1] >= 6 && count[1] <= 6) && (count[2] >= 5 && count[2] <= 6) &&
        (count[3] >= 3 && count[3] <= 3) && (count[4] >= 0 && count[4] <= 2) && (count[5] >= 3 && count[5] <= 3)) printf("속도표지판: 70\n");
    else if ((count[0] >= 5 && count[0] <= 5) && (count[1] >= 7 && count[1] <= 7) && (count[2] >= 7 && count[2] <= 7) &&
        (count[3] >= 4 && count[3] <= 5) && (count[4] >= 0 && count[4] <= 2) && (count[5] >= 2 && count[5] <= 3)) printf("속도표지판: 60\n");
    else if ((count[0] >= 5 && count[0] <= 6) && (count[1] >= 6 && count[1] <= 7) && (count[2] >= 7 && count[2] <= 7) &&
        (count[3] >= 4 && count[3] <= 5) && (count[4] >= 0 && count[4] <= 0) && (count[5] >= 2 && count[5] <= 3)) printf("속도표지판: 50\n");
    else if ((count[0] >= 7 && count[0] <= 7) && (count[1] >= 6 && count[1] <= 6) && (count[2] >= 7 && count[2] <= 7) &&
        (count[3] >= 5 && count[3] <= 5) && (count[4] >= 0 && count[4] <= 0) && (count[5] >= 3 && count[5] <= 3)) printf("속도표지판: 30\n");
    else if ((count[0] >= 7 && count[0] <= 7) && (count[1] >= 6 && count[1] <= 6) && (count[2] >= 5 && count[2] <= 5) &&
        (count[3] >= 4 && count[3] <= 5) && (count[4] >= 0 && count[4] <= 2) && (count[5] >= 2 && count[5] <= 3)) printf("속도표지판: 20\n");
    else printf("속도표지판이 아닙니다..\n");

}
void drawStuff()
{

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;


    Canny(img_gray, canny_output, thresh, thresh * 2, 3);
    //imshow("Canny", canny_output);

    findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    drawing = Mat::zeros(canny_output.size(), CV_8UC3);
    drawAllTriangles(drawing, contours);

    //imshow("Triangles", drawing);
}
// »ï°¢ÇüÀ» Ã£ŽÂ ÇÔŒö
void drawAllTriangles(Mat& img1, const vector< vector<Point> >& contours)
{
    vector<Point> approxTriangle;
    vector<Point> centers;
    Point center_temp;

    for (size_t i = 0; i < contours.size(); i++){
        approxPolyDP(contours[i], approxTriangle, arcLength(Mat(contours[i]), true)*0.05, true);

        if (approxTriangle.size() == 3)
        {

                if(contourArea(contours[i]) > CONTOUR_MIN){
        //	drawContours(img, contours, i, Scalar(255, 255, 255), CV_FILLED); // fill GREEN
            vector<Point>::iterator vertex;
            int count=0;
            Point a_point,b_point,c_point;
            center_temp.x=0;
            center_temp.y=0;


            for (vertex = approxTriangle.begin(); vertex != approxTriangle.end(); ++vertex)
            {
                //circle(img, *vertex, 3, Scalar(0, 0, 255), 1);
                    center_temp.x += vertex->x;
                    center_temp.y += vertex->y;
                    count=count+1;
                    //printf("vertex : %d\n",count);
                    if(count==1)
                    {
                    //   printf("count1\n");

                        a_point.x=vertex->x;
                        a_point.y=vertex->y;

                    }

                    if(count==2)
                    {
                        //     printf("count2\n");

                        b_point.x=vertex->x;
                        b_point.y=vertex->y;

                    }

                    if(count==3)
                    {
                         //    printf("count3\n");

                        c_point.x=vertex->x;
                        c_point.y=vertex->y;
                    }


            }

                  double a_b=sqrt(((double)a_point.x-(double)b_point.x)*((double)a_point.x-(double)b_point.x)+((double)a_point.y-(double)b_point.y)*((double)a_point.y-(double)b_point.y));
                  double b_c=sqrt(((double)b_point.x-(double)c_point.x)*((double)b_point.x-(double)c_point.x)+((double)b_point.y-(double)c_point.y)*((double)b_point.y-(double)c_point.y));
                  double c_a=sqrt(((double)c_point.x-(double)a_point.x)*((double)c_point.x-(double)a_point.x)+((double)c_point.y-(double)a_point.y)*((double)c_point.y-(double)a_point.y));
                  double a_b_c= a_b+b_c+c_a;

                 // printf("a_b%lf\n",a_b);
                //  printf("b_c%lf\n",b_c);
                //  printf("c_a%lf\n",c_a);
                //  printf("a_b_c%lf\n",a_b_c);

                  double a_b_c3=a_b_c/3;

                center_temp.x /= 3;
                center_temp.y /= 3;
                centers.push_back(center_temp);
              if(((a_b_c3+a_b_c3*5/100)>a_b&&a_b>(a_b_c3-a_b_c3*5/100))&&((a_b_c3+a_b_c3*5/100)>b_c&&b_c>(a_b_c3-a_b_c3*5/100))&&((a_b_c3+a_b_c3*5/100)>c_a&&c_a>(a_b_c3-a_b_c3*5/100)) )

              {
                  printf("triangle detect\n");

                  //printf("im center point\n");
                //circle(drawing, center_temp, 4, Scalar(255, 0, 0), 1);

                //	printf("(1)x (0):%f, x (640): %f,  y (0):%f  ,y (480):%f",(center_temp.x - a_b_c3*3/2-10),(center_temp.x - a_b_c3*3/2-10),( center_temp.y - a_b_c3*3/2-10),( center_temp.y - a_b_c3*3/2-10));
                //  printf("(2)x (0):%f, x (640): %f,  y (0):%f  ,y (480):%f",center_temp.x-a_b_c3*3/2-10,(center_temp.x - a_b_c3*3/2-10),a_b_c3*3,a_b_c3*3);

            if( ((center_temp.x - a_b_c3/2-20)>=0 )&&
                ((center_temp.x + a_b_c3/2+20)<=640 )&&
                ((center_temp.y + a_b_c3/2*1.73205/2+20*1.73205/2)<=480)&&
                ((center_temp.y - a_b_c3/2*1.73205/2-20*1.73205/2)>=0))
            {
           //printf("in it");
                Rect triangle_Rect((double)center_temp.x-(double)a_b_c3/2-5,
                                   (double)center_temp.y-(double)a_b_c3/2*1.73205/2-5/2*1.73205,
                                   (double)a_b_c3+10,
                                   (double)a_b_c3/2*1.73205+10/2*1.73205);
                Mat triangle_ROI;
                frame_color_ycrcb_tri(triangle_Rect).copyTo(triangle_ROI);

                img=triangle_ROI;
                imshow("triagle",triangle_ROI);



//printf(":%f:,:%f:\n",(double)(a_b_c3),(double)(a_b_c3/2.0*1.73205));
//printf(":%f:,:%f:\n",img.rows,img.cols);

                  if(img.empty()){ break;}

                  else{
                  resize(templ, templ, Size((double)(a_b_c3)*0.5,(double)(a_b_c3/2.0*0.5*1.73205)));
                   //   resize(templ, templ, Size(img.rows/,img.cols));

                      //  printf("tri  row: %d,  tri col: %d\n",triangle_ROI.rows,triangle_ROI.cols);
               //   printf("teml row: %d, teml col: %d\n",templ.rows,templ.cols);

                  imshow("resize",templ);

                  MatchingMethod(1, 0);
                  }

            }



                }
                }
        }

        //imshow("Triangles",drawing);

    }
}

Mat Labeling(Mat *src)
{
    Mat resultMat = Mat(src->rows, src->cols, CV_8UC1, Scalar(0));

    int width = src->cols;
    int height = src->rows;


    unsigned char* buffer = new unsigned char[height * width];


    int* result = new int[height * width];

    for (int y = 0; y < height; y++){
        for (int x = 0; x < width; x++){

            buffer[width * y + x] = src->data[width * y + x];

            result[width * y + x] = 0;
        }
    }

    std::stack<Point> st;

    int labelNumber = 0;

    for (int y = 1; y < height - 1; y++){
        for (int x = 1; x < width - 1; x++){


            if (buffer[width * y + x] != 0 || result[width * y + x] != 0) continue;

            labelNumber = labelNumber + 50;
            st.push(Point(x, y));
            while (!st.empty()){

                int ky = st.top().y;
                int kx = st.top().x;
                st.pop();

                result[width * ky + kx] = labelNumber;

                // search 8-neighbor
                for (int ny = ky - 1; ny <= ky + 1; ny++){

                    if (ny < 0 || ny >= height) continue;
                    for (int nx = kx - 1; nx <= kx + 1; nx++){

                        if (nx < 0 || nx >= width) continue;


                        if (buffer[width * ny + nx] != 0 || result[width * ny + nx] != 0) continue;

                        st.push(Point(nx, ny));


                        resultMat.at<uchar>(ky, kx) = labelNumber;
                    }
                }
            }
        }
    }
    return resultMat;
}

int main(int argc, char **argv)
{

//    argv[2] ="5.jpg" ;
   templ = imread("5.jpg", 1);
    //	cvtColor(templ,templ, CV_BGR2YCrCb);
       // imshow("templ_bangzi",templ);

    ros::init(argc, argv, "camera_sample");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::namedWindow("dsd");
  // cv::namedWindow("image_window");
  //    cv::namedWindow("result_window");
  // cv::namedWindow("templ_bangzi");
   // cv::namedWindow("imgD");

   // cv::namedWindow("imgE");
   // cv::namedWindow("imgF");

    cv::namedWindow("MASK");
    cv::namedWindow("temp_vel_gray_ROI");
    cv::namedWindow("triagle");
   cv::namedWindow("resize");
 cv::namedWindow("labeling");
   cv::namedWindow("IMAGE");

   cv::namedWindow("ROI");

    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("stereo/left/image_raw", 1, imageCallback);

    ros::spin();
    cv::destroyWindow("view");
    cv::destroyWindow("dsd");
 // cv::destroyWindow("image_window");
  // cv::destroyWindow("result_window");
//    cv::destroyWindow("imgD");
  //  cv::destroyWindow("imgE");
  //  cv::destroyWindow("imgF");
     cv::destroyWindow("IMAGE");
    cv::destroyWindow("temp_vel_gray_ROI");
    cv::destroyWindow("triagle");
    cv::destroyWindow("resize");
    cv::destroyWindow("labeling");
    cv::destroyWindow("ROI");
    cv::destroyWindow("MASK");



  //  cv::destroyWindow("templ_bangzi");



}
