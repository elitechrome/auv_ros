#include "LaneDetection.h"
#include <time.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

LaneDetectionClass::LaneDetectionClass()
{


}

LaneDetectionClass::~LaneDetectionClass()
{

}

//////////////////////////////////////////////////////////////////////////////
////////////////////////////Lane Detection Mode///////////////////////////////
//////////////////////////////////////////////////////////////////////////////
Mat LaneDetectionClass::LaneDetection_BE(Mat Temp_Img)
{
    //Image Processing : rgb2gray
    Mat Gray_Img(Temp_Img.rows, Temp_Img.cols, Temp_Img.depth());
    rgb2gray(Temp_Img, Gray_Img);

    //Image Processing : Set ROI
    Mat Gray_ROI;
    cv::Rect ROI_Rect(0, m_ROI_offset_h, Temp_Img.cols, m_ROI_h_len);
    Gray_ROI = Gray_Img(ROI_Rect);

    //imshow("Gray_ROI", Gray_ROI);

    /*
    Mat First_Img;
    First_Img = Temp_Img(ROI_Rect);
    imshow("First Image", First_Img);
    */
    int Width = Gray_ROI.cols;
    int Height = Gray_ROI.rows;

    //Image Processing : Smoothing / LPF
    Mat Gray_Smooth;
    GaussianBlur(Gray_ROI, Gray_Smooth, Size(7,7), 4);

    //imshow("Gray_Smooth", Gray_Smooth);

    Mat Bin_Img;
    cv::adaptiveThreshold(Gray_Smooth, Bin_Img, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 51, -20.0);

    imshow("Bin_Img", Bin_Img);

    Mat Mop_Img;
    //Filter Mask
    Mat Kernel7(7,7,CV_8U, Scalar(255));
    Mat Kernel5(7,7,CV_8U, Scalar(255));
    Mat Kernel11(11,11,CV_8U, Scalar(255));
    cv::erode(Bin_Img, Mop_Img, Kernel7);
    cv::dilate(Mop_Img, Mop_Img, Kernel5);
    cv::erode(Bin_Img, Mop_Img, Kernel5);

    imshow("Mop_Img", Mop_Img);

    Mat Mag_bin(Height, Width, CV_8U);
    //thinning(Bin_Img, Mag_bin, m_Bin_th);
    thinning(Mop_Img, Mag_bin, m_Bin_th);

    imshow("Mag_bin", Mag_bin);

    //Computer Vision : HoughT
    Mat Hough_Img;
    vector<Vec4i> line;
    HoughLinesP(Bin_Img, line, Hough_rho, CV_PI/180, Hough_th, Hough_minlen, Hough_maxgap);

    //Data Transform
    vector<Lane> Lane_Candi;
    Lane_Candi = LinetoLane(line, m_ROI_offset_h, m_ROI_h_len);


    //reference line draw
    RefLine(Temp_Img);

    Hough_Img = Lane_Est(Lane_Candi, Temp_Img, 0);


    return Hough_Img;
}

Mat LaneDetectionClass::LaneDetection_EE(Mat Temp_Img)
{

    //Sleep(100);

        //Image Processing : rgb2gray
        Mat Gray_Img(Temp_Img.rows, Temp_Img.cols, Temp_Img.depth());
        rgb2gray(Temp_Img, Gray_Img);

        //Image Processing : Set ROI
        Mat Gray_ROI;
        cv::Rect ROI_Rect(0, m_ROI_offset_h, Temp_Img.cols, m_ROI_h_len);
        Gray_ROI = Gray_Img(ROI_Rect);

        int Width = Gray_ROI.cols;
        int Height = Gray_ROI.rows;

        Mat Gray_Smooth;
        GaussianBlur(Gray_ROI, Gray_Smooth, Size(11,11), 1);


        //Edge detection
        Mat Diff_img(Height, Width, CV_16S, Scalar(0));
        Mat Kernel_Sobel(3,3, CV_8S, Scalar(0));

        Kernel_Sobel.data[0] = 3; 	Kernel_Sobel.data[1] = 0; Kernel_Sobel.data[2] = -3;
        Kernel_Sobel.data[3] = 10; 	Kernel_Sobel.data[4] = 0; Kernel_Sobel.data[5] = -10;
        Kernel_Sobel.data[6] = 3;	Kernel_Sobel.data[7] = 0;Kernel_Sobel.data[8] = -3;
        filter2D(Gray_Smooth, Diff_img, Gray_Smooth.depth(), Kernel_Sobel);

        int th_1st = 200;
        int th_2nd = 235;

        for(int i=0; i<Height; i++)
            for(int j =0; j<Width; j++)
            {
                int temp;
                temp = Diff_img.data[i*Width+j];
                if(temp < th_1st)
                    Diff_img.data[i*Width+j] = 0;

                Diff_img.data[i*Width + j] = (int)Diff_img.data[i*Width + j] + (int)Gray_Smooth.data[i*Width + j];
            }

        filter2D(Diff_img, Diff_img, Gray_Smooth.depth(), Kernel_Sobel);



        for(int i=0; i<Height; i++)
            for(int j =0; j<Width; j++)
            {
                int temp;
                temp = (int)Diff_img.data[i*Width+j];
                if(temp < th_2nd)
                    Diff_img.data[i*Width+j] = 0;
            }


        Mat Mag_bin(Height, Width, CV_8U);
        thinning(Diff_img, Mag_bin, 20);


        Mat Hough_Img;
        vector<Vec4i> line;
        HoughLinesP(Mag_bin, line, Hough_rho, CV_PI/180, Hough_th, Hough_minlen, Hough_maxgap);

        //Data Transform
        vector<Lane> Lane_Candi;
        Lane_Candi = LinetoLane(line, m_ROI_offset_h, m_ROI_h_len);

        //reference line draw
        //RefLine(Temp_Img);

        Hough_Img = Lane_Est(Lane_Candi, Temp_Img, 0);

        //imshow("Hough_Img", Hough_Img);

        ViewVP(Temp_Img, VP_x, m_ROI_offset_h - 30, GREEN);

        //imshow("Test Edge", Mag_bin);


        return Temp_Img;


}

Mat LaneDetectionClass::LaneDetection_ER(Mat Temp_Img)
{
    //not implemented
    return Temp_Img;
}

Mat LaneDetectionClass::LaneDetection_BR(Mat Temp_Img)
{
    //not implemented
    return Temp_Img;
}

Mat LaneDetectionClass::LaneDetection_CHEVP(Mat Temp_Img)
{
    return Temp_Img;
}

void LaneDetectionClass::rgb2gray(Mat src, Mat& dst)
{
    int width;
    int height;
    int widthstep;

    width = src.cols;
    height = src.rows;
    widthstep = width*3;

    for(int i=0; i<height; i++)
        for(int j =0; j<width; j++)
        {
            dst.data[i*width+j] = (unsigned char)(src.data[i*widthstep+(j*3)+2]*0.9+ src.data[i*widthstep+(j*3)+1]*0.1);
        }

}

//중심기준선 그리기
void LaneDetectionClass::RefLine(Mat& src)
{
    if(m_Mid_pos >0 && m_Mid_pos < src.cols)
    {
        Point pt1(m_Mid_pos, Max_y);
        Point pt2(m_Mid_pos, Min_y);
        line(src, pt1, pt2, Scalar(255, 0, 255), 1, CV_AA);	//중심 기준선 그리기
    }
}

//Multi-ROI를 위한 roi_num변수 추가(roi 번호를 저장), 0으로 디폴트(마지막 인자 미 입력시 0으로 설정 -> multi-roi 사용 안함)
Mat LaneDetectionClass::Lane_Est(vector<Lane> Lane_Candi, Mat src, int roi_num = 0)
{
    vector<Lane> Left_Candi;
    vector<Lane> Right_Candi;
    vector<Lane> Dir_arr;
    int LaneCandi_size = Lane_Candi.size();	//차선후보개수
    int De_bound = m_Mid_pos;

    Mat Temp_src_Img = src;

    //Left, Right distribution
    for(int Lane_Count = 0; Lane_Count < LaneCandi_size; Lane_Count++)
    {
        if(Lane_Candi[Lane_Count].theta > 20 && Lane_Candi[Lane_Count].theta < 160)
        {
            if(Lane_Candi[Lane_Count].theta < 90)
            {
                Right_Candi.push_back(Lane_Candi[Lane_Count]);
            }
            else
            {
                Left_Candi.push_back(Lane_Candi[Lane_Count]);
            }

        }
    }

    int m_size = Right_Candi.size();
    int n_size = Left_Candi.size();
    double max_factor = 0;
    double max_VPEsty = 0;
    double max_lenup = 0;
    double max_lendown = 0;
    double max_coldiff = 0;
    double max_VPx = 0;
    double max_VPy = 0;
    int Left_index = 0;
    int Right_index = 0;

    if(m_size >0  &&  n_size > 0)	//왼쪽 그리고 오른쪽 차선이 있을때
    {
        for(int m = 0; m < m_size; m++)
        {
            for(int n = 0; n < n_size; n++)
            {
                //Vanishing Point based
                double VPEsty, ColEst, DiffEst;
                double al = (double)(Left_Candi[n].ExP2.y - Left_Candi[n].ExP1.y) / (double)(Left_Candi[n].ExP2.x - Left_Candi[n].ExP1.x);
                double ar = (double)(Right_Candi[m].ExP2.y - Right_Candi[m].ExP1.y) / (double)(Right_Candi[m].ExP2.x - Right_Candi[m].ExP1.x);
                double A11, A12, A21, A22, B1, B2;

                //Vanishing Point
                //자율차 데이터
                //double Ref_vp = -440.4006;
                //double Sigma_vp = 209.8414;

                //산자부 데이터
                double Ref_vp = -200.4006;
                double Sigma_vp = 80.8414;

                //ITS(Adaptive)
                //double Ref_vp = vanishingP_x;
                //double Sigma_vp = 50;

                A11 = -1.0/(-al + ar);
                A12 = -A11;
                A21 = ar/(-al + ar);
                A22 = al/(-al + ar);
                B1 = -al*Left_Candi[n].ExP1.x + Left_Candi[n].ExP1.y;
                B2 = -ar*Right_Candi[m].ExP1.x + Right_Candi[m].ExP1.y;
                VPEsty = -(A21*B1 + A22 * B2);

                //output용 vanishing point	초기화 후 다시연산
                vanishingP_x = 0;
                vanishingP_y = 0;
                vanishingP_x = -(B1 - B2)/(al - ar);
                vanishingP_y = (al*B2 - ar*B1)/(al - ar);

                //double VPy = -(A21*B1 + A22 * B2);
                //double VPx = -(A11*B1 + A12 * B2);
                double VPx = vanishingP_x;
                double VPy = vanishingP_y;
                double abs_vp = abs(VPEsty - Ref_vp);
                double eval_vp = exp(-1.0/2.0 * (abs_vp/Sigma_vp) * (abs_vp/Sigma_vp));



                //length based
                double Ref_len = 0.0;


                // 산자부
                if(roi_num == 1)
                    Ref_len = 900;

                else if(roi_num == 2)
                    Ref_len = 1400;

                else if(roi_num == 3)
                    Ref_len = 1800;

                //do not multi-roi
                else
                    Ref_len = 800;


                double Sigma_len = Ref_len*0.2;
                double length_up = Right_Candi[m].ExP1.x - Left_Candi[n].ExP1.x;
                double length_down = Right_Candi[m].ExP2.x - Left_Candi[n].ExP2.x;
                DiffEst = (length_up + length_down) - Ref_len;

                double eval_len = exp(-1.0/2.0 * (DiffEst/Sigma_len) * (DiffEst/Sigma_len));



                //Color based
                double LeftB1, LeftB2, RightB1, RightB2, mLeftB, mRightB;
                double Sigma_col = 47.5;
                LeftB1 = src.data[Left_Candi[n].p2.y * (ImWidth*3) + (Left_Candi[n].p2.x)*3];
                LeftB2 = src.data[Left_Candi[n].p1.y * (ImWidth*3) + (Left_Candi[n].p1.x)*3];
                RightB1 = src.data[Right_Candi[m].p2.y * (ImWidth*3) + (Right_Candi[m].p2.x)*3];
                RightB2 = src.data[Right_Candi[m].p1.y * (ImWidth*3) + (Right_Candi[m].p1.x)*3];
                mLeftB = (LeftB1 + LeftB2)/2.0;
                mRightB = (RightB1 + RightB2)/2.0;

                ColEst = mRightB - mLeftB;
                if(ColEst > 0)
                    ColEst = 0;

                double eval_col = exp(-1.0/2.0 * (ColEst/Sigma_col) * (ColEst/Sigma_col));


                double w_vp = 0.0;
                double w_len = 1.0;
                double w_col = 0.0;

                double factor = w_vp * eval_vp + w_len * eval_len + w_col * eval_col;

                if(factor > max_factor)
                {
                    max_factor = factor;
                    max_VPEsty = eval_vp;
                    max_lendown = eval_len;
                    max_lenup = length_up;
                    max_coldiff = eval_col;
                    max_VPx = VPx;
                    max_VPy = VPy;
                    Left_index = n;
                    Right_index = m;
                }
                VP_x = max_VPx;
                VP_y = max_VPy;
            }
        }
        Left = Left_Candi[Left_index];
        Right = Right_Candi[Right_index];
        Dir_arr.push_back(Left);
        Dir_arr.push_back(Right);
        Lane_Status2 = 3;
    }
    //오른쪽 차선 후보가 없고 왼쪽 차선 후보만 존재할 때
    else if(m_size == 0 && n_size > 0)
    {
        Lane_Status2 = 2;
        max_factor = 0.0;
    }

    //왼쪽 차선 후보가 없고 왼쪽 차선 후보만 존재할 때
    else if(m_size >0 && n_size == 0)
    {
        Lane_Status2 = 1;
        max_factor = 0.0;
    }

    src = ViewLane_Candi(src, Left_Candi, RED);
    src = ViewLane_Candi(src, Right_Candi, BLUE);

    Lane_Status = (unsigned char)(max_factor * 255);
    src = ViewLane(src, Dir_arr, GREEN);

    return src;
}


Mat LaneDetectionClass::ViewLane_Candi(Mat src, vector<Lane> Lane_Candi, Scalar Line_Color)
{
    int Lane_size = Lane_Candi.size();
    for(int Lane_Count = 0; Lane_Count < Lane_size; Lane_Count++)
    {
        line(src, Lane_Candi[Lane_Count].ExP1, Lane_Candi[Lane_Count].ExP2 , Line_Color, 1, CV_AA);
    }

    return src;
}

Mat LaneDetectionClass::ViewVP(Mat src, int vpx, int vpy, Scalar VP_Color)
{
    circle(src, Point(vpx, vpy), 5, VP_Color);

    return src;
}

Mat LaneDetectionClass::ViewLane(Mat src, vector<Lane> Lane_Candi, Scalar Line_Color)
{
    //Trapezoid Shape Draw
    int Lane_size = Lane_Candi.size();

    for(int Lane_Count = 0; Lane_Count < Lane_size; Lane_Count++)
    {
        line(src, Lane_Candi[Lane_Count].ExP1, Lane_Candi[Lane_Count].ExP2, Line_Color, 2, CV_AA);
    }

    if(Lane_size == 2)
    {

        line(src, Lane_Candi[0].ExP2, Lane_Candi[1].ExP2, Line_Color, 1, CV_AA);
        line(src, Lane_Candi[0].ExP1, Lane_Candi[1].ExP1, Line_Color, 1, CV_AA); // 사다리꼴

        //중심선 그리기
        int upline_mid =  ((Lane_Candi[0].ExP1.x + Lane_Candi[1].ExP1.x)/2.0);	//상단 중심점 x좌표
        int downline_mid = ((Lane_Candi[0].ExP2.x + Lane_Candi[1].ExP2.x)/2.0);	//하단 중심점 x좌표

        line(src,Point(upline_mid, Min_y), Point(downline_mid, Max_y), Line_Color, 2, CV_AA );	//중심선 그리기

        if(Max_y == (m_ROI_offset_h + m_ROI_h_len))
        {
            line(src,Point(upline_mid, Min_y), Point(upline_mid, Min_y+30), Line_Color, 2, CV_AA );		//중심선과의 거리지점 잘보이게 하는 선
            ////////////////////////////////////////////////////////////////////////////////
            double Dist = 2600.0; //VP_y 값에 따라 변화, 차량의 높이, 카메라의 각도정보 필요
            ////////////////////////////////////////////////////////////////////////////////
            int Error_horiz_p = upline_mid - m_Mid_pos;
            Error_horiz = pixel2mm(Error_horiz_p, Dist);

            line(src, Point(m_Mid_pos, Min_y+10), Point(m_Mid_pos + Error_horiz_p, Min_y+10), Line_Color, 1, CV_AA);
            char Err_text[30];
            sprintf(Err_text, "%d[pixel] %d[mm]", Error_horiz_p, Error_horiz);
            putText(src, Err_text, Point(m_Mid_pos - 30, Min_y + 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, GREEN );
            //여기까지
        }

    }
    //오른쪽 또는 왼쪽 일때
    else if(Lane_size == 1)
    {
        if(Lane_Status2 == 1) // 오른쪽
        {
            Error_horiz = 0;
        }
        else if(Lane_Status2 == 2) //왼쪽
        {
            Error_horiz = 0;
        }

    }
    return src;
}

void LaneDetectionClass::thinning(Mat src, Mat& dst, int th)
{
    int width;
    int height;
    unsigned char curr_data;
    unsigned char past_data;
    unsigned char max_data;
    width = src.cols;
    height = src.rows;


    for(int i=0; i<height; i++)
    {
        past_data = 0;
        max_data = 0;

        for(int j = 0; j < width ; j++)
        {
            curr_data = src.data[i*width+j];
            //case 1
            if(j > 0 && curr_data > th && curr_data > max_data && past_data != 0)
            {
                dst.data[i*width+j] = curr_data;
                dst.data[i*width+j-1] = 0;
                max_data = curr_data;
                past_data = curr_data;
            }
            //case 2
            else if(j > 0 && curr_data > th && curr_data > max_data && past_data == 0)
            {
                dst.data[i*width+j] = curr_data;
                max_data = curr_data;
                past_data = curr_data;
            }
            //case 3
            else if(j > 0 && curr_data > th && curr_data < max_data && past_data != 0)
            {
                past_data = curr_data;
                dst.data[i*width+j] = 0;
            }
            //case 4
            else if(j == 0 && curr_data > th)
            {
                dst.data[i*width+j] = curr_data;
                max_data = curr_data;
                past_data = curr_data;
            }
            //case 5
            else if(j > 0 && curr_data < th && past_data != 0)
            {
                dst.data[i*width+j] = curr_data = 0;
                max_data = 0;
                past_data = 0;
            }
            else
            {
                dst.data[i*width+j] = 0;
            }

        }
    }
}

vector<Lane> LaneDetectionClass::LinetoLane(vector<Vec4i> line, int offset, int len)
{
    vector<Lane> Lane_Candi;
    int line_size = line.size();

    Max_y = len + offset;
    Min_y = offset;

    if(line_size == 0)
        return Lane_Candi;


    for(int line_Count = 0; line_Count < line_size; line_Count++)
    {
        Lane Temp;
        if(line[line_Count].val[1] > line[line_Count].val[3])
        {
            Temp.p2 = Point(line[line_Count].val[0], line[line_Count].val[1] + offset);
            Temp.p1 = Point(line[line_Count].val[2], line[line_Count].val[3] + offset);
        }
        else
        {
            Temp.p2 = Point(line[line_Count].val[2], line[line_Count].val[3] + offset);
            Temp.p1 = Point(line[line_Count].val[0], line[line_Count].val[1] + offset);
        }

        double Pow_x = (line[line_Count].val[0] - line[line_Count].val[2])*
                       (line[line_Count].val[0] - line[line_Count].val[2]);
        double Pow_y = (line[line_Count].val[1] - line[line_Count].val[3])*
                       (line[line_Count].val[1] - line[line_Count].val[3]);

        Temp.Length = sqrt(Pow_x + Pow_y);
        double theta_rad = atan2((double)(Temp.p2.y - Temp.p1.y), (double)(Temp.p2.x - Temp.p1.x));

        Temp.theta = (180/PI) * theta_rad;

        Temp.ExP1.y = Min_y;
        Temp.ExP2.y = Max_y;

        if(Temp.p1.x != Temp.p2.x)
        {	//y=ax+b
            double a, b;
            a = (double)(Temp.p2.y - Temp.p1.y)/(double)(Temp.p2.x - Temp.p1.x);		//직선의 기울기
            b = -a * Temp.p1.x + Temp.p1.y;											//직선의 y절편	-> ROI기준

            Temp.ExP1.x = (int)((Min_y - b)/a);
            Temp.ExP2.x = (int)((Max_y - b)/a);
        }
        else
        {
            Temp.ExP1.x = Temp.p1.x;
            Temp.ExP2.x = Temp.p2.x;
        }

        Lane_Candi.push_back(Temp);
    }


    return Lane_Candi;
}


void LaneDetectionClass::gray2bin(Mat& src, Mat& dst, int th)
{
    int width;
    int height;

    width = src.cols;
    height = src.rows;

    for(int i=0; i<height; i++)
        for(int j =0; j<width; j++)
        {
            if(src.data[i*width+j] > th)
                dst.data[i*width+j] = 255;
            else
                dst.data[i*width+j] = 0;
        }
}


int LaneDetectionClass::pixel2mm(int pixel, double Dist)
{
    double p2mRatio = Dist/Camparam_fx;
    /*double mm = pixel * p2mRatio;*/

    double Ratio1 = m_ROI_offset_h*(-0.0055) + 2.1870;
    double Ratio2 = 3.7152;
    double mm = pixel * Ratio2 * Ratio1;

    return (int)mm;
}


void LaneDetectionClass::ParamSet(int ROI_offset_h, int ROI_h_len, int Bin_th, int Mid_pos, int Hough_th_in)
{

    //Hough Parameters
    Hough_th = Hough_th_in;
    Hough_rho = 3;
    Hough_minlen = 10;
    Hough_maxgap = 20;
    m_Lane_DW = 0;

    //Camera Paraemters
    Camparam_fx = 1564.633;
    Camparam_fy = 1579.996;
    Camparam_cx = 628.04;
    Camparam_cy = 544.99;

    m_ROI_h_len = ROI_h_len;
    m_ROI_offset_h = ROI_offset_h;
    m_Bin_th = Bin_th;
    m_Mid_pos = Mid_pos;
}

LaneOut LaneDetectionClass::LaneMain(Mat Temp_Img, int sel_method)
{
    Max_y = m_ROI_h_len + m_ROI_offset_h;
    Min_y = m_ROI_offset_h;
    ImWidth = Temp_Img.cols;
    ImHeight = Temp_Img.rows;

    if(sel_method == BLOB_EVAL)
        LaneDetection_BE(Temp_Img);
    else if(sel_method == BLOB_RANSAC)
        LaneDetection_BR(Temp_Img);
    else if(sel_method == EDGE_EVAL)
        LaneDetection_EE(Temp_Img);
    else if(sel_method == EDGE_RANSAC)
        LaneDetection_ER(Temp_Img);
    else if(sel_method == CHEVP)
        LaneDetection_CHEVP(Temp_Img);
    else
        LaneDetection_BE(Temp_Img);

    //lateral error [mm]
    m_Output.Lat_Error = Error_horiz;
    //Evaluation index 0.0 ~ 1.0
    m_Output.Eval_index = Lane_Status;
    //Lane Status 0 ~3,  0:not exist, 1:only right, 2: only left, 3: both
    m_Output.Lane_State = Lane_Status2;
    //Lane departure warning 0~8, warning level, not implemented
    m_Output.Lane_DW = LDW_Est();
    //Vanishing point, y = 0, x = estimated VP.x, not implemented
    m_Output.Lane_VP.y = VP_y;
    m_Output.Lane_VP.x = VP_x;
    //Mid lane, not implemented
    m_Output.Lane_mid.p1.x = 0;
    m_Output.Lane_mid.p1.y = 0;
    m_Output.Lane_mid.p2.x = 0;
    m_Output.Lane_mid.p2.y = 0;

    return m_Output;
}


int LaneDetectionClass::LDW_Est()
{
    int diff = m_Mid_pos - VP_x;
    int warning_f = 0;
    // diff > 0 : left, diff < 0 right


    //right
    if(diff > 0 && diff <= 20)
        warning_f = 0;
    else if(diff > 20 && diff <= 40 && m_Lane_DW == 0)
        warning_f = 1;
    else if(diff > 40 && diff <= 60 && m_Lane_DW == 1)
        warning_f = 2;
    else if(diff > 60 && m_Lane_DW == 2)
        warning_f = 3;

    //left
    else if(diff > -20 && diff <= 0)
        warning_f = 0;
    else if(diff > -40 && diff <= -20 )
        warning_f = -1;
    else if(diff > -60 && diff <= -40 )
        warning_f = -2;
    else if(diff <= -60)
        warning_f = -3;

    return warning_f;
}
