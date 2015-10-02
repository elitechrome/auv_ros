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
    return Temp_Img;
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
	
	//imshow("Gray_Smooth", Gray_Smooth);

	//이진화
	Mat dual_bin(Gray_Smooth.rows, Gray_Smooth.cols, Gray_Smooth.depth());
	double_binary(Gray_Smooth, dual_bin, 180, 70);

        imshow("dual_bin", dual_bin);
	

	Mat Mop_Img;
	//Filter Mask
	Mat Kernel3(3,3,CV_8U, Scalar(255));
	Mat Kernel7(7,7,CV_8U, Scalar(255));
	Mat Kernel5(7,7,CV_8U, Scalar(255));
	Mat Kernel11(11,11,CV_8U, Scalar(255));
	//cv::erode(dual_bin, Mop_Img, Kernel7);
	cv::dilate(dual_bin, Mop_Img, Kernel3);
	//cv::erode(Mop_Img, Mop_Img, Kernel3);

        imshow("Mop_Img", Mop_Img);


	//Edge detection
	Mat Diff_img(Height, Width, CV_16S, Scalar(0));
	Mat Kernel_Sobel(3,3, CV_8S, Scalar(0));

	Kernel_Sobel.data[0] = 3; 	Kernel_Sobel.data[1] = 10; Kernel_Sobel.data[2] = 3;
	Kernel_Sobel.data[3] = 0; 	Kernel_Sobel.data[4] = 0; Kernel_Sobel.data[5] = 0;
	Kernel_Sobel.data[6] = -3;	Kernel_Sobel.data[7] = -10;Kernel_Sobel.data[8] = -3;
	filter2D(Mop_Img, Diff_img, Mop_Img.depth(), Kernel_Sobel);
	
	int th_1st = 100;
	int th_2nd = 150;

	for(int i=0; i<Height; i++)
		for(int j =0; j<Width; j++)
		{
			int temp;
			temp = Diff_img.data[i*Width+j];
			if(temp < th_1st)
				Diff_img.data[i*Width+j] = 0;

			Diff_img.data[i*Width + j] = (int)Diff_img.data[i*Width + j] + (int)Mop_Img.data[i*Width + j];
		}

	filter2D(Diff_img, Diff_img, Diff_img.depth(), Kernel_Sobel);	

	for(int i=0; i<Height; i++)
		for(int j =0; j<Width; j++)
		{
			int temp;
			temp = (int)Diff_img.data[i*Width+j];
			if(temp < th_2nd)
				Diff_img.data[i*Width+j] = 0;
		}


        imshow("Diff_img", Diff_img);

	//Mat Mag_bin(Height, Width, CV_8U);
	//thinning(Diff_img, Mag_bin, m_Bin_th);

	//Computer Vision : HoughT
	vector<Vec4i> line;
	HoughLinesP(Diff_img, line, Hough_rho, CV_PI/180, Hough_th, Hough_minlen, Hough_maxgap);


	//Speed Bump Detection
	vector<Speedbump> Speedbump_Candi; 
	Speedbump_Candi = line2speedbump(line, m_ROI_offset_h);

	Mat Hough_SB_Img;
	Hough_SB_Img = Speed_bump_Est(Speedbump_Candi, Temp_Img);

        imshow("Hough_SB_Img", Hough_SB_Img);

	return Hough_SB_Img;
	
	
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
			dst.data[i*width+j] = (unsigned char)(src.data[i*widthstep+(j*3)+2]*0.5+ src.data[i*widthstep+(j*3)+1]*0.5);
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
	
	//Speed bump check
	//2015.09.01 추가
	global_Speed_bump = 0;
	global_Speed_bump_distance = 200.0;	//초기값은 200픽셀(그냥  저멀리 있으므로 높게 잡음)

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
	//Speed bump
	m_Output.Speed_bump = global_Speed_bump;
	m_Output.Speed_bump_distance = global_Speed_bump_distance;

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


////////////////////////////////////////////////////////방지턱 인식 알고리즘////////////////////////////////////////////////////////////////////////////
void LaneDetectionClass::double_binary(Mat src, Mat& dst, int up, int down)
{
	int width;
	int height;
	
	width = src.cols;
	height = src.rows;
	
	for(int i=0; i<height; i++)
		for(int j =0; j<width; j++)
		{	//dst.data[i*width+j] = (unsigned char)(src.data[i*width+j]);
			
			if(src.data[i*width+j] >= up || src.data[i*width+j] < down)
				dst.data[i*width+j] = (unsigned char)(255);
			
			else
				dst.data[i*width+j] = (unsigned char)(0);//(src.data[i*width+j]);
			
		}
		
}

Mat LaneDetectionClass::Speed_bump_Est(vector<Speedbump> SpeedBump_Candi, Mat src)
{
	int SB_th = 10;	//방지턱 두께

	vector<Speedbump> maybe_Speedbump, final_maybe_Speedbump;
	Speedbump final_Speedbump;
	

	int SpeedBump_Candi_size = SpeedBump_Candi.size();

	if(SpeedBump_Candi_size == 0)
		global_Speed_bump = 0;
		
	Mat Temp_src_Img = src;

	for(int SpeedBump_Count = 0; SpeedBump_Count < SpeedBump_Candi_size; SpeedBump_Count++)
	{	
		//조건변화 - 대각선만 찾자
		if(SpeedBump_Candi[SpeedBump_Count].theta < 5 && SpeedBump_Candi[SpeedBump_Count].theta > -5)						//155~175도 사이이고(각도가 왜이럴까)
		{	
			if(SpeedBump_Candi[SpeedBump_Count].length > 40)	//길이가 40픽셀 이상이면
				maybe_Speedbump.push_back(SpeedBump_Candi[SpeedBump_Count]);													//방지턱 한번더(가중치)		
			
			if(SpeedBump_Candi[SpeedBump_Count].length > 60)	//길이가 60픽셀 이상이면
				maybe_Speedbump.push_back(SpeedBump_Candi[SpeedBump_Count]);													//방지턱 한번더(가중치)																												
					
			if(SpeedBump_Candi[SpeedBump_Count].length > 80)	//길이가 80픽셀 이상이면										
			{	maybe_Speedbump.push_back(SpeedBump_Candi[SpeedBump_Count]);													//방지턱 더더더(가중치)
				maybe_Speedbump.push_back(SpeedBump_Candi[SpeedBump_Count]);
			}
			if(SpeedBump_Candi[SpeedBump_Count].length > 100)	//길이가 100픽셀 이상이면										
				maybe_Speedbump.push_back(SpeedBump_Candi[SpeedBump_Count]);													//방지턱 더더더(가중치)
		
			if(SpeedBump_Candi[SpeedBump_Count].length > 100)	//길이가 100픽셀 이상이면										
			{	maybe_Speedbump.push_back(SpeedBump_Candi[SpeedBump_Count]);													//방지턱 더더더(가중치)
				maybe_Speedbump.push_back(SpeedBump_Candi[SpeedBump_Count]);
				maybe_Speedbump.push_back(SpeedBump_Candi[SpeedBump_Count]);
			}
			
			if(SpeedBump_Candi[SpeedBump_Count].length > 150)	//길이가 150픽셀 이상이면										
			{	maybe_Speedbump.push_back(SpeedBump_Candi[SpeedBump_Count]);													//방지턱 더더더(가중치)
				maybe_Speedbump.push_back(SpeedBump_Candi[SpeedBump_Count]);
				maybe_Speedbump.push_back(SpeedBump_Candi[SpeedBump_Count]);
				maybe_Speedbump.push_back(SpeedBump_Candi[SpeedBump_Count]);
			}

		}		

		
	}

	//ViewSpeedbump_Candi(src, maybe_Speedbump, YELLOW);	

	//직선의 방정식 세우기 -->y 찾기
	int candi_size = maybe_Speedbump.size();
	for(int maybe_count=0; maybe_count<candi_size; maybe_count++)
	{
		if(maybe_Speedbump[maybe_count].SB_p1.x != maybe_Speedbump[maybe_count].SB_p2.x)
		{	//y=ax+b
			double a, b;
			a = (double)(maybe_Speedbump[maybe_count].SB_p2.y - maybe_Speedbump[maybe_count].SB_p1.y)/(double)(maybe_Speedbump[maybe_count].SB_p2.x - maybe_Speedbump[maybe_count].SB_p1.x);		//직선의 기울기
			maybe_Speedbump[maybe_count].b = -a * maybe_Speedbump[maybe_count].SB_p1.x + maybe_Speedbump[maybe_count].SB_p1.y;			//직선의 y절편	-> ROI기준
			
		}	
	}

	//제일 많은놈 보팅 후 그놈을 방지턱 부분으로 인식
	double distance = 10;	//일단 디스턴스 : 5픽셀
	int index_now = 0;


	for(int maybe_count=0; maybe_count<candi_size; maybe_count++)
	{	
		if(maybe_Speedbump[maybe_count].index == 0)		//현재의 놈이 index = 0일때만 연산
		{	
			index_now++;	
			maybe_Speedbump[maybe_count].index = index_now;	//0인놈에 index_now+1부여

			for(int j=0; j<candi_size; j++)		//다음놈들과 y절편 차이하나하나 비교
			{	
				if(maybe_Speedbump[j].index == 0 && abs(maybe_Speedbump[maybe_count].b - maybe_Speedbump[j].b) < distance)
					maybe_Speedbump[j].index = maybe_Speedbump[maybe_count].index;	//다음거 index가 0이고, distance이하라면 같은인덱스
				
			}
		}
	}
	
	int size_index=0;
	int vote_max = 0;
	int max_index;
	vector<int> may_index;

	//가장 큰 인덱스값 찾기
	for(int maybe_count=0; maybe_count<candi_size; maybe_count++)
		if(size_index < maybe_Speedbump[maybe_count].index)
			size_index = maybe_Speedbump[maybe_count].index;
	
	//vote 0으로 초기화
	int* vote=(int*)malloc(2000);	//여유있게 500개분량
	
	for(int i=0; i<size_index; i++)
		vote[i] = 0;
	
	//투표
	for(int maybe_count=0; maybe_count<candi_size; maybe_count++)
	{
		int num = vote[maybe_Speedbump[maybe_count].index] + 1;
		vote[maybe_Speedbump[maybe_count].index] = num;
	}

	
	for(int i=0; i<size_index; i++)
	{
		if(vote[i] > 5)	//동일직선 5개 이상이면 일단 저장	
		{
			may_index.push_back(i);
		}
		//얘는 일단 안씀


		if(vote[i]>vote_max)
		{	vote_max = vote[i];
			max_index = i;
		}
	}
	
	free(vote);

	
	if(vote_max > 3)
	{
		//평균내서 사이즈를 끝에서 끝까지 표현
		double sum = 0;
		int mean = 0;

		for(int maybe_count=0; maybe_count<candi_size; maybe_count++)
			if(maybe_Speedbump[maybe_count].index == max_index)
				sum = sum + maybe_Speedbump[maybe_count].b;

		mean = floor(sum/vote_max);	//방지턱위치의 높이

		final_Speedbump.SB_p1.x = 0;
		final_Speedbump.SB_p1.y = mean;
		final_Speedbump.SB_p2.x = src.cols;
		final_Speedbump.SB_p2.y = mean;
	
		line(src, final_Speedbump.SB_p1, final_Speedbump.SB_p2, ORANGE, 3, CV_AA);
		global_Speed_bump = 1;
	}
	
	return src;
}

//line을 방지턱 후보로
vector<Speedbump> LaneDetectionClass::line2speedbump(vector<Vec4i> line, int offset)	
{
	vector<Speedbump> SpeedBump_Candi;
	int line_size = line.size();

	if(line_size == 0)
		return SpeedBump_Candi;

	for(int line_Count = 0; line_Count < line_size; line_Count++)
	{
		Speedbump Temp;

		if(line[line_Count].val[1] > line[line_Count].val[3])
		{
			Temp.SB_p2 = Point(line[line_Count].val[0], line[line_Count].val[1] + offset);
			Temp.SB_p1 = Point(line[line_Count].val[2], line[line_Count].val[3] + offset);
		}
		else
		{
			Temp.SB_p2 = Point(line[line_Count].val[2], line[line_Count].val[3] + offset);
			Temp.SB_p1 = Point(line[line_Count].val[0], line[line_Count].val[1] + offset);
		}

		double Pow_x = (line[line_Count].val[0] - line[line_Count].val[2])* (line[line_Count].val[0] - line[line_Count].val[2]);
		double Pow_y = (line[line_Count].val[1] - line[line_Count].val[3])* (line[line_Count].val[1] - line[line_Count].val[3]);
	
		Temp.length = sqrt(Pow_x + Pow_y);

		double theta_rad = atan2((double)(Temp.SB_p2.y - Temp.SB_p1.y), (double)(Temp.SB_p2.x - Temp.SB_p1.x));

		Temp.theta = (180/PI) * theta_rad;
		
		SpeedBump_Candi.push_back(Temp);
	}

	return SpeedBump_Candi;
}

Mat LaneDetectionClass::ViewSpeedbump_Candi(Mat src, vector<Speedbump> Speedbump_Candi, Scalar Color)
{
	int Speedbump_size = Speedbump_Candi.size();
	
	for(int SB_Count=0; SB_Count < Speedbump_size; SB_Count++)
	{
		line(src, Speedbump_Candi[SB_Count].SB_p1, Speedbump_Candi[SB_Count].SB_p2, Color, 1, CV_AA);
	}

	return src;
}
