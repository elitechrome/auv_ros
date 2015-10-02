#ifndef __LANEDETECTION_H
#define __LANEDETECTION_H
#include <iostream>
#include <math.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"


#define BLUE Scalar(255, 0, 0) 
#define RED Scalar(0, 0, 255) 
#define GREEN Scalar(0, 255, 0) 
#define BLACK Scalar(0, 0, 0)
#define YELLOW Scalar(0, 255, 255)
#define ORANGE Scalar(50, 180, 255)
#define PI 3.14

#define BLOB_EVAL 0
#define EDGE_EVAL 1
#define BLOB_RANSAC 2
#define EDGE_RANSAC 3
#define CHEVP 4

using namespace cv;
using namespace std;

typedef struct Lane{
	double theta;
	Point p1, p2;
	Point ExP1, ExP2;
	double Length;
};

typedef struct Speedbump{
	double theta;
	Point SB_p1, SB_p2;
	
	
	double b;
	int index;
	
	Speedbump() : index(0) {}

	double length;
};

typedef struct LaneOut{
	unsigned char Eval_index;
	unsigned char Lane_State;
	int Lane_DW;
	int Lat_Error; 
	Point Lane_VP; 
	Lane Lane_mid; 
	//방지턱 검출을 위한 변수 추가
	//2015.09.01
	//Speed_bump : 방지턱 존재유무 표시, 0 = 미인식, 1 = 인식
	//방지턱과 기준높이사이의 거리차
	int Speed_bump;
	float Speed_bump_distance;	

};


class LaneDetectionClass
{
public:
	//function
	LaneDetectionClass(void);
	~LaneDetectionClass(void);

	//main function
	// 170, 120, 35, 640, 15
	LaneOut LaneMain(Mat Temp_Img, int sel_method);
	void ParamSet(int ROI_offset_h, int ROI_h_len, int Bin_th, int Mid_pos, int Hough_th_in);

	//Lane detection algorithms
	Mat LaneDetection_BE(Mat Temp_Img); //Blob based preprocessing + Evaluation function
	Mat LaneDetection_EE(Mat Temp_Img); //Edge based preprocessing + Evaluation function
	Mat LaneDetection_BR(Mat Temp_Img); //Blob based preprocessing + RANSAC
	Mat LaneDetection_ER(Mat Temp_Img); //Edge based preprocessing + RANSAC
	Mat LaneDetection_CHEVP(Mat Temp_Img); //CHEVP(Multi ROI)
	
	//Speed Bump Detection algorithms
	Mat Speed_bump_Est(vector<Speedbump> SpeedBump_Candi, Mat src);
	vector<Speedbump> line2speedbump(vector<Vec4i> line, int offset);	
	

	//modified image processing & computer vision function
	void rgb2gray(Mat src, Mat& dst);
	void gray2bin(Mat& src, Mat& dst, int th);
	void thinning(Mat src, Mat& dst, int th);
	int pixel2mm(int pixel, double Dist);

	int LDW_Est();
	
	void double_binary(Mat src, Mat& dst, int up, int down);
	

	//Display function
	void RefLine(Mat& src);

	Lane LineGeneration(double theta, Point P2, int y);
	Mat ViewLine(Mat src, vector<Vec4i> lines);
	Mat ViewLane_Candi(Mat src, vector<Lane> Lane_Candi, Scalar Line_Color);
	Mat ViewLane(Mat src, vector<Lane> Lane_Candi, Scalar Line_Color);
	Mat ViewVP(Mat src, int vpx, int vpy, Scalar VP_Color);
	Mat ViewSpeedBump(Mat src);
	
	Mat ViewSpeedbump_Candi(Mat src, vector<Speedbump> Speedbump_Candi, Scalar Color);


private:
	//variable
	LaneOut m_Output;

	//Parameters
	int m_ROI_offset_h;
	int m_ROI_h_len;
	int m_Bin_th;
	int m_Mid_pos;

	//Static Parameters
	int Hough_rho;    // Hough Parameter1
	int Hough_th;    // Hough Parameter2
	int Hough_minlen; // Hough Parameter3
	int Hough_maxgap;// Hough Parameter4

	//Camera model parameter
	double Camparam_fx;
	double Camparam_fy;
	double Camparam_cx;
	double Camparam_cy;

	//Operation Parameters
	Lane Left, Right;
	unsigned char Lane_Status;
	unsigned char Lane_Status2;
	int VP_x;
	int VP_y;
	int Error_horiz;
	int ImWidth;
	int ImHeight;
	int Min_y;
	int Max_y;
	int m_Lane_DW;
	int vanishingP_x;
	int vanishingP_y;
	int global_Speed_bump;
	double global_Speed_bump_distance;

	vector<Lane> left, right, center;

	/*
	int left_up_x;	//multi_roi에서 상하단 연결시 사용할 parameter
	int left_up_y;
	int left_down_x;
	int left_down_y;
	int right_up_x;
	int right_up_y;
	int right_down_x;
	int right_down_y;
	int center_up_x;
	int center_up_y;
	int center_down_x;
	int center_down_y
	*/

};
#endif
