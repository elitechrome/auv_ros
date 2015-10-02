#ifndef __VISION_MAIN_H
#define __VISION_MAIN_H

#include "LaneDetection.h"


#include <iostream>
#include <math.h>


#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"


//Operation Mode
#define REALTIME_MODE 0
#define POSTPROCESS_MODE 1
#define DATAACQUISITION_MODE 2
#define REALTIME_AND_ACQUISITION_MODE 3

//Message Display line
#define MANUAL_MSG_LINE 1
#define CAN_MSG_LINE 18
#define LANE_STATUS_LINE 13
#define TEMP_LINE 21
#define EXIT_MSG_LINE 19

#define LD_LIMIT 0.5
#define PI 3.14
#define MENU_LINE 11

using namespace cv;
using namespace std;



typedef struct TxData
{
	//차선인식 관련
	unsigned char Lane_status;
	unsigned char Lane_eval;
	int Error_hriz;
};

//
int Frame_No = 1;
int Operation_Mode = 1;
int Detection_Mode = 1;
int ImWidth, ImHeight;

// flag
bool start_flag = false;
bool window1_flag = false;
bool window2_flag = false;


//////////////////
int Max_y;
int Min_y;
int CAN_Count = 0;

TxData m_Txdata;

//Contral variable

int ROI_offset_h = 220;
int ROI_h_len = 160;
int Bin_th = 35;
int Mid_pos = 320;
int Hough_th = 100;

void DataSave(LaneOut Result_lane);

//Menu
void Key_Operation(int keyval);
void manual();

Mat Temp_Img;
Mat Temp_Img2;
Mat Result_lane;

int sign_flag = 0;

LaneDetectionClass m_LaneDetectionClass;

#endif
