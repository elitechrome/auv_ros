#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "sstream"
#include <unistd.h>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "VcUdp.h"
#include "ConfigInfo.h"
#include "CmdPacket.h"

class VisionSTCam{
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Publisher left_pub;
    image_transport::Publisher right_pub;
    image_transport::Publisher disparity_pub;
    cv::Mat leftImg;
    cv::Mat rightImg;
    cv::Mat disparityImg;
    sensor_msgs::ImagePtr leftImgMsg;
    sensor_msgs::ImagePtr rightImgMsg;
    sensor_msgs::ImagePtr disparityImgMsg;

private:
    bool isInit;
    // UDP sockets
    boost::shared_ptr<CVcUdp> m_sockUdpForFindingCameras, m_sockResponseUdp; //sockets as server

    CVcUdp m_sockSendCmdUdp;			// socket as client
    boost::shared_ptr<CVcUdp> m_sockUdpForImgData;		// socket as server
    boost::mutex mutex;

    // config 관련
    char m_szMyIpAddr[16];			// This PC's IP address
    CConfigInfo m_clMyConfigInfo;	// Config. info of this PC and partner camera
    bool m_bHardwareError;
    bool m_bMyConfigIsReady;		// Flag, displays the config status
    int m_nAlreadyOccupiedCamId;
    int m_nMyCamId;

    // cmdProc 관련
    boost::shared_ptr<CCmdPacket> m_clCmdProc;
    bool m_bForcedStop;

    byte *m_bmpImageL;
    byte *m_bmpImageR;
    byte *m_bmpImageD;

    //auto-exposure
    int startingOffset;
    int checkSize;
    int endingOffset;
    int avgIntensity, lastAvgIntensity;
    uint64 lastAccumVal;
    int curExposure, calcExposure, curGlobalGain;
    int gainStep;
    int intensityDiff, lastIntensityDiff;
    int dividingFactor;
    int m_nSetAvgIntensity;
    int m_cScrollBarGlobalGain, m_cScrollBarExposure;

public:
    VisionSTCam(ros::NodeHandle _nh):nh(_nh),it(_nh)
    {
        startingOffset = 3*(FRAME_WIDTH*(FRAME_HEIGHT-270));
        checkSize = 3*640;
        endingOffset = startingOffset + checkSize;
        avgIntensity, lastAvgIntensity = 0;
        lastAccumVal = 0;
        curExposure, calcExposure, curGlobalGain;
        gainStep = 1;
        intensityDiff = 0, lastIntensityDiff = 1;
        dividingFactor = DEFAULT_DIVIDING_FACTOR;

        left_pub    = it.advertise("stereo/left/image_raw", 1);
        right_pub   = it.advertise("stereo/right/image_raw", 1);
        disparity_pub = it.advertise("stereo/disparity/image_raw",1);

        //Control Panel
        cv::namedWindow("Control Intensity", CV_WINDOW_AUTOSIZE);
        cv::resizeWindow("Control Intensity", 480, 400);
        cv::createTrackbar("Global Gain", "Control Intensity", &m_cScrollBarGlobalGain, 200);
        cv::createTrackbar("Exposure", "Control Intensity", &m_cScrollBarExposure, 100);
        m_cScrollBarGlobalGain = DEFAULT_GLOBAL_GAIN;
        m_cScrollBarExposure = DEFAULT_EXPOSURE;

        leftImg = cv::Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3);
        rightImg = cv::Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3);
        disparityImg = cv::Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);
        m_sockUdpForFindingCameras = boost::shared_ptr<CVcUdp>(new CVcUdp());

        if(!GetMyIpAddr()){
            ROS_ERROR("check your NIC. found IP is 127.0.0.1");
            isInit=false;
        }
        m_bMyConfigIsReady = m_clMyConfigInfo.ReadConfigInfo(m_szMyIpAddr);
            if(!m_bMyConfigIsReady)		// CONFIG file 정보가 제대로 읽혀지지 않았으면
            {
                ROS_WARN("Configuration Info Err.!");
                ROS_WARN("Default Value set.!");
            }
        if(!CreateSocketUDP()){
            ROS_ERROR("error occured when creating UDP socket");
            isInit = false;
        }

        isInit=true;
        m_clCmdProc = boost::shared_ptr<CCmdPacket>(new CCmdPacket(&m_sockSendCmdUdp, m_sockResponseUdp.get()));
        m_clCmdProc->VstCmdGetStatus_IsStarted(m_clMyConfigInfo.m_strCameraIpAddr);

        usleep(200000);

        if(m_sockResponseUdp->m_nIsCamStatusRun == STATUS_STOP)  // 받은 메시지가 Stop 상태이면
        {
            ROS_INFO("m_sockResponseUdp STATUS:STOP %s",m_clMyConfigInfo.m_strCameraIpAddr);
            int rtn = m_clCmdProc->VstCmdInitialize(m_clMyConfigInfo.m_strCameraIpAddr);
            if(rtn == CMD_ERROR)
            {
                ROS_ERROR("Init Command Error!");

            }
            else
            {
                ROS_ERROR("Init Command OK!");
            }

            m_sockResponseUdp->m_nIsCamStatusRun = STATUS_NO_RESPONSE;
            m_clCmdProc->VstCmdGetStatus_IsStarted(m_clMyConfigInfo.m_strCameraIpAddr);
            usleep(100000); // 기다렸다가...
            ///OnBnClickedRun();
            m_bForcedStop = false;

            // TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

            rtn = m_clCmdProc->VstCmdStart(m_clMyConfigInfo.m_strCameraIpAddr);
            if(rtn == CMD_ERROR)
            {
                ROS_ERROR("Run Command Error!");
            }
            else
            {
                ROS_INFO("Run Command OK!");
            }

            m_sockResponseUdp->m_nIsCamStatusRun = STATUS_NO_RESPONSE;
            m_clCmdProc->VstCmdGetStatus_IsStarted(m_clMyConfigInfo.m_strCameraIpAddr);

            // 표시 화면 생성
            ///SetTimer(TIMER_IMG_DISPLAY, msec_IMAGE_DISPLAY_CYCLE, NULL);
            ///SetTimer(TIMER_DISPLAY_IMG_DATA_COMM_STATUS, 1000, NULL);
            usleep(100000); // 기다렸다가...

            SetGlobalGain(m_cScrollBarGlobalGain);
            SetExposure(m_cScrollBarExposure);
        }
        else if(m_sockResponseUdp->m_nIsCamStatusRun == STATUS_RUN) // 현재 메시지를 송출 중이라면
        {
            ROS_INFO("m_sockResponseUdp STATUS:RUN");
            ///OnBnClickedRun();
            m_bForcedStop = false;

             //TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
            int rtn = m_clCmdProc->VstCmdStart(m_clMyConfigInfo.m_strCameraIpAddr);
            if(rtn == CMD_ERROR)
            {
                ROS_ERROR("Run Command Error!");
            }
            else
            {
                ROS_INFO("Run Command OK!");
            }

            m_sockResponseUdp->m_nIsCamStatusRun = STATUS_NO_RESPONSE;
            m_clCmdProc->VstCmdGetStatus_IsStarted(m_clMyConfigInfo.m_strCameraIpAddr);

            // 표시 화면 생성
            ///SetTimer(TIMER_IMG_DISPLAY, msec_IMAGE_DISPLAY_CYCLE, NULL);
            ///SetTimer(TIMER_DISPLAY_IMG_DATA_COMM_STATUS, 1000, NULL);
            //usleep(100000); // 기다렸다가...
            SetGlobalGain(DEFAULT_GLOBAL_GAIN);
            SetExposure(DEFAULT_EXPOSURE);
            ///SetTimer(TIMER_UDP_DATA_COMM_DISPLAY, msec_IMAGE_DISPLAY_CYCLE, NULL);
        }
    }
    ~VisionSTCam(){
        m_clCmdProc->VstCmdStop(m_clMyConfigInfo.m_strCameraIpAddr);
    }
    bool GetMyIpAddr(){
        memset(m_szMyIpAddr, 0, sizeof(m_szMyIpAddr));
        char *sMyIp;
        sMyIp = m_sockUdpForFindingCameras->GetMyIPAddress();
        memcpy(m_szMyIpAddr, sMyIp, strlen(sMyIp));
        if(sMyIp == NULL || strcmp(sMyIp, "127.0.0.1") == 0)
        {
            ROS_ERROR("could not find host ip.");
            return false;
        }
        ROS_INFO("host ip is : %s", m_szMyIpAddr);
        return true;
    }
    bool CreateSocketUDP()
    {
        bool rtn = false;

        // Create 'Command Receiving' socket as Server
        if(m_sockResponseUdp == NULL)
        {
            //m_sockResponseUdp = (CVcUdp *)AfxBeginThread(RUNTIME_CLASS(CVcUdp),THREAD_PRIORITY_LOWEST, 0, CREATE_SUSPENDED, NULL); //스레드 생성
            m_sockResponseUdp = boost::shared_ptr<CVcUdp>(new CVcUdp());
            if(m_sockResponseUdp == NULL)
            {
                ROS_DEBUG("Creating Cmd-Response Socket Rcv-thread Fail\n");
                return false;
            }

            rtn = m_sockResponseUdp->InitUdpSock(m_clMyConfigInfo.m_nCommPort,
                BYTES_RESPONSE_PACKET, true);
            if(rtn == false) return false;

            m_sockResponseUdp->ResumeThread();
        }

        // Create Command Sending socket as Client
        rtn = m_sockSendCmdUdp.InitUdpSock(CMD_SEND_PORT, NULL, false);
        if(rtn == false) return false;

        // Create Image Data Receiving socket as Server
        if(m_sockUdpForImgData == NULL)
        {
            m_sockUdpForImgData =  boost::shared_ptr<CVcUdp>(new CVcUdp());
            if(m_sockUdpForImgData == NULL)
            {
                ROS_ERROR("Creating Image-Data Socket Rcv-thread Fail\n");
                return false;
            }
            rtn = m_sockUdpForImgData->InitUdpSock(m_clMyConfigInfo.m_nCommPort+1,
                BYTES_IMAGE_DATA_PACKET, true);
            if(rtn == false)
            {
                return false;
            }
            m_sockUdpForImgData->ResumeThread();

        }

        return true;
    }
    void SetGlobalGain(int pos)
    {
        ROS_INFO("Global gain: %d", pos);
        m_clCmdProc->VstCmdSetGlobalGain(pos, m_clMyConfigInfo.m_strCameraIpAddr);
//        m_cScrollBarGlobalGain=pos;
        return;
    }
    void SetExposure(int pos)
    {
        ROS_INFO("Exposure: %d", pos);
        m_clCmdProc->VstCmdSetExposure(pos, m_clMyConfigInfo.m_strCameraIpAddr);
//        m_cScrollBarExposure=pos;
    }


    void ControlIntensity()
    {
        while (nh.ok()&&!leftImg.empty()){

        uint64 accumVal = 0;

        // 현재 Left 이미지 270번째 라인 Intensity 적산
        for(int i=startingOffset; i<endingOffset; i+=2)
        {
            accumVal += leftImg.data[i];
        }

        // 평균 Intensity 계산
        // 현재 intensity 영향을 반으로 줄이기
        avgIntensity = (lastAccumVal + accumVal)/checkSize;
        intensityDiff = m_nSetAvgIntensity - avgIntensity;

        // 설정한 intensity 범위안에 있거나
        // 그보다 바깥 범위에 있지만, 이전 intensity와 현재 intensity 차이가 10이하면 스킵
        // 너무 민감하게 반응하지 않게 하기 위한 조건들을 더함.
        if((abs(intensityDiff) < 11) ||
            (abs(intensityDiff) < 21 && abs(avgIntensity - lastAvgIntensity) < 10))
        {
            return;
        }

        lastAccumVal = accumVal;
        lastAvgIntensity = avgIntensity;

        // 이전의 intensity diff와 방향이 반대라면
        // 즉, 이전의 exposure 조정값이 너무 커서
        // 목표 intensity 범위를 지나쳐 버렸다면
        if((intensityDiff > 0 && lastIntensityDiff < 0) ||
            (intensityDiff < 0 && lastIntensityDiff > 0))
        {
            // Gain도 낮추고, Exposure 조정값의 폭도 줄도록 한다.
            curGlobalGain = m_cScrollBarGlobalGain;
            if(curGlobalGain >= MIN_GLOBAL_GAIN + gainStep)
                SetGlobalGain(curGlobalGain - gainStep);

            dividingFactor *= 2;
        }
        else if(dividingFactor > DEFAULT_DIVIDING_FACTOR)
        {
            dividingFactor /= 2;	// 빠른 반응을 위해 다시 dividingFactor값을 낮춰둔다.
        }

        lastIntensityDiff = intensityDiff;

        // 현재의 카메라 exposure
        curExposure = m_cScrollBarExposure;

        // 새로 적용할 exposure 값
        calcExposure = curExposure + intensityDiff / dividingFactor;

        // Exposure 만으로 Intensity 조절이 안될 때, Gain도 수정적용
        if(calcExposure < MIN_EXPOSURE)
        {
            calcExposure = MIN_EXPOSURE;
            curGlobalGain = m_cScrollBarGlobalGain;
            if(curGlobalGain > MIN_GLOBAL_GAIN + gainStep)
                SetGlobalGain(curGlobalGain - gainStep);
            return;
        }
        else if(calcExposure > MAX_EXPOSURE)
        {
            calcExposure = MAX_EXPOSURE;
            curGlobalGain = m_cScrollBarGlobalGain;
            if(curGlobalGain < MAX_GLOBAL_GAIN - gainStep)
                SetGlobalGain(curGlobalGain + gainStep);
            return;
        }
        ROS_INFO("Gain : %d, Exposure : %d", curGlobalGain - gainStep, calcExposure);
        SetExposure(calcExposure);
        }
    }
    void process(){
        while (nh.ok()){
        static int lastGoodFrameCount = 0;
        static unsigned long lastCount = 0;

            // 이미지가 아직 갱신되지 않았다면 리턴
            if(!m_sockUdpForImgData->m_bDidRx)
                continue;
            else{
            unsigned char LR, LG, LB, LD;
            unsigned char RR, RG, RB;
            byte *videoBuf = m_sockUdpForImgData->m_ucpImgCopy;

            for(int y=0; y<FRAME_HEIGHT; y++){
            for(int x=0; x<FRAME_WIDTH; x++){

                //L
                LG = videoBuf[(y*FRAME_WIDTH + x) * BYTES_1_PIXEL + 0];
                LB = videoBuf[(y*FRAME_WIDTH + x) * BYTES_1_PIXEL + 1];
                LR = videoBuf[(y*FRAME_WIDTH + x) * BYTES_1_PIXEL + 5];

                //R
                RG = videoBuf[(y*FRAME_WIDTH + x) * BYTES_1_PIXEL + 2];
                RB = videoBuf[(y*FRAME_WIDTH + x) * BYTES_1_PIXEL + 3];
                RR = videoBuf[(y*FRAME_WIDTH + x) * BYTES_1_PIXEL + 7];

                //Disparity Map
                LD = videoBuf[(y*FRAME_WIDTH + x) * BYTES_1_PIXEL + 4];

                leftImg.at<cv::Vec3b>(y,x) = cv::Vec3b(LB,LG,LR);
                rightImg.at<cv::Vec3b>(y,x) = cv::Vec3b(RB,RG,RR);
                disparityImg.at<uchar>(y,x) = LD;
            }
            }
            flip(rightImg,rightImg,1);
            // 화면 표시가 끝났으므로 data를 copy할 수 있게 한다.
            m_sockUdpForImgData->m_bDidRx = false;
//            if((lastCount)%2000==0){
            ControlIntensity();
//                SetGlobalGain(m_cScrollBarGlobalGain);
//                SetExposure(m_cScrollBarExposure);
//            }

        ROS_INFO("FrameRate: %.2f, Success: %d, Fail: %d", m_sockUdpForImgData->m_fFrameRate,
            m_sockUdpForImgData->m_ulGoodFrameCount, m_sockUdpForImgData->m_ulBadFrameCount);
        //ROS_DEBUG("[MAC: %s]", m_clMyConfigInfo.m_bCameraMacAddr);

        /// /////////////////////////////////////////////////////////////////////////////


        //if(!(leftImg.empty()||rightImg.empty())) 
            {
                leftImgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", leftImg).toImageMsg();
                rightImgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rightImg).toImageMsg();
                disparityImgMsg = cv_bridge::CvImage(std_msgs::Header(),"mono8",disparityImg).toImageMsg();
                left_pub.publish(leftImgMsg);
                right_pub.publish(rightImgMsg);
                disparity_pub.publish(disparityImgMsg);
            }
        boost::this_thread::sleep(boost::posix_time::millisec(10));
            }
        }
    }
    bool IsInit()
    {
        return isInit;
    }
};
int main(int argc, char** argv)
{
    // Check if video source has been passed as a parameter

    ros::init(argc, argv, "vision_st_driver_node");

    ros::NodeHandle nh;
    VisionSTCam vst(nh);
//    boost::thread ctrl_thread(boost::bind(&VisionSTCam::ControlIntensity, &vst));
//    boost::thread grab_thread = boost::thread(boost::bind(&VisionSTCam::process, &vst));
    vst.process();
//    while (nh.ok()) {

//        }
    ros::spinOnce();
}
