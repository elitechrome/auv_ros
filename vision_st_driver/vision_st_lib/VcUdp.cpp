
#include "VcUdp.h"
#include <cstdio>
#include <sys/time.h>
#include <stdlib.h>
namespace
{
    class __GET_TICK_COUNT
    {
    public:
        __GET_TICK_COUNT()
        {
        if (gettimeofday(&tv_, NULL) != 0)
            throw 0;
        }
        timeval tv_;
    };
    __GET_TICK_COUNT timeStart;
}

unsigned long GetTickCount()
{
    static time_t   secStart    = timeStart.tv_.tv_sec;
    static time_t   usecStart   = timeStart.tv_.tv_usec;
                    timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec - secStart) * 1000 + (tv.tv_usec - usecStart) / 1000;
}

CVcUdp::CVcUdp()
{
    m_bIngTx = m_bIngRx = m_bDidTx = m_bDidRx = false;
    m_szRcvBuff = new char[DEFAULT_SIZE_OF_PACKET];
    m_szSendBuff = new char[DEFAULT_SIZE_OF_PACKET];

	m_nRcvDataLen = m_nSendDataLen = 0;
	m_nBytesToRcv = DEFAULT_SIZE_OF_PACKET;
	m_ucpImg = m_ucpImgCopy = NULL;
	m_clConfigInfo = NULL;
	m_nIsCamStatusRun = STATUS_NO_RESPONSE;
	m_fFrameRate = 0.0;
    m_bIsSignificantFault = false;
}

CVcUdp::~CVcUdp()
{
    if(!m_thread)
    delete m_thread;
}
void CVcUdp::ResumeThread(){
    m_thread = new boost::thread(boost::bind(&CVcUdp::Run, this));
}
bool CVcUdp::InitUdpSock(int portNo, int bytesToRcv, bool isServer)
{
    m_sUdp = m_cUdp = -1;
	m_sPortUdp = portNo;
    m_bReceiveRunFlag = true;		// 초기 시작 시 recvfrom을 지속시키기 위해

	if(isServer)
	{
        int rtn = -1;
		m_nBytesToRcv = bytesToRcv;
		m_nSvrSockInLen = sizeof(m_sinUdpS);

		m_sUdp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if(m_sUdp == -1)
		{
            printf("INVALID_SOCKET\n");
            return false;
		}

		SetSocketOptions();

        //fill in sockaddr_in struct
		memset((char *)&m_sinUdpS, 0, sizeof(m_sinUdpS));
		m_sinUdpS.sin_family = AF_INET;
		m_sinUdpS.sin_port = htons(m_sPortUdp);
		m_sinUdpS.sin_addr.s_addr = INADDR_ANY;

		//bind to the socket
        rtn = bind(m_sUdp, (struct sockaddr*)&m_sinUdpS, sizeof(m_sinUdpS));
        if(rtn == -1)
		{
            printf("sock: bind error\n");
            return false;
		}
	}
	else
	{
		//Client ----------------------------------
		m_nClientSockInLen = sizeof(m_sinUdpC);

		memset((char *)&m_sinUdpC, 0, sizeof(m_sinUdpC));
		m_sinUdpC.sin_family = AF_INET;

		m_sinUdpC.sin_port = htons(m_sPortUdp);


		m_cUdp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if(m_cUdp == -1)
		{
            printf("INVALID_SOCKET\n");
            return false;
		}

		int opt = 1;
		setsockopt (m_cUdp, SOL_SOCKET, SO_BROADCAST, (char*)&opt, sizeof(opt));
	}

    return true;
}

void CVcUdp::SetSocketOptions()
{
    static int maxSd = 0;

	// udp recv buffer를 이렇게 키워두지 않으면 수신속도가 느려진다?
	int udp_size = 1032*768*16;//65536*128;
	setsockopt (m_sUdp, SOL_SOCKET, SO_RCVBUF, (char*)&udp_size, sizeof(udp_size));
}

void CVcUdp::CloseUdpSocket()
{
    if(m_sUdp != -1)
        close(m_sUdp);
    if(m_cUdp != -1)
        close(m_cUdp);
}

bool CVcUdp::SendData(char *buff, int len, const char *destination)
{
	int n=0;
    printf("%s %d-Snd[%d]. %d bytes\n", destination, m_sPortUdp, buff[8], len);

	m_sinUdpC.sin_addr.s_addr = inet_addr(destination);;

	char *nextP = (char *)buff;
    m_bIngTx = true;		// false는 다른 곳에서 만든다.

	m_nSendDataLen = len;
	while(len)
	{
        n = sendto(m_cUdp, nextP, len, 0, (struct sockaddr *)&m_sinUdpC, m_nClientSockInLen);
		len -= n;
		nextP += n;
	}
    m_bDidTx = true;		// false는 다른 곳에서 만든다.

    return true;
}

void CVcUdp::OnReceiveUdpResponseMsg()
{
	int n = 0;
    printf("Rcv. Started[%d]\n", m_sPortUdp);
	unsigned char *rcvData = (unsigned char *)m_szRcvBuff;

	///////////////////// udp receive 시 timeout을 구현
	//timeval timeout;
	timeval timeout;
	timeout.tv_sec = 0L;
	timeout.tv_usec = 100000L;

	//fd_set sockfds;
	fd_set sockfds;
	FD_ZERO(&sockfds);
	FD_SET(m_sUdp, &sockfds);
	//////////////////// end of (udp receive 시 timeout을 구현)

	while(m_bReceiveRunFlag)
	{
//		///////////////////// udp receive 시 timeout을 구현
//		FD_ZERO(&sockfds);
//		FD_SET(m_sUdp, &sockfds);
//		if(select(-1, &sockfds, 0, 0, &timeout) <= 0)
//			continue;
        if(setsockopt(m_sUdp, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) <0)
            continue;
//		///////////////////// end of (udp receive 시 timeout을 구현)


		n = recvfrom(m_sUdp, m_szRcvBuff, DEFAULT_SIZE_OF_PACKET, 0,
            (struct sockaddr *)&m_sinUdpS, &m_nSvrSockInLen);

        m_bIngRx = true;		// false는 다른 곳에서 만든다.

        //printf("%d- [%d bytes]\n", m_sPortUdp, n);
		if(n <= 0) continue;

		int sizeToRcv = BYTES_RESPONSE_PACKET - n;	// kjc
		m_nRcvDataLen = n;

		while(sizeToRcv > 0)
		{
			n = recvfrom(m_sUdp, m_szRcvBuff + m_nRcvDataLen, DEFAULT_SIZE_OF_PACKET, 0, 
                (struct sockaddr *)&m_sinUdpS, &m_nSvrSockInLen);

			if(n<0) n=0;
			sizeToRcv -= n;
			m_nRcvDataLen += n;
		}

        m_bDidRx = true;		// false는 다른 곳에서 만든다.
        //printf("[Received data] - size : %d\n", m_nRcvDataLen);

		// process the find command's reply messages
		if(m_sPortUdp == FIND_CAMERA_RESULT_RCV_PORT)
		{
			if(rcvData[CAM_MSG_FIND_BUFFER_INDEX_REG] != CAM_MSG_FIND){
				continue;
			}

			CConfigInfo configInfo;
			char tmp[32];

			switch(rcvData[CAM_MSG_FIND_BUFFER_INDEX])
			{
			case CAM_MSG_CONFIG_CAM_IP_ADDR:
				memset(tmp, 0x00, sizeof(tmp));
				sprintf(tmp,"%02X%02X%02X%02X%02X%02X", rcvData[1], rcvData[2], 
					rcvData[3], rcvData[4], rcvData[5], rcvData[6]);
				memcpy(configInfo.m_bCameraMacAddr, tmp, strlen(tmp));
                //printf("MAC: %s\n", tmp);

				memset(tmp, 0x00, sizeof(tmp));
				sprintf(tmp,"%d.%d.%d.%d", rcvData[8], rcvData[9], rcvData[10], rcvData[11]);
				memcpy(configInfo.m_strCameraIpAddr, tmp, strlen(tmp));
				SetCameraInformation(configInfo, CAM_MSG_CONFIG_CAM_IP_ADDR);
				break;

			case CAM_MSG_CONFIG_CAM_SUBNET_MASK:
				memset(tmp, 0x00, sizeof(tmp));
				sprintf(tmp,"%02X%02X%02X%02X%02X%02X", rcvData[1], rcvData[2], 
					rcvData[3], rcvData[4], rcvData[5], rcvData[6]);
				memcpy(configInfo.m_bCameraMacAddr, tmp, strlen(tmp));
                //printf("MAC: %s\n", tmp);

				memset(tmp, 0x00, sizeof(tmp));
				sprintf(tmp,"%d.%d.%d.%d", rcvData[8], rcvData[9], rcvData[10], rcvData[11]);
				memcpy(configInfo.m_strCameraSubnetMask, tmp, strlen(tmp));
				SetCameraInformation(configInfo, CAM_MSG_CONFIG_CAM_SUBNET_MASK);
				break;

			case CAM_MSG_CONFIG_CAM_GATEWAY_ADDR:
				memset(tmp, 0x00, sizeof(tmp));
				sprintf(tmp,"%02X%02X%02X%02X%02X%02X", rcvData[1], rcvData[2], 
					rcvData[3], rcvData[4], rcvData[5], rcvData[6]);
				memcpy(configInfo.m_bCameraMacAddr, tmp, strlen(tmp));
                //printf("MAC: %s\n", tmp);

				memset(tmp, 0x00, sizeof(tmp));
				sprintf(tmp,"%d.%d.%d.%d", rcvData[8], rcvData[9], rcvData[10], rcvData[11]);
				memcpy(configInfo.m_strCameraGwAddr, tmp, strlen(tmp));
				SetCameraInformation(configInfo, CAM_MSG_CONFIG_CAM_GATEWAY_ADDR);
				break;

			case CAM_MSG_CONFIG_HOST_IP_ADDR:
				memset(tmp, 0x00, sizeof(tmp));
				sprintf(tmp,"%02X%02X%02X%02X%02X%02X", rcvData[1], rcvData[2], 
					rcvData[3], rcvData[4], rcvData[5], rcvData[6]);
				memcpy(configInfo.m_bCameraMacAddr, tmp, strlen(tmp));
                //printf("MAC: %s\n", tmp);

				memset(tmp, 0x00, sizeof(tmp));
				sprintf(tmp,"%d.%d.%d.%d", rcvData[8], rcvData[9], rcvData[10], rcvData[11]);
				memcpy(configInfo.m_strHostIpAddr, tmp, strlen(tmp));
				SetCameraInformation(configInfo, CAM_MSG_CONFIG_HOST_IP_ADDR);
				break;

			case CAM_MSG_CONFIG_CAM_PORT_NO:
				memset(tmp, 0x00, sizeof(tmp));
				sprintf(tmp,"%02X%02X%02X%02X%02X%02X", rcvData[1], rcvData[2], 
					rcvData[3], rcvData[4], rcvData[5], rcvData[6]);
				memcpy(configInfo.m_bCameraMacAddr, tmp, strlen(tmp));
                //printf("MAC: %s\n", tmp);

				configInfo.m_nCommPort = (rcvData[8]<<8) | rcvData[9];			
				SetCameraInformation(configInfo, CAM_MSG_CONFIG_CAM_PORT_NO);
				break;

			case CAM_MSG_CONFIG_CAM_NICKNAME1:
				memset(tmp, 0x00, sizeof(tmp));
				sprintf(tmp,"%02X%02X%02X%02X%02X%02X", rcvData[1], rcvData[2], 
					rcvData[3], rcvData[4], rcvData[5], rcvData[6]);
				memcpy(configInfo.m_bCameraMacAddr, tmp, strlen(tmp));
                //printf("MAC: %s\n", tmp);

				memcpy(configInfo.m_strCameraNick1, &rcvData[9], 5);
				SetCameraInformation(configInfo, CAM_MSG_CONFIG_CAM_NICKNAME1);
			break;

			case CAM_MSG_CONFIG_CAM_NICKNAME2:
				memset(tmp, 0x00, sizeof(tmp));
				sprintf(tmp,"%02X%02X%02X%02X%02X%02X", rcvData[1], rcvData[2], 
					rcvData[3], rcvData[4], rcvData[5], rcvData[6]);
				memcpy(configInfo.m_bCameraMacAddr, tmp, strlen(tmp));
                //printf("MAC: %s\n", tmp);

				memcpy(configInfo.m_strCameraNick2, &rcvData[9], 5);
				SetCameraInformation(configInfo, CAM_MSG_CONFIG_CAM_NICKNAME2);
				break;
			default:
				break;
			}
		}
		// process the control command's reply messages
		else
		{
			// IsRun 명령의 response가 도착하면 다음과 같이 정리 
			if(rcvData[CAM_MSG_FIND_BUFFER_INDEX] == CAM_MSG_IIC_RD_WR1 &&
				rcvData[9] == 0x20 && rcvData[10] == 0x00)
			{
				if(rcvData[11] == 1)
					m_nIsCamStatusRun = STATUS_RUN;
				else
					m_nIsCamStatusRun = STATUS_STOP;
			}
		}
	usleep(10000);	
	}
}

void CVcUdp::OnReceiveUdpImgData()
{
    printf("Img Data Rcv. Started [%d]\n", m_sPortUdp);

	int n=0;
	m_usFrameCount = 0;
	m_ulBadFrameCount = 0;
	m_ulGoodFrameCount = 0;

    ///////////////////// udp receive 시 timeout을 구현
    //timeval timeout;
    timeval timeout;
    timeout.tv_sec = 0L;
    timeout.tv_usec = 100000L;

    //fd_set sockfds;
    fd_set sockfds;
    FD_ZERO(&sockfds);
    FD_SET(m_sUdp, &sockfds);
    ///////////////////// end of (udp receive 시 timeout을 구현)
	while(m_bReceiveRunFlag)
	{
    //boost::mutex::scoped_lock l(m_mutex);
//		///////////////////// udp receive 시 timeout을 구현
//        FD_ZERO(&sockfds);
//        FD_SET(m_sUdp, &sockfds);
//		if(select(-1, &sockfds, 0, 0, &timeout) <= 0)
//			continue;
        if(setsockopt(m_sUdp, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) <0)
            continue;
//		///////////////////// end of (udp receive 시 timeout을 구현)

		n = recvfrom(m_sUdp, m_szRcvBuff, m_nBytesToRcv, 0,
            (struct sockaddr *)&m_sinUdpS, &m_nSvrSockInLen);

        //printf("%d- [%d bytes] %02d %02d\n", m_sPortUdp, n, (unsigned char)m_szRcvBuff[2], (unsigned char)m_szRcvBuff[3]);

		int sizeToRcv = BYTES_IMAGE_DATA_PACKET - n;
		m_nRcvDataLen = n;

		while(sizeToRcv > 0)
		{
			n = recvfrom(m_sUdp, m_szRcvBuff + m_nRcvDataLen, m_nBytesToRcv, 0, 
                (struct sockaddr *)&m_sinUdpS, &m_nSvrSockInLen);

			if(n<0) n=0;
			sizeToRcv -= n;
			m_nRcvDataLen += n;
		}

        ImageDataProc();
        
	}
}

void CVcUdp::ImageDataProc()
{
    static bool flag;
    static unsigned long stick = 0;
    static unsigned long etick = 0;
    static unsigned short packetCount = 0;
    static bool startCheckingFlag = false;

	//m_usFrameCount = ((unsigned char)m_szRcvBuff[0]<<8) | ((unsigned char)m_szRcvBuff[1]<<0);
	m_usPacketNo = ((unsigned char)m_szRcvBuff[2]<<8) | ((unsigned char)m_szRcvBuff[3]<<0);
	if(m_usPacketNo == 0)
	{
		packetCount = 0;
        startCheckingFlag = true;
	}
	else if(m_usPacketNo < 0 || m_usPacketNo >= FRAME_HEIGHT*4) return;

	// 4bytes 중복된 frame count + packet count 값을 확인하면 
	// Camera가 제대로 지정된 사이즈로 packet 데이터를 보내고 있는지 확인할 수 있다.
	if(*(int *)&m_szRcvBuff[0] != *(int *)&m_szRcvBuff[4])
	{
        m_bIsSignificantFault = true;
		return;
	}

	// 수신된 packet을 해당 image line으로 copy
	memcpy(&m_ucpImg[m_usPacketNo*BYTES_PURE_IMAGE_IN_PACKET], 
		&m_szRcvBuff[BYTES_IMAGE_PACKET_HADER], BYTES_PURE_IMAGE_IN_PACKET);

	// 수신된 패킷 번호와 내부에서 관리하는 packet 번호가 다를 경우
	// 에러로 처리 - 이부분은 좀더 고민해서 처리할 필요 있음.
	if(m_usPacketNo != packetCount && startCheckingFlag)
	{
        startCheckingFlag = false;
		m_ulBadFrameCount++;
		return;
	}

	// 이미지 1 frame이 완성되었으면
    if(++packetCount >= FRAME_HEIGHT*4)
    {
        m_usFrameCount++;

        // 상위 화면처리가 끝나지 않았으면 copy하지 않는다
        if(!m_bDidRx)
        {
            memcpy(m_ucpImgCopy, m_ucpImg, FRAME_HEIGHT * FRAME_WIDTH * BYTES_1_PIXEL);
            m_ulGoodFrameCount++;
            m_bDidRx = true;		// false는 다른 곳에서 만든다.
        }

		// frame rate 계산을 위해서 사용
		if(m_usFrameCount > 100) 
		{
			if(flag==0){	
				stick = GetTickCount();	
				flag = 1;
			}
			else{
				etick = GetTickCount();
				m_fFrameRate = (float)m_usFrameCount/(float)(etick - stick) * 1000;
				flag = 0;
			}
			m_usFrameCount = 0;
		}
	}
}

// Udp class를 resume할 때 실행됨
int CVcUdp::Run()
{
	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.

	// Find와 Command 처리용일 경우
	if(m_nBytesToRcv == BYTES_RESPONSE_PACKET)
	{
		// Find용 udp thread일 때에는 찾은 Camera들 정보를 담기 위한 변수할당
		if(m_sPortUdp == FIND_CAMERA_RESULT_RCV_PORT)
		{
            m_clConfigInfo = new CConfigInfo[sizeof(CConfigInfo) * MAX_NUM_CAMERA];
			m_nNumSavedCamInfo = 0;
		}
        printf("response thread activated\n");
		OnReceiveUdpResponseMsg();
	}
	else	// 이미지 처리용일 경우
	{
        m_ucpImg = new byte[FRAME_HEIGHT * FRAME_WIDTH * BYTES_1_PIXEL];
        m_ucpImgCopy = new byte[FRAME_HEIGHT * FRAME_WIDTH * BYTES_1_PIXEL];
        printf("imagegrabber thread activated\n");
		OnReceiveUdpImgData();
	}

    return 0;
}


int CVcUdp::ExitInstance()
{
	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.
    m_bReceiveRunFlag = false;
    usleep(300*1000);
	CloseUdpSocket();

	if(m_ucpImg != NULL)
        free(m_ucpImg);
	if(m_ucpImgCopy != NULL)
        free(m_ucpImgCopy);
	if(m_clConfigInfo != NULL)
        free(m_clConfigInfo);

	if(m_szRcvBuff != NULL)
        free(m_szRcvBuff);
	if(m_szSendBuff != NULL)
        free(m_szSendBuff);

    return 0;
}

// 함수..
 
char * CVcUdp::GetMyIPAddress ()
{
    struct hostent* myhost;    
    struct in_addr myinaddr;

    char szHostName [40];

    int iResult = gethostname (szHostName, sizeof (szHostName));
    if (iResult == -1) {
        printf (" gethostname failed\n");
             return NULL;
    }
    if (strcmp (szHostName, "") == 0) {        
        printf (" gethostname's name is invalid\n");
             return NULL;
    }
    printf (" host name is %s\n", szHostName);

    myhost = gethostbyname (szHostName);
    if (myhost == NULL) {
        printf (" gethostname failed\n");
        return NULL;
    }
    printf  (" official host name : %s\n", myhost->h_name);
    printf  (" host address type : %d\n", myhost->h_addrtype);
    printf  (" length of host address : %d\n", myhost->h_length);

	int n = 0;
    while (myhost->h_addr_list [n] != NULL) {
        myinaddr.s_addr = *((unsigned long*) (myhost->h_addr_list [n]));
        printf  ("MY IP address %d : %s\n", n+1, inet_ntoa (myinaddr));

		// 본 시스템 운용과 관련된 IP인지 확인
        if((myinaddr.s_addr&0x000000ff) == IPv4_S_B1 &&
            (myinaddr.s_addr&0x0000ff00)>(8) == IPv4_S_B2 &&
            (myinaddr.s_addr&0x00ff0000)>(8*2) == IPv4_S_B3)
			return inet_ntoa(myinaddr);
        n++;
    }

	// 약속된 IP 그룹이 없을 때, 맨 마지막에 읽힌 ip address를 보낸다.
    //return inet_ntoa(myinaddr);
    return "192.168.11.1";
}

byte ** CVcUdp::cDim2D(int row, int column)
{
    int  i;
    long size, step;
    byte *buf_1d;
    byte **buf_2d;

    buf_2d = (byte **)calloc( row, sizeof(byte *) );
    if ( buf_2d == NULL ) return NULL;
    size = (long)row*column;
    buf_1d = (byte *)calloc( size, sizeof(byte) );
    if ( buf_1d == NULL ) return NULL;
    step = (long)column;
    for ( i = 0 ; i < row ; i++, buf_1d += step )
        buf_2d[i] = buf_1d;
    return (buf_2d);
}
 
void CVcUdp::dimFree(byte **buf_2d)
{
    free( buf_2d[0] );
    free( buf_2d );
}

int CVcUdp::SetCameraInformation(CConfigInfo configInfo, int type)
{
	int n;
	for(n=0; n<m_nNumSavedCamInfo; n++)
	{
		if(strcmp(m_clConfigInfo[n].m_bCameraMacAddr, configInfo.m_bCameraMacAddr) == 0) 
			break;
	}

	if(n < m_nNumSavedCamInfo) // 저장된 정보 중에서 찾았다면
	{
		switch(type)
		{
			case(CAM_MSG_CONFIG_PN_SN_REVISION):
				break;
			case(CAM_MSG_CONFIG_CAM_IP_ADDR):
				memcpy(m_clConfigInfo[n].m_strCameraIpAddr, configInfo.m_strCameraIpAddr, sizeof(configInfo.m_strCameraIpAddr));
				break;
			case(CAM_MSG_CONFIG_CAM_SUBNET_MASK):
				memcpy(m_clConfigInfo[n].m_strCameraSubnetMask, configInfo.m_strCameraSubnetMask, sizeof(configInfo.m_strCameraSubnetMask));			
				break;
			case(CAM_MSG_CONFIG_CAM_GATEWAY_ADDR):
				memcpy(m_clConfigInfo[n].m_strCameraGwAddr, configInfo.m_strCameraGwAddr, sizeof(configInfo.m_strCameraGwAddr));			
				break;
			case(CAM_MSG_CONFIG_HOST_IP_ADDR):
				memcpy(m_clConfigInfo[n].m_strHostIpAddr, configInfo.m_strHostIpAddr, sizeof(configInfo.m_strHostIpAddr));
				break;
			case(CAM_MSG_CONFIG_CAM_PORT_NO):
				m_clConfigInfo[n].m_nCommPort = configInfo.m_nCommPort;
				break;			
			case(CAM_MSG_CONFIG_CAM_NICKNAME1):
				memcpy(m_clConfigInfo[n].m_strCameraNick1, configInfo.m_strCameraNick1, sizeof(configInfo.m_strCameraNick1));
				break;
			case(CAM_MSG_CONFIG_CAM_NICKNAME2):
				memcpy(m_clConfigInfo[n].m_strCameraNick2, configInfo.m_strCameraNick2, sizeof(configInfo.m_strCameraNick2));
				break;
			default: break;
		}
	}
	else	// 기 저장된 정보에 없는 새로운 CAM 정보이면 
	{
		n = m_nNumSavedCamInfo++;

		memcpy(m_clConfigInfo[n].m_bCameraMacAddr, configInfo.m_bCameraMacAddr, sizeof(configInfo.m_bCameraMacAddr));
		switch(type){
			case(CAM_MSG_CONFIG_PN_SN_REVISION):
				break;
			case(CAM_MSG_CONFIG_CAM_IP_ADDR):
				memcpy(m_clConfigInfo[n].m_strCameraIpAddr, configInfo.m_strCameraIpAddr, sizeof(configInfo.m_strCameraIpAddr));
				break;
			case(CAM_MSG_CONFIG_CAM_SUBNET_MASK):
				memcpy(m_clConfigInfo[n].m_strCameraSubnetMask, configInfo.m_strCameraSubnetMask, sizeof(configInfo.m_strCameraSubnetMask));			
				break;
			case(CAM_MSG_CONFIG_CAM_GATEWAY_ADDR):
				memcpy(m_clConfigInfo[n].m_strCameraGwAddr, configInfo.m_strCameraGwAddr, sizeof(configInfo.m_strCameraGwAddr));			
				break;
			case(CAM_MSG_CONFIG_HOST_IP_ADDR):
				memcpy(m_clConfigInfo[n].m_strHostIpAddr, configInfo.m_strHostIpAddr, sizeof(configInfo.m_strHostIpAddr));
				break;
			case(CAM_MSG_CONFIG_CAM_PORT_NO):
				m_clConfigInfo[n].m_nCommPort = configInfo.m_nCommPort;
				break;			
			case(CAM_MSG_CONFIG_CAM_NICKNAME1):
				memcpy(m_clConfigInfo[n].m_strCameraNick1, configInfo.m_strCameraNick1, sizeof(configInfo.m_strCameraNick1));
				break;
			case(CAM_MSG_CONFIG_CAM_NICKNAME2):
				memcpy(m_clConfigInfo[n].m_strCameraNick2, configInfo.m_strCameraNick2, sizeof(configInfo.m_strCameraNick2));
				break;
			default: break;
		}
	}	
	return 1;
}
