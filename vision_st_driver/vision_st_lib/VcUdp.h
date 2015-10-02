// Udp.h: interface for the CVcUdp class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __VPUDP_H
#define __VPUDP_H

#include "define.h"
#include "ConfigInfo.h"
#include "arpa/inet.h"
#include "sys/socket.h"
#include "netdb.h"
#include <unistd.h>
#include <cstring>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
typedef uint8_t byte;
class CVcUdp
{
public:
	// UDP ��� ����
    int m_sUdp, m_cUdp;
    struct sockaddr_in m_sinUdpS, m_sinUdpC;
	short m_sPortUdp;
    socklen_t m_nSvrSockInLen, m_nClientSockInLen;
	char *m_szRcvBuff, *m_szSendBuff;
	int m_nBytesToRcv;
	int m_nRcvDataLen, m_nSendDataLen;
    int m_bIngTx, m_bIngRx, m_bDidTx, m_bDidRx;
	bool m_bReceiveRunFlag;

	// Reponse �޽��� ó����
	byte *m_szRcvResponseMsgBuff;
	CConfigInfo *m_clConfigInfo;
	int m_nNumSavedCamInfo;
	int m_nIsCamStatusRun;

	// �̹��� ������ ���� ����
	byte *m_ucpImg, *m_ucpImgCopy; 

	// �̹��� ������ ó�� ����
	unsigned short m_usFrameCount;
	unsigned short m_usPacketNo;
	bool m_bIsSignificantFault;

	float m_fFrameRate;
	unsigned long	m_ulGoodFrameCount;
	unsigned long	m_ulBadFrameCount;

    boost::thread *m_thread;
    boost::mutex m_mutex;
// methods
public:
	bool InitUdpSock(int portNo, int bytesToRcv, bool isServer);
	void OnReceiveUdpResponseMsg();
	void OnReceiveUdpImgData();
	bool SendData(char *buff, int len, const char *destination);
	char * GetMyIPAddress ();

	CVcUdp();
	virtual ~CVcUdp();
    int Run();
    int ExitInstance();
    void ResumeThread();


protected:
	void SetSocketOptions();
	void CloseUdpSocket();	byte ** cDim2D(int row, int column);
	void dimFree(byte **buf_2d);
	void ImageDataProc();
	int SetCameraInformation(CConfigInfo configInfo, int type);
};
#endif // __VPUDP_H
