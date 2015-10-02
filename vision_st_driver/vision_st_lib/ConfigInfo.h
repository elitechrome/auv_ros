#ifndef __CONFIGINFO_H
#define __CONFIGINFO_H
#include "define.h"


#define INFO_FILENAME "./myConfigInfo.csv"

#define DEFAULT_NICKNAME1	"NICK1"
#define DEFAULT_NICKNAME2	"NICK2"
#if IS_FIRST_MANAGER
#define DEFAULT_CAMERA_IP_ADDR "192.168.11.71"
#else
#define DEFAULT_CAMERA_IP_ADDR "192.168.11.71"
#endif
#define DEFAULT_SUBNET_MASK	"255.255.255.0"

// default port no는 define.h와의 혼선을 피하기위해 define.h에
// 선언된 DEFAULT_CMD_RCV_PORT를 사용함.

class CConfigInfo
{
public:
	char m_strCameraNick1[8];
	char m_strCameraNick2[8];
	char m_bCameraMacAddr[16];
	char m_strCameraIpAddr[16];
	char m_strCameraSubnetMask[16];
	char m_strCameraGwAddr[16];
	int m_nCommPort;
	char m_strHostIpAddr[16];

public:
	bool ReadConfigInfo(const char *myIpAddress);
	bool ReadConfigDataFromIniFile(const char *myIpAddress);
	bool WriteConfigInfo();

	CConfigInfo(void);
	~CConfigInfo(void);

protected:
	void InitVariables();

};

#endif
