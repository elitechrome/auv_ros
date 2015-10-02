#include "ConfigInfo.h"
#include "IniParser.h"
#include "arpa/inet.h"
#include <stdlib.h>

CConfigInfo::CConfigInfo(void)
{
	InitVariables();
}


CConfigInfo::~CConfigInfo(void)
{
}

void CConfigInfo::InitVariables()
{
	memset(m_strCameraNick1, 0, sizeof(m_strCameraNick1));
	memset(m_strCameraNick2, 0, sizeof(m_strCameraNick2));
	memset(m_bCameraMacAddr, 0, sizeof(m_bCameraMacAddr));
	memset(m_strCameraIpAddr, 0, sizeof(m_strCameraIpAddr));
	memset(m_strCameraSubnetMask, 0, sizeof(m_strCameraSubnetMask));
	memset(m_strCameraGwAddr, 0, sizeof(m_strCameraGwAddr));
	m_nCommPort = 0;
	memset(m_strHostIpAddr, 0, sizeof(m_strHostIpAddr));

	strcpy(m_strCameraIpAddr, DEFAULT_CAMERA_IP_ADDR);
	strcpy(m_strCameraSubnetMask, DEFAULT_SUBNET_MASK);
	m_nCommPort = DEFAULT_CMD_RCV_PORT;
}

bool CConfigInfo::ReadConfigInfo(const char *myIpAddress)
{
	FILE	*fp;
	char	buff[128];
	int		sizeofBuff = sizeof(buff);

	if((fp = fopen(INFO_FILENAME, "r")) != NULL){
		memset(buff, 0x00, sizeofBuff);
		while(fgets(buff, sizeofBuff, fp) != NULL) {
			int nLen = strlen(buff);
			char cIndex[32];
			int nCnt = 0;
			int nFirst = 0;
			int nLast = 0;

			for(int i=0;i<nLen;i++){
				if(buff[i] == ','){
					memset(cIndex, 0, sizeof(cIndex));
					memcpy(cIndex, buff+nFirst, i-nFirst);								
					if(nCnt == 0){
                        printf("[%s], ", cIndex);
						memcpy(m_strCameraNick1, cIndex, strlen(cIndex));
					}
					else if(nCnt == 1){
                        printf("[%s], ", cIndex);
						memcpy(m_strCameraNick2, cIndex, strlen(cIndex));
					}
					else if(nCnt == 2){
                        printf("[%s], ", cIndex);
						memcpy(m_bCameraMacAddr, cIndex, strlen(cIndex));
					}
					else if(nCnt == 3){
                        printf("[%s], ", cIndex);
						memcpy(m_strCameraIpAddr, cIndex, strlen(cIndex));
					}
					else if(nCnt == 4){
                        printf("[%s], ", cIndex);
						memcpy(m_strCameraSubnetMask, cIndex, strlen(cIndex));
					}
					else if(nCnt == 5){
                        printf("[%s], ", cIndex);
						memcpy(m_strCameraGwAddr, cIndex, strlen(cIndex));
					}
					else if(nCnt == 6){
                        printf("[%s], ", cIndex);
						m_nCommPort = atoi(cIndex);
					}
					else if(nCnt == 7){
                        printf("[%s]\n", cIndex);
						memcpy(m_strHostIpAddr, cIndex, strlen(cIndex));
						break;
					}
					nCnt++;
					nFirst = i+1;
				}
			}
		}
		if(fp != NULL) fclose(fp);

		// csv file 내에 있는 자기 IP와 앞에서 자동으로 찾은 자기 IP를 비교
		// 같지 않으면 csv을 잘못된 정보로 처리한다.
		if(strcmp(myIpAddress, m_strHostIpAddr) == 0)
		{
			return true;
		}
		else	// 같지 않으면
		{
			InitVariables();
			// ini file 읽기, ini file에서 자기 IP와 일치하는 내용으로 기본설정
			return(ReadConfigDataFromIniFile(myIpAddress));
		}

	}
	else{
		// ini file 읽기, ini file에서 자기 IP와 일치하는 내용으로 기본설정

        printf("csv 파일 열기 실패\n");
		return(ReadConfigDataFromIniFile(myIpAddress));
	}

	return true;
}


bool CConfigInfo::WriteConfigInfo()
{

	char	buff[128];
	int		sizeofBuff = sizeof(buff);

	FILE	*fp;

	fp = fopen(INFO_FILENAME, "w");

	if(fp != NULL){
        memset(buff, 0x00, sizeofBuff);
        snprintf(buff, sizeof(buff),"%s,%s,%s,%s,%s,%s,%d,%s,",
			m_strCameraNick1, m_strCameraNick2, m_bCameraMacAddr, 
			m_strCameraIpAddr, m_strCameraSubnetMask, m_strCameraGwAddr, m_nCommPort, m_strHostIpAddr);
		fprintf(fp, "%s\n", buff);
		fclose(fp);
    }
	else
	{
        printf("파일 쓰기 실패\n");
        return false;
	}

    return true;
}

bool CConfigInfo::ReadConfigDataFromIniFile(const char *myIpAddress)
{
	CIniParser iniParser("./Config.ini");
	struct sockaddr_in a;

	a.sin_addr.s_addr = inet_addr(myIpAddress);
    printf("MY LAST IP ID : %d\n", (a.sin_addr.s_addr&0xff000000)>(8*3));

	char sectionName[32];
	memset(sectionName, 0, sizeof(sectionName));
	
	// 해당 camera 찾기
    sprintf(sectionName, "Camera #%02d", (a.sin_addr.s_addr&0x11000000)>6);

	string nick1 = iniParser.Get(sectionName, "NICK_NAME1", "E");
	memcpy(m_strCameraNick1, nick1.data(), nick1.size());
    printf("%s NICK_NAME1 : %s\n", sectionName, m_strCameraNick1);

	string nick2 = iniParser.Get(sectionName, "NICK_NAME2", "E");
	memcpy(m_strCameraNick2, nick2.data(), nick2.size());
    printf("%s NICK_NAME2 : %s\n", sectionName, m_strCameraNick2);

	string macAddr = iniParser.Get(sectionName, "MAC_ADDR", "E");
	memcpy(m_bCameraMacAddr, macAddr.data(), macAddr.size());
    printf("%s MAC_ADDR : %s\n", sectionName, m_bCameraMacAddr);

	string ipAddr = iniParser.Get(sectionName, "IP_ADDR", "E");
	memcpy(m_strCameraIpAddr, ipAddr.data(), ipAddr.size());
    printf("%s IP_ADDR : %s\n", sectionName, m_strCameraIpAddr);

	string subNetMask = iniParser.Get(sectionName, "SUBNET_MASK", "E");
	memcpy(m_strCameraSubnetMask, subNetMask.data(), subNetMask.size());
    printf("%s SUBNET_MASK : %s\n", sectionName, m_strCameraSubnetMask);

	string gwIpAddr = iniParser.Get(sectionName, "GATEWAY_IP_ADDR", "E");
	memcpy(m_strCameraGwAddr, gwIpAddr.data(), gwIpAddr.size());
    printf("%s GATEWAY_IP_ADDR : %s\n", sectionName, m_strCameraGwAddr);

	m_nCommPort = iniParser.GetInteger(sectionName, "PORT", 0);
    printf("%s PORT : %d\n", sectionName, m_nCommPort);

	string hostIpAddr = iniParser.Get(sectionName, "HOST_IP_ADDR", "E");
	memcpy(m_strHostIpAddr, hostIpAddr.data(), hostIpAddr.size());
    printf("%s HOST_IP_ADDR : %s\n", sectionName, m_strHostIpAddr);

	if(strcmp(m_strHostIpAddr, myIpAddress) != 0 || m_nCommPort == 0)
	{
		InitVariables();
        return false;
	}

    return true;
}
