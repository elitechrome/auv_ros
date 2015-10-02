#ifndef __CMDPACKET_H
#define __CMDPACKET_H
#include "VcUdp.h"

class CCmdPacket
{
public:
	CVcUdp *m_classCmdProcUdp, *m_classResponseUdp;
	char m_szCmdPacket[BYTES_CMD_PACKET+1];



public:
	CCmdPacket(CVcUdp *cmdProcUdp, CVcUdp *cmdResponseUdp);
	void ReInit(CVcUdp *cmdResponseUdp);

	// command processing functions
	int VstCmdFindCam(const char *destinationIP);
	int VstCmdSetCamConfigInfo(const char *nick1, const char *nick2,
		const char *macAddr, const char *ipAddr, const char *subnetMask, const char *gwIpAddr,
		int portNo, const char *hostIpAddr, const char *destinationIP);
	int VstCmdInitialize(const char *destinationIP);
	int VstCmdReset(const char *destinationIP);
	int VstCmdStart(const char *destinationIP);
	int VstCmdStop(const char *destinationIP);
	int VstCmdGetStatus_IsStarted(const char *destinationIP);
	int VstCmdSetGlobalGain(unsigned int value, const char *destinationIP);
	int VstCmdSetExposure(unsigned int value,  const char *destinationIP);
	int VstCmdSetAecEnabled(const char *destinationIP);
	int VstCmdSetAecDisabled(const char *destinationIP);
	int VstCmdSetTo69_2Hz(const char *destinationIP);
	
	~CCmdPacket(void);

protected:
	int UDP_data1_data2_to_15B_cmd_TXD(char data1[7], char data2[7], const char *destinationIP);
	int mt9m025_reg_wr(unsigned char slave_addr, unsigned int sub_addr, unsigned int data, const char *destinationIP);
	int vita1300_reg_wr(unsigned int addr, unsigned int data, const char *destinationIP);
	unsigned char hex_to_int(char c);
	int CheckResponse(char *data2);

};

#endif
