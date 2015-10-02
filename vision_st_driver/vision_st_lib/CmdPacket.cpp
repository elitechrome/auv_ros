#include "CmdPacket.h"
#include <cstdio>

CCmdPacket::CCmdPacket(CVcUdp *cmdProcUdp, CVcUdp *cmdResponseUdp)
{
	m_classCmdProcUdp = cmdProcUdp;
	m_classResponseUdp = cmdResponseUdp;

	memset(m_szCmdPacket, 0, sizeof(m_szCmdPacket));
    memset(m_szCmdPacket, 0, sizeof(m_szCmdPacket));
}

CCmdPacket::~CCmdPacket(void)
{
}

void CCmdPacket::ReInit(CVcUdp *cmdResponseUdp)
{
	m_classResponseUdp = cmdResponseUdp;
}

int CCmdPacket::UDP_data1_data2_to_15B_cmd_TXD(char data1[7], char data2[7], const char *destinationIP)
{
	//data1/2 convert to cmd[15]
	//HEADER
	m_szCmdPacket[   0] = 0x81;

	//data1 to cmd
	m_szCmdPacket[1+ 0] =                          ((data1[0] & 0xFE)>>1);	//                            data1[0]H7=cmd[0+0]x1111111
	m_szCmdPacket[1+ 1] = ((data1[0] & 0x01)<<6) | ((data1[1] & 0xFC)>>2);	//data1[0]L1=cmd[0+1]x1xxxxxx,data1[1]H6=cmd[0+1]xx111111
	m_szCmdPacket[1+ 2] = ((data1[1] & 0x03)<<5) | ((data1[2] & 0xF8)>>3);	//data1[1]L2=cmd[0+2]x11xxxxx,data1[2]H5=cmd[0+2]xxx11111
	m_szCmdPacket[1+ 3] = ((data1[2] & 0x07)<<4) | ((data1[3] & 0xF0)>>4);	//data1[2]L3=cmd[0+3]x111xxxx,data1[3]H4=cmd[0+3]xxxx1111
	m_szCmdPacket[1+ 4] = ((data1[3] & 0x0F)<<3) | ((data1[4] & 0xE0)>>5);	//data1[3]L4=cmd[0+4]x1111xxx,data1[4]H3=cmd[0+4]xxxxx111
	m_szCmdPacket[1+ 5] = ((data1[4] & 0x1F)<<2) | ((data1[5] & 0xC0)>>6);	//data1[4]L5=cmd[0+5]x11111xx,data1[5]H2=cmd[0+5]xxxxxx11
	m_szCmdPacket[1+ 6] = ((data1[5] & 0x3F)<<1) | ((data1[6] & 0x01)>>0);	//data1[5]L6=cmd[0+6]x111111x,data1[6]L1=cmd[0+6]xxxxxxx1

	//data2 to cmd
	m_szCmdPacket[8+ 0] =                          ((data2[0] & 0xFE)>>1);	//                            data2[0]H7=cmd[7+0]x1111111
	m_szCmdPacket[8+ 1] = ((data2[0] & 0x01)<<6) | ((data2[1] & 0xFC)>>2);	//data2[0]L1=cmd[7+1]x1xxxxxx,data2[1]H6=cmd[7+1]xx111111
	m_szCmdPacket[8+ 2] = ((data2[1] & 0x03)<<5) | ((data2[2] & 0xF8)>>3);	//data2[1]L2=cmd[7+2]x11xxxxx,data2[2]H5=cmd[7+2]xxx11111
	m_szCmdPacket[8+ 3] = ((data2[2] & 0x07)<<4) | ((data2[3] & 0xF0)>>4);	//data2[2]L3=cmd[7+3]x111xxxx,data2[3]H4=cmd[7+3]xxxx1111
	m_szCmdPacket[8+ 4] = ((data2[3] & 0x0F)<<3) | ((data2[4] & 0xE0)>>5);	//data2[3]L4=cmd[7+4]x1111xxx,data2[4]H3=cmd[7+4]xxxxx111
	m_szCmdPacket[8+ 5] = ((data2[4] & 0x1F)<<2) | ((data2[5] & 0xC0)>>6);	//data2[4]L5=cmd[7+5]x11111xx,data2[5]H2=cmd[7+5]xxxxxx11
	m_szCmdPacket[8+ 6] = ((data2[5] & 0x3F)<<1) | ((data2[6] & 0x01)>>0);	//data2[5]L6=cmd[7+6]x111111x,data2[6]L1=cmd[7+6]xxxxxxx1

	m_classCmdProcUdp->SendData(m_szCmdPacket, BYTES_CMD_PACKET, destinationIP);
    printf("Data1 : ");
    for(int i =0; i < 8; i++)
        printf("%02d ",data1[i]);
    printf("\n");
    printf("Data2 : ");
    for(int i =0; i < 8; i++)
        printf("%02d ",data2[i]);
    printf("\n");

    printf("SendData : ");
    for(int i =0; i < BYTES_CMD_PACKET; i++)
        printf("%02d ",(unsigned char)m_szCmdPacket[i]);
    printf("\n");
    usleep(ms_UDP_CMD_DELAY*1000);
    usleep(ms_UDP_CMD_DELAY*1000);

	int rtn = CMD_OK;
	if(data2[0] == CAM_MSG_IIC_RD_WR1 || 
		data2[0] == CAM_MSG_IIC_RD_WR2 || 
		data2[0] == CAM_MSG_VITA1300_REG_RD_WR ||
		data2[0] == CAM_MSG_REBOOT)
	{
		rtn = CheckResponse(data2);
	}
		
	return rtn;
}

int CCmdPacket::CheckResponse(char *data2)
{
	for(int i=0; i<3; i++)
	{
		char responsedMsg[BYTES_RESPONSE_PACKET];
		memcpy(responsedMsg, m_classResponseUdp->m_szRcvBuff, BYTES_RESPONSE_PACKET);
		int rtn = CMD_OK;

		switch(data2[0])
		{
		case CAM_MSG_IIC_RD_WR1:
			if(data2[1] != responsedMsg[9])
				rtn = CMD_ERROR;
			else if(data2[2] != responsedMsg[10])
				rtn = CMD_ERROR;
			break;
		case CAM_MSG_IIC_RD_WR2:
			if(data2[1] != responsedMsg[9])
				rtn = CMD_ERROR;
			else if(data2[2] != responsedMsg[10])
				rtn = CMD_ERROR;
			else if(data2[3] != responsedMsg[11])
				rtn = CMD_ERROR;
			break;
		case CAM_MSG_VITA1300_REG_RD_WR:
			if(data2[1] != responsedMsg[9])
				rtn = CMD_ERROR;
			else if(data2[2] != responsedMsg[10])
				rtn = CMD_ERROR;
			else if(data2[3] != responsedMsg[11])
				rtn = CMD_ERROR;
			else if(data2[4] != responsedMsg[12])
				rtn = CMD_ERROR;
			break;
		default:
			if(data2[0] != responsedMsg[14])
				rtn = CMD_ERROR;
			break;
		}
        printf("CheckResponse : ");
        for(int i =0; i < BYTES_RESPONSE_PACKET; i++)
            printf("%02d ",(unsigned char)responsedMsg[i]);
        printf("\n");
        if(rtn == CMD_OK) return rtn;
        usleep(ms_UDP_CMD_DELAY*1000);
	}

	return CMD_ERROR;
}

int CCmdPacket::mt9m025_reg_wr(unsigned char slave_addr, unsigned int sub_addr, unsigned int data, const char *destinationIP)
{
	char data1[7]={0,0,0,0,0,0,0};
	char data2[7]={0,0,0,0,0,0,0};

	unsigned char rd_wr = 0;	//0:wr

	data2[0] = CAM_MSG_MT9M025_REG_WR; //0x18 => mt9m025_reg_wr(IIC_gpio_write_2byte_2byte_slaveaddr)
	data2[1] = (sub_addr>>8)&0xFF;	//addr H
	data2[2] = (sub_addr>>0)&0xFF;	//addr L
	data2[3] = (data>>8)&0xFF;	//data H
	data2[4] = (data>>0)&0xFF;	//data L
	data2[5] = rd_wr;			//rd_wr, no effect

	data1[1] = slave_addr;//0x30;			//0x30 => mt9m025 CIS slave addr

	int rtn = UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);

	return rtn;
}

int CCmdPacket::vita1300_reg_wr(unsigned int addr, unsigned int data, const char *destinationIP)
{
	char data1[7]={0,0,0,0,0,0,0};
	char data2[7]={0,0,0,0,0,0,0};

	unsigned char rd_wr = 0;	//0:wr

	data2[0] = CAM_MSG_VITA1300_REG_RD_WR;//0x13 => vita1300 reg rd_wr
	data2[1] = (addr>>8)&0xFF;	//addr H
	data2[2] = (addr>>0)&0xFF;	//addr L
	data2[3] = (data>>8)&0xFF;	//data H
	data2[4] = (data>>0)&0xFF;	//data L
	data2[5] = rd_wr;			//rd_wr

	int rtn = UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);

	return rtn;
}

unsigned char CCmdPacket::hex_to_int(char c)
{
	if(c == '0')	return 0x0;
	if(c == '1')	return 0x1;
	if(c == '2')	return 0x2;
	if(c == '3')	return 0x3;
	if(c == '4')	return 0x4;
	if(c == '5')	return 0x5;
	if(c == '6')	return 0x6;
	if(c == '7')	return 0x7;
	if(c == '8')	return 0x8;
	if(c == '9')	return 0x9;
	if(c == 'A')	return 0xA;
	if(c == 'B')	return 0xB;
	if(c == 'C')	return 0xC;
	if(c == 'D')	return 0xD;
	if(c == 'E')	return 0xE;
	if(c == 'F')	return 0xF;
	if(c == 'a')	return 0xA;
	if(c == 'b')	return 0xB;
	if(c == 'c')	return 0xC;
	if(c == 'd')	return 0xD;
	if(c == 'e')	return 0xE;
	if(c == 'f')	return 0xF;
	else			return 0xFF;
}

////////////////////////////////////////////////////////////////////
int CCmdPacket::VstCmdFindCam(const char *destinationIP)
{
	char data1[7]={0,0,0,0,0,0,0};
	char data2[7]={0,0,0,0,0,0,0};

	data1[6] = 0x01;//0x01 => connection CMD
	data2[0] = CAM_MSG_FIND;//0x01 => 0x01 => CAMERA?, 무조건 처리

	UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);

	return CMD_OK;
}

int CCmdPacket::VstCmdSetCamConfigInfo(const char *nick1, const char *nick2,
	const char *macAddr, const char *ipAddr, const char *subnetMask, const char *gwIpAddr,
	int portNo, const char *hostIpAddr, const char *destinationIP)
{
	char data1[7]={0,0,0,0,0,0,0};
	char data2[7]={0,0,0,0,0,0,0};

	data1[0] = (hex_to_int(macAddr[ 0])<<4) | (hex_to_int(macAddr[ 1])<<0);
	data1[1] = (hex_to_int(macAddr[ 2])<<4) | (hex_to_int(macAddr[ 3])<<0);
	data1[2] = (hex_to_int(macAddr[ 4])<<4) | (hex_to_int(macAddr[ 5])<<0);
	data1[3] = (hex_to_int(macAddr[ 6])<<4) | (hex_to_int(macAddr[ 7])<<0);
	data1[4] = (hex_to_int(macAddr[ 8])<<4) | (hex_to_int(macAddr[ 9])<<0);
	data1[5] = (hex_to_int(macAddr[10])<<4) | (hex_to_int(macAddr[11])<<0);
	data1[6] = 0x01;//0x01 => connection CMD

    struct sockaddr_in a;
	a.sin_addr.s_addr = inet_addr(ipAddr);
	//0x03 => s_IP1_ADDRv4 config
	data2[0] = CAM_MSG_CONFIG_CAM_IP_ADDR;//0x03 => s_IP1_ADDRv4 config
    sscanf(inet_ntoa(a.sin_addr),"%d.%d.%d.%d", &data2[1], &data2[2], &data2[3], &data2[4]);
//    data2[1] = a.sin_addr.S_un.S_un_b.s_b1;//
//	data2[2] = a.sin_addr.S_un.S_un_b.s_b2;//
//	data2[3] = a.sin_addr.S_un.S_un_b.s_b3;//
//	data2[4] = a.sin_addr.S_un.S_un_b.s_b4;//
    UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);

	a.sin_addr.s_addr = inet_addr(subnetMask);
	//0x04 => s_CAM_SUBNET_MASK config
	data2[0] = CAM_MSG_CONFIG_CAM_SUBNET_MASK;//0x04 => s_CAM_SUBNET_MASK config
    sscanf(inet_ntoa(a.sin_addr),"%d.%d.%d.%d", &data2[1], &data2[2], &data2[3], &data2[4]);
//	data2[1] = a.sin_addr.S_un.S_un_b.s_b1;//
//	data2[2] = a.sin_addr.S_un.S_un_b.s_b2;//
//	data2[3] = a.sin_addr.S_un.S_un_b.s_b3;//
//	data2[4] = a.sin_addr.S_un.S_un_b.s_b4;//
	UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);

	a.sin_addr.s_addr = inet_addr(gwIpAddr);
	//0x05 => s_CAM_GATEWAY_IP_ADDR config
	data2[0] = CAM_MSG_CONFIG_CAM_GATEWAY_ADDR;//0x05 => s_CAM_GATEWAY_IP_ADDR config
    sscanf(inet_ntoa(a.sin_addr),"%d.%d.%d.%d", &data2[1], &data2[2], &data2[3], &data2[4]);
//	data2[1] = a.sin_addr.S_un.S_un_b.s_b1;//
//	data2[2] = a.sin_addr.S_un.S_un_b.s_b2;//
//	data2[3] = a.sin_addr.S_un.S_un_b.s_b3;//
//	data2[4] = a.sin_addr.S_un.S_un_b.s_b4;//
	UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);

	a.sin_addr.s_addr = inet_addr(hostIpAddr);
	//0x06 => s_CAM_UDP_TX_DEST_IP_ADDR config
	data2[0] = CAM_MSG_CONFIG_HOST_IP_ADDR;//0x06 => s_CAM_UDP_TX_DEST_IP_ADDR config
    sscanf(inet_ntoa(a.sin_addr),"%d.%d.%d.%d", &data2[1], &data2[2], &data2[3], &data2[4]);
//	data2[1] = a.sin_addr.S_un.S_un_b.s_b1;//
//	data2[2] = a.sin_addr.S_un.S_un_b.s_b2;//
//	data2[3] = a.sin_addr.S_un.S_un_b.s_b3;//
//	data2[4] = a.sin_addr.S_un.S_un_b.s_b4;//
	UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);


	//0x07 => udp_tx2_control_port config
	data2[0] = CAM_MSG_CONFIG_CAM_PORT_NO;//0x07 => udp_tx2_control_port config
	data2[1] = (portNo>>8)&0xFF;//
	data2[2] = (portNo>>0)&0xFF;//
	UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);

	//0x14 => EEPOM wr NICK_NAME1 5B write
	data2[0] = CAM_MSG_CONFIG_CAM_NICKNAME1;//0x14 => EEPOM wr NICK_NAME1 5B write
	data2[1] = nick1[0];//'a';//
	data2[2] = nick1[1];//'b';//
	data2[3] = nick1[2];//'c';//
	data2[4] = nick1[3];//'d';//
	data2[5] = nick1[4];//'e';//
	UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);
    usleep(50*1000);//additional delay for NICK_NAME1 writing

	//0x15 => EEPOM wr NICK_NAME2 5B write
	data2[0] = CAM_MSG_CONFIG_CAM_NICKNAME2;//0x15 => EEPOM wr NICK_NAME1 5B write
	data2[1] = nick2[0];//'A';//
	data2[2] = nick2[1];//'B';//
	data2[3] = nick2[2];//'C';//
	data2[4] = nick2[3];//'D';//
	data2[5] = nick2[4];//'E';//
	UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);
    usleep(50*1000);//additional delay for NICK_NAME2 writing

	return CMD_OK;
}

int CCmdPacket::VstCmdInitialize(const char *destinationIP)
{
	char data1[7]={0,0,0,0,0,0,0};
	char data2[7]={0,0,0,0,0,0,0};
	int rtn;

	// 초기화 실패를 고려하여 2번까지 시도
	for(int i=0; i<2; i++)
	{
		data2[0] = CAM_MSG_IIC_RD_WR1;//0x11 => iic write, slave addr, sub addr, 1byte
		data2[1] = 0x20;// => FPGA slave addr
		data2[2] = 0x00;// => reg(0)(0)
		data2[3] = 0x00;// => s_started 
		rtn = UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);
		if(rtn == CMD_ERROR) continue;

		//FPGA0 s_BUFFER_SIZE settings
		data2[0] = CAM_MSG_IIC_RD_WR1;//0x11 => iic write, slave addr, sub addr, 1byte
		data2[1] = 0x20;// => FPGA0 slave addr
		data2[2] = 44;// => reg(44)
		data2[3] = (X_WIDTH>>8);// => s_BUFFER_SIZE H
		rtn = UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);
		if(rtn == CMD_ERROR) continue;

		data2[0] = CAM_MSG_IIC_RD_WR1;//0x11 => iic write, slave addr, sub addr, 1byte
		data2[1] = 0x20;// => FPGA0 slave addr
		data2[2] = 45;// => reg(45)
		data2[3] = (X_WIDTH>>0);// => s_BUFFER_SIZE L
		rtn = UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);
		if(rtn == CMD_ERROR) continue;

		////FPGA1 s_BUFFER_SIZE settings
		//data2[0] = CAM_MSG_IIC_RD_WR1;//0x11 => iic write, slave addr, sub addr, 1byte
		//data2[1] = 0x10;// => FPGA1 slave addr
		//data2[2] = 44;// => reg(44)
		//data2[3] = (X_WIDTH>>8);// => s_BUFFER_SIZE H
		//rtn = UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);
		//if(rtn == CMD_ERROR) continue;

		//data2[0] = CAM_MSG_IIC_RD_WR1;//0x11 => iic write, slave addr, sub addr, 1byte
		//data2[1] = 0x10;// => FPGA1 slave addr
		//data2[2] = 45;// => reg(45)
		//data2[3] = (X_WIDTH>>0);// => s_BUFFER_SIZE L
		//rtn = UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);
		//if(rtn == CMD_ERROR) continue;

		data2[0] = CAM_MSG_IIC_RD_WR2;//0x12 => iic write, s_transfer_size 3B wr
		data2[1] = (S_TRANSFER_SIZE>>16)&0xFF;// => H
		data2[2] = (S_TRANSFER_SIZE>> 8)&0xFF;// => 
		data2[3] = (S_TRANSFER_SIZE>> 0)&0xFF;// => L
		rtn = UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);
		if(rtn == CMD_ERROR) continue;


		//20150707, r_add_sync, 25 & 26
        long r_add_sync = (R_ADD_SYNC)/4;	//X B, X/4 clk
		unsigned int r_add_sync_plus1 = 100+ r_add_sync;	//4 byte per clk, + 100(default value)
		r_add_sync_plus1 |= (1<<15);		//enable write

		data2[0] = CAM_MSG_IIC_RD_WR1;//0x11 => iic write, slave addr, sub addr, 1byte
		data2[1] = 0x20;// => FPGA0 slave addr
		data2[2] = 47;// => reg(47), r_add_sync H
		data2[3] = (unsigned char)((r_add_sync_plus1>> 8)&0xFF);
		rtn = UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);
		if(rtn == CMD_ERROR) continue;

		data2[0] = CAM_MSG_IIC_RD_WR1;//0x11 => iic write, slave addr, sub addr, 1byte
		data2[1] = 0x20;// => FPGA0 slave addr
		data2[2] = 48;// => reg(48), r_add_sync L
		data2[3] = (unsigned char)((r_add_sync_plus1>> 0)&0xFF);
		rtn = UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);
		if(rtn == CMD_ERROR) continue;

		break;
	}
	return rtn;
}

int CCmdPacket::VstCmdReset(const char *destinationIP)
{
	char data1[7]={0,0,0,0,0,0,0};
	char data2[7]={0,0,0,0,0,0,0};

	data2[0] = CAM_MSG_REBOOT;//0x17 => cmd_reboot
	data2[1] = 0x01;// cmd_reboot = 1

	int rtn = UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);

	return rtn;
}

int CCmdPacket::VstCmdStart(const char *destinationIP)
{
	char data1[7]={0,0,0,0,0,0,0};
	char data2[7]={0,0,0,0,0,0,0};

	data2[0] = CAM_MSG_IIC_RD_WR1;//0x11 => iic write, slave addr, sub addr, 1byte
	data2[1] = 0x20;// => FPGA slave addr
	data2[2] = 0x00;// => reg(0)(0)
	data2[3] = 0x01;// => s_started 

	int rtn = UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);

	return rtn;
}


int CCmdPacket::VstCmdStop(const char *destinationIP)
{
	char data1[7]={0,0,0,0,0,0,0};
	char data2[7]={0,0,0,0,0,0,0};

	data2[0] = CAM_MSG_IIC_RD_WR1;//0x11 => iic write, slave addr, sub addr, 1byte
	data2[1] = 0x20;// => FPGA slave addr
	data2[2] = 0x00;// => reg(0)(0)
	data2[3] = 0x00;// => s_started 

	int rtn = UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);

	return rtn;
}

int CCmdPacket::VstCmdGetStatus_IsStarted(const char *destinationIP)
{
	char data1[7]={0,0,0,0,0,0,0};
	char data2[7]={0,0,0,0,0,0,0};

	data2[0] = CAM_MSG_IIC_RD_WR1;//0x11 => iic write, slave addr, sub addr, 1byte
	data2[1] = 0x20;// => FPGA slave addr
	data2[2] = 0x00;// => reg(0)(0)
	data2[3] = 0x00;// => s_started 
	data2[4] = 0x01;// => read reg

	UDP_data1_data2_to_15B_cmd_TXD(data1, data2, destinationIP);

	return CMD_OK;
}

int CCmdPacket::VstCmdSetAecEnabled(const char *destinationIP)
{
	int rtn = vita1300_reg_wr(160,	16 | (1<<0), destinationIP); //AEC[Block Offset: 160]

	return rtn;
}

int CCmdPacket::VstCmdSetAecDisabled(const char *destinationIP)
{
	int rtn = vita1300_reg_wr(160,	16 | (0<<0), destinationIP); //AEC[Block Offset: 160]

	return rtn;
}

int CCmdPacket::VstCmdSetTo69_2Hz(const char *destinationIP)
{
	int rtn = vita1300_reg_wr(197,	255, destinationIP);	//black_lines, [7:0]
	if(rtn == CMD_OK)
		rtn = vita1300_reg_wr(198,	32, destinationIP);			//dummy_lines, [11:0]

	return rtn;
}

int CCmdPacket::VstCmdSetGlobalGain(unsigned int value, const char *destinationIP)
{
	int rtn = mt9m025_reg_wr(0x30, 0x305E, value, destinationIP);

	return rtn;
}

int CCmdPacket::VstCmdSetExposure(unsigned int value,  const char *destinationIP)
{
	int rtn = mt9m025_reg_wr(0x30, 0x3012, value, destinationIP);	//exposure
	return rtn;
}
