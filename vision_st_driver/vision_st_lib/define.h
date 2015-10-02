#ifndef _DEFINE_H
#define _DEFINE_H
// General
#define CMD_OK					1
#define	CMD_ERROR				-1

#define STATUS_NO_RESPONSE		-1
#define STATUS_STOP				0
#define STATUS_RUN				1	

// camera 개수
#define MAX_NUM_CAMERA			1

// Image Info.
#define MAX_SENSOR_FRAME_WIDTH	1280
#define FRAME_WIDTH				640		// Left(640) + Right(640)
#define FRAME_HEIGHT			360
#define BYTES_1_PIXEL			8		// (Color R(1)G(1)B(1)+1 bytes) * 2

// Image data to control CAMERA
#define X_WIDTH				1288	// + packet counter(8bytes)
#define Y_HEIGHT			(FRAME_HEIGHT*4)
#define R_ADD_SYNC			123648			//(1288*1440+123648)/1024
											//1288*1536%1288 = 0		packet transfer
											//1288*1536%1024 = 0		dram buffering
											//1288(1536-1440) = 123648
#define S_TRANSFER_SIZE		(X_WIDTH/4*Y_HEIGHT*4+R_ADD_SYNC)/16

// IP ADDR GROUP
#define IPv4_S_B1	192
#define IPv4_S_B2	168
#define IPv4_S_B3	11		
#define	IPv4_S_B4			// This Byte is variable
#define MULTICAST_IP_ADDR	"255.255.255.255"	

// UDP default receive PORTS
#define CMD_SEND_PORT					1029
#define FIND_CAMERA_RESULT_RCV_PORT		1030
#define DEFAULT_CMD_RCV_PORT			1033	
#define DEFAULT_IMG_DATA_RCV_PORT		(DEFAULT_CMD_RCV_PORT + 1)

// RCV packet, cmd packet
#define DEFAULT_SIZE_OF_PACKET		1500
#define BYTES_CMD_PACKET			15
#define BYTES_RESPONSE_PACKET		16
#define BYTES_IMAGE_PACKET_HADER	8
#define BYTES_PURE_IMAGE_IN_PACKET	1280
#define BYTES_IMAGE_DATA_PACKET		(BYTES_IMAGE_PACKET_HADER + BYTES_PURE_IMAGE_IN_PACKET)
#define ms_UDP_CMD_DELAY			50

// commands
#define CAM_MSG_FIND_BUFFER_INDEX_REG		0x07
#define CAM_MSG_FIND_BUFFER_INDEX			14

#define CAM_MSG_FIND						0x01
#define CAM_MSG_CONFIG_PN_SN_REVISION		0x02
#define CAM_MSG_CONFIG_CAM_IP_ADDR			0x03
#define CAM_MSG_CONFIG_CAM_SUBNET_MASK		0x04
#define CAM_MSG_CONFIG_CAM_GATEWAY_ADDR		0x05
#define CAM_MSG_CONFIG_HOST_IP_ADDR			0x06
#define CAM_MSG_CONFIG_CAM_PORT_NO			0x07
#define CAM_MSG_CONFIG_CAM_NICKNAME1		0x14
#define CAM_MSG_CONFIG_CAM_NICKNAME2		0x15
#define CAM_MSG_REBOOT						0x17

#define CAM_MSG_IIC_RD_WR1					0x11
#define CAM_MSG_IIC_RD_WR2					0x12
#define CAM_MSG_VITA1300_REG_RD_WR			0x13
#define CAM_MSG_MT9M025_REG_WR				0x18

#define DEFAULT_GLOBAL_GAIN					45
#define DEFAULT_EXPOSURE					10
#define GAIN_STEP	1
#define DEFAULT_DIVIDING_FACTOR	16
#define MIN_GLOBAL_GAIN 64
#define MAX_GLOBAL_GAIN 90
#define MIN_EXPOSURE 5
#define MAX_EXPOSURE 720
#define DEFAULT_AVG_INTENSITY 100

// ETC
#define ORIGINAL_PROGRAM_ANME		"CommWithVSTCamera"
#define MODIFIED_PROGRAM_NAME		"VSTCameraManager"
#define LOG_FOLDER_NAME				"LOG"	
#define IMAGE_FOLDER_NAME			"Images"
//#define IMAGE_FOLDER_NAME			"E:\\Images"	// 현재는 CommWithVSTCameraDlg.cpp 239라인을
													// 함께 바꿔줘야...

#endif // _DEFINE_H
