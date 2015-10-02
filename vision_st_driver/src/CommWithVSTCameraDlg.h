
// CommWithVSTCameraDlg.h : ��� ����
//

#pragma once
#include "CmdPacket.h"
#include "ConfigInfo.h"
#include "afxwin.h"
#include "dirent.h"

#include <GdiPlus.h>
using namespace Gdiplus;

#define TIMER_CHECK_PARTNER_CAMERA			0x100
#define TIMER_IMG_DISPLAY					0x101
#define TIMER_DISPLAY_IMG_DATA_COMM_STATUS	0x102
#define TIMER_UDP_DATA_COMM_DISPLAY			0x103
#define msec_IMAGE_DISPLAY_CYCLE			10

// �̹��� ���÷��� ���̾�α׿� �����ϰ� ���
#define WM_USER_SET_MY_NEW_PARTNER_CAM		0x402

// CCommWithVSTCameraDlg ��ȭ ����
class CCommWithVSTCameraDlg : public CDialogEx
{
// �����Դϴ�.
public:
	CCommWithVSTCameraDlg(CWnd* pParent = NULL);	// ǥ�� �������Դϴ�.

	// UDP sockets
	CVcUdp *m_sockUdpForFindingCameras, *m_sockResponseUdp; //sockets as server
	CVcUdp m_sockSendCmdUdp;			// socket as client
	CVcUdp *m_sockUdpForImgData;		// socket as server

	// config ����
	char m_szMyIpAddr[16];			// This PC's IP address
	CConfigInfo m_clMyConfigInfo;	// Config. info of this PC and partner camera
	bool m_bHardwareError;
	bool m_bMyConfigIsReady;		// Flag, displays the config status
	int m_nAlreadyOccupiedCamId;
	int m_nMyCamId;

	// cmdProc ����
	CCmdPacket *m_clCmdProc;
	bool m_bForcedStop;

	// display image����
	byte	*m_bmpImageL, *m_bmpImageR, *m_bmpImageD;
	BITMAPINFO m_szBitmapinfo;
	HWND hwndFrame0, hwndFrame1, hwndFrame2;
	RECT rc0, rc1, rc2;
	HDC hdcFrame0, hdcFrame1, hdcFrame2;
	PAINTSTRUCT ps0, ps1, ps2;

	// etc
	CListBox m_cListMsgLog;
	CString	m_cStrImageFolder, m_cStrCurFileName;
	DIR *dir;
	struct dirent *ent;
	GdiplusStartupInput gdiplusStartupInput;
	ULONG_PTR gdiplusToken;

// ��ȭ ���� �������Դϴ�.
	enum { IDD = IDD_COMMWITHVSTCAMERA_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV �����Դϴ�.


// �����Դϴ�.
protected:
	HICON m_hIcon;

	// ������ �޽��� �� �Լ�
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

public:
	afx_msg void OnBnClickedCmdStop();

	void SetRunEnv();
	void SetPostInit();
	bool GetMyIpAddr();
	bool CreateSocketUDP();
	bool RecreateSocketUDP();
	bool RecreateFindSocketUDP(bool createFlag);
	void ShowAndSaveImage();
	void InitDisplay();
	void DisplayStatus(char *str);
	void DisplayPCInfo();
	void DisplayPartnerCamInfo();
	void MessageDisplayAndLog(BYTE *bData, int nLength, char cSel='X', char cMode = 'A');
	void SetExposure(int pos);
	void SetGlobalGain(int pos);
	CString GetNextImageFileName(void);
	void ReadAImageFileAndDisplay(CString loadFile);
	void ReadAFileInOrderAndDisplay();
	int GetEncoderClsid(const WCHAR* format, CLSID* pClsid);
	void SetGlobalGainAndExposure(int globalGain, int exposure);

	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnBnClickedConfig();
	afx_msg void OnBnClickedRun();
	void IsRun();
	afx_msg void OnBnClickedCmdReset();
	afx_msg void OnBnClickedCmdInit();
	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	CScrollBar m_cScrollBarExposure;
	CScrollBar m_cScrollBarGlobalGain;
	CButton m_cChkAutoSave;
	CButton m_cChkIsJpg;
	CButton m_cChkAutoLoad;
	virtual BOOL OnWndMsg(UINT message, WPARAM wParam, LPARAM lParam, LRESULT* pResult);
	afx_msg void OnBnClickedButtonLoadAFileImage();
	afx_msg void OnBnClickedCheckAutoSave();
	afx_msg void OnBnClickedButtonSaveAnImage();
	afx_msg void OnBnClickedCheckAutoLoad();
	afx_msg void OnBnClickedButtonLoadNext();
	afx_msg void OnBnClickedCheckIsJpg();
};
