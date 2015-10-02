#pragma once
#include "afxcmn.h"
#include "afxwin.h"

#include "CmdPacket.h"

#define WM_USER_SET_MY_NEW_PARTNER_CAM	0x402

// CConfigDlg 대화 상자입니다.

class CConfigDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CConfigDlg)

public:
	CConfigDlg(CWnd* pParent = NULL);   // 표준 생성자입니다.
	virtual ~CConfigDlg();

// 대화 상자 데이터입니다.
	enum { IDD = IDD_DIALOG_CONFIG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

	DECLARE_MESSAGE_MAP()

public:
	CListCtrl m_clistFindResultList;
	CEdit m_cEditSetNick1;
	CEdit m_cEditSetNick2;
	CIPAddressCtrl m_cIpAddrSetCamIpAddr;
	CIPAddressCtrl m_cIpAddrSetCamSubnetMask;
	CIPAddressCtrl m_cIpAddrSetCamGwIpAddr;
	CEdit m_cEditSetPortNo;
	CIPAddressCtrl m_cIpAddrSetHostIpAddr;
	CCmdPacket *m_clCmdProc; 
	CVcUdp *m_clUdpForFindingCameras;
	CConfigInfo *m_clMyConfigInfo;
	char *m_szMyIpAddr;

	int m_nLastSelectedPos;

	void init(CCmdPacket *CmdProc, CVcUdp *udpForFindingCameras, 
		CConfigInfo *clMyConfigInfo, char *myIpAddr);
	void InitView();
	void DisplayFoundCams();
	void DisplaySelectedCamInfo(int pos);
	void DeleteSelectedCamInfo();

	virtual BOOL OnInitDialog();
	afx_msg void OnBnClickedFind();
	afx_msg void OnLvnItemchangedOnLineCamList(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnBnClickedCmdConfig();
	afx_msg void OnBnClickedBeMyPartner();
};
