#pragma once
#include "afxcmn.h"
#include "afxwin.h"

#include "CmdPacket.h"

#define WM_USER_SET_MY_NEW_PARTNER_CAM	0x402

// CConfigDlg ��ȭ �����Դϴ�.

class CConfigDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CConfigDlg)

public:
	CConfigDlg(CWnd* pParent = NULL);   // ǥ�� �������Դϴ�.
	virtual ~CConfigDlg();

// ��ȭ ���� �������Դϴ�.
	enum { IDD = IDD_DIALOG_CONFIG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

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
