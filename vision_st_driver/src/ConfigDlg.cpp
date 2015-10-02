// ConfigDlg.cpp : 구현 파일입니다.
//

#include "stdafx.h"
#include "CommWithVSTCamera.h"
#include "ConfigDlg.h"
#include "afxdialogex.h"


// CConfigDlg 대화 상자입니다.

IMPLEMENT_DYNAMIC(CConfigDlg, CDialogEx)

CConfigDlg::CConfigDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CConfigDlg::IDD, pParent)
{
	m_nLastSelectedPos = -1;
}

CConfigDlg::~CConfigDlg()
{
}

void CConfigDlg::init(CCmdPacket *CmdProc, CVcUdp *udpForFindingCameras, CConfigInfo *clMyConfigInfo, char *myIpAddr)
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_clCmdProc = CmdProc;
	m_clUdpForFindingCameras = udpForFindingCameras;
	m_clMyConfigInfo = clMyConfigInfo;
	m_szMyIpAddr = myIpAddr;
}

void CConfigDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_ON_LINE_CAM_LIST, m_clistFindResultList);
	DDX_Control(pDX, IDC_NICK_NAME1, m_cEditSetNick1);
	DDX_Control(pDX, IDC_NICK_NAME2, m_cEditSetNick2);
	DDX_Control(pDX, IDC_IP_ADDR, m_cIpAddrSetCamIpAddr);
	DDX_Control(pDX, IDC_SUBNET_MASK, m_cIpAddrSetCamSubnetMask);
	DDX_Control(pDX, IDC_PORT_NO, m_cEditSetPortNo);
	DDX_Control(pDX, IDC_GATEWAY_IP_ADDR, m_cIpAddrSetCamGwIpAddr);
	DDX_Control(pDX, IDC_HOST_IP_ADDR, m_cIpAddrSetHostIpAddr);
}


BEGIN_MESSAGE_MAP(CConfigDlg, CDialogEx)
	ON_BN_CLICKED(IDC_FIND, &CConfigDlg::OnBnClickedFind)
	ON_NOTIFY(LVN_ITEMCHANGED, IDC_ON_LINE_CAM_LIST, &CConfigDlg::OnLvnItemchangedOnLineCamList)
	ON_BN_CLICKED(IDC_CMD_CONFIG, &CConfigDlg::OnBnClickedCmdConfig)
	ON_BN_CLICKED(IDC_BE_MY_PARTNER, &CConfigDlg::OnBnClickedBeMyPartner)
END_MESSAGE_MAP()


// CConfigDlg 메시지 처리기입니다.


void CConfigDlg::OnBnClickedFind()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_clistFindResultList.DeleteAllItems();
	m_nLastSelectedPos = -1;
	DeleteSelectedCamInfo();

	m_clUdpForFindingCameras->m_nNumSavedCamInfo = 0;
	m_clCmdProc->VstCmdFindCam(MULTICAST_IP_ADDR);
	Sleep(1000);
	DisplayFoundCams();
}

void CConfigDlg::DisplayFoundCams()
{
	int n = m_clUdpForFindingCameras->m_nNumSavedCamInfo;
	if(n > MAX_NUM_CAMERA) return;

	CConfigInfo *configInfo;
	for(int i=0; i<n; i++)
	{
		configInfo = &(m_clUdpForFindingCameras->m_clConfigInfo[i]);

		CString sTmp;
		// 일련번호
		sTmp.Format(_T("%02d"), i+1);
		m_clistFindResultList.InsertItem(i, sTmp);

		// NickName
		sTmp.Format(_T("%s %s"), configInfo->m_strCameraNick1, configInfo->m_strCameraNick2);
		m_clistFindResultList.SetItemText(i, 1, sTmp);

		// CAM Mac address
		m_clistFindResultList.SetItemText(i, 2, configInfo->m_bCameraMacAddr);

		// CAM IP address
		m_clistFindResultList.SetItemText(i, 3, configInfo->m_strCameraIpAddr);

		// CAM Subnet mask
		m_clistFindResultList.SetItemText(i, 4, configInfo->m_strCameraSubnetMask);

		// CAM GW IP address
		m_clistFindResultList.SetItemText(i, 5, configInfo->m_strCameraGwAddr);

		// CAM Comm. port
		sTmp.Format(_T("%d"), configInfo->m_nCommPort);
		m_clistFindResultList.SetItemText(i, 6, sTmp);

		// PC IP address
		m_clistFindResultList.SetItemText(i, 7, configInfo->m_strHostIpAddr);
	}
}

BOOL CConfigDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  여기에 추가 초기화 작업을 추가합니다.
	// AfxInitRichEdit();

	InitView();

	return TRUE;  // return TRUE unless you set the focus to a control
	// 예외: OCX 속성 페이지는 FALSE를 반환해야 합니다.
}

void CConfigDlg::InitView()
{
	m_clistFindResultList.SetExtendedStyle(LVS_EX_FULLROWSELECT| LVS_EX_GRIDLINES);
	m_clistFindResultList.SetBkColor(RGB(0xb8,0xd7,0xff));
	m_clistFindResultList.SetTextBkColor(RGB(0xb8,0xd7,0xff));

	m_clistFindResultList.InsertColumn(0, (LPTSTR)"ID", LVCFMT_LEFT, 30);
	m_clistFindResultList.InsertColumn(1, (LPTSTR)"NICK NAME", LVCFMT_LEFT, 100);
	m_clistFindResultList.InsertColumn(2, (LPTSTR)"MAC_ADDR", LVCFMT_LEFT, 110);
	m_clistFindResultList.InsertColumn(3, (LPTSTR)"IP_ADDR", LVCFMT_LEFT, 110);
	m_clistFindResultList.InsertColumn(4, (LPTSTR)"SUBNET_MASK", LVCFMT_LEFT, 110);
	m_clistFindResultList.InsertColumn(5, (LPTSTR)"GW_IP_ADDR", LVCFMT_LEFT, 110);
	m_clistFindResultList.InsertColumn(6, (LPTSTR)"PORT", LVCFMT_LEFT, 45);
	m_clistFindResultList.InsertColumn(7, (LPTSTR)"HOST_IP_ADDR", LVCFMT_LEFT, 110);

	SetDlgItemText(IDC_MY_PARTER_ID, m_clMyConfigInfo->m_bCameraMacAddr);
}


void CConfigDlg::OnLvnItemchangedOnLineCamList(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMLISTVIEW pNMLV = reinterpret_cast<LPNMLISTVIEW>(pNMHDR);
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	if(pNMLV->iItem != m_nLastSelectedPos)
	{
		m_nLastSelectedPos = pNMLV->iItem;

		TRACE("선택된 List Pos = %d\n", pNMLV->iItem);

		DisplaySelectedCamInfo(m_nLastSelectedPos);
	}

	*pResult = 0;
}

void CConfigDlg::DisplaySelectedCamInfo(int pos)
{
	m_cEditSetNick1.SetWindowTextA(m_clUdpForFindingCameras->m_clConfigInfo[pos].m_strCameraNick1);
	m_cEditSetNick2.SetWindowTextA(m_clUdpForFindingCameras->m_clConfigInfo[pos].m_strCameraNick2);
	SetDlgItemText(IDC_MAC_ADDR, m_clUdpForFindingCameras->m_clConfigInfo[pos].m_bCameraMacAddr);
	m_cIpAddrSetCamIpAddr.SetWindowTextA(m_clUdpForFindingCameras->m_clConfigInfo[pos].m_strCameraIpAddr); 
	m_cIpAddrSetCamSubnetMask.SetWindowTextA(m_clUdpForFindingCameras->m_clConfigInfo[pos].m_strCameraSubnetMask);
	CString s; s.Format("%d", m_clUdpForFindingCameras->m_clConfigInfo[pos].m_nCommPort);
	m_cEditSetPortNo.SetWindowTextA(s); 
	m_cIpAddrSetCamGwIpAddr.SetWindowTextA(m_clUdpForFindingCameras->m_clConfigInfo[pos].m_strCameraGwAddr); 
	m_cIpAddrSetHostIpAddr.SetWindowTextA(m_clUdpForFindingCameras->m_clConfigInfo[pos].m_strHostIpAddr);
}

void CConfigDlg::DeleteSelectedCamInfo()
{
	m_cEditSetNick1.SetWindowTextA("");
	m_cEditSetNick2.SetWindowTextA("");
	SetDlgItemText(IDC_MAC_ADDR, "");
	m_cIpAddrSetCamIpAddr.SetWindowTextA(""); 
	m_cIpAddrSetCamSubnetMask.SetWindowTextA("");
	m_cEditSetPortNo.SetWindowTextA(""); 
	m_cIpAddrSetCamGwIpAddr.SetWindowTextA(""); 
	m_cIpAddrSetHostIpAddr.SetWindowTextA("");
}


void CConfigDlg::OnBnClickedCmdConfig()
{
	// TODO: Add your control notification handler code here
	if(m_nLastSelectedPos == -1)
		return;

	m_clCmdProc->VstCmdStop(m_clUdpForFindingCameras->m_clConfigInfo[m_nLastSelectedPos].m_strCameraIpAddr);

	BYTE chkIpA, chkIpB, chkIpC, chkIpD;
	CString a, b, c, d, e, f, g, h, dd;
	m_cEditSetNick1.GetWindowTextA(a);
	m_cEditSetNick2.GetWindowTextA(b);
	GetDlgItemText(IDC_MAC_ADDR, c);
	m_cIpAddrSetCamIpAddr.GetWindowTextA(d);
	m_cIpAddrSetCamIpAddr.GetAddress(chkIpA, chkIpB, chkIpC, chkIpD);
	if((chkIpA != IPv4_S_B1) || (chkIpB != IPv4_S_B2) || (chkIpC != IPv4_S_B3) ||
		(chkIpD == 0) || (chkIpD  == 255))
	{
		CString sTmp;
		sTmp.Format("IP Address는 %d.%d.%d.?(1~254) 이어야 합니다.", 
			IPv4_S_B1, IPv4_S_B2,IPv4_S_B3);
		AfxMessageBox(sTmp);
		return;
	}

	m_cIpAddrSetCamSubnetMask.GetWindowTextA(e);
	m_cIpAddrSetCamGwIpAddr.GetWindowTextA(f); 
	m_cEditSetPortNo.GetWindowTextA(g); int gg = atoi(g);
	m_cIpAddrSetHostIpAddr.GetWindowTextA(h);

	m_clCmdProc->VstCmdSetCamConfigInfo(a, b, c, d, e, f, gg, h, MULTICAST_IP_ADDR);

	Sleep(100);
	OnBnClickedFind();
}


void CConfigDlg::OnBnClickedBeMyPartner()
{
	// TODO: Add your control notification handler code here
	if(m_nLastSelectedPos == -1)
		return;

	if(strcmp(m_szMyIpAddr, 
		m_clUdpForFindingCameras->m_clConfigInfo[m_nLastSelectedPos].m_strHostIpAddr) != 0)
	{
		CString sTmp;
		sTmp.Format("HOST_IP_ADDR이 이 컴퓨터의 IP %s 이어야 \n이 컴퓨터에서 카메라와 연결될 수 있습니다.\nHOST_IP_ADDR을 다시 설정해 주세요", m_szMyIpAddr);
		AfxMessageBox(sTmp);
		return;
	}

	memcpy(m_clMyConfigInfo, &(m_clUdpForFindingCameras->m_clConfigInfo[m_nLastSelectedPos]), 
		sizeof(CConfigInfo));

	GetParent()->PostMessageA(WM_USER_SET_MY_NEW_PARTNER_CAM, 0, 0);

	SetDlgItemText(IDC_MY_PARTER_ID, m_clMyConfigInfo->m_bCameraMacAddr);
	m_nLastSelectedPos = -1;
}
