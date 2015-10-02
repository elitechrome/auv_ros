
// CommWithVSTCameraDlg.cpp : ���� ����
//

#include "stdafx.h"
#include "CommWithVSTCamera.h"
#include "CommWithVSTCameraDlg.h"
#include "afxdialogex.h"
#include "IniParser.h"
#include "ConfigDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// ���� ���α׷� ������ ���Ǵ� CAboutDlg ��ȭ �����Դϴ�.

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// ��ȭ ���� �������Դϴ�.
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

// �����Դϴ�.
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CCommWithVSTCameraDlg ��ȭ ����
CCommWithVSTCameraDlg::CCommWithVSTCameraDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CCommWithVSTCameraDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);

	m_sockUdpForFindingCameras = m_sockResponseUdp = 
		m_sockUdpForImgData = NULL;
	m_bMyConfigIsReady = false;
	m_bHardwareError = false;

	//default values for UDP Comm.
	m_clMyConfigInfo.m_nCommPort = DEFAULT_CMD_RCV_PORT;
	m_bForcedStop = false;

	hwndFrame0 = NULL;
	m_nAlreadyOccupiedCamId = 0;
	m_nMyCamId = 0;
	dir = NULL;
	m_cStrCurFileName = "";
}

void CCommWithVSTCameraDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_LOG_MSG, m_cListMsgLog);
	DDX_Control(pDX, IDC_SCROLLBAR_EXPOSURE, m_cScrollBarExposure);
	DDX_Control(pDX, IDC_SCROLLBAR_GLOBAL_GAIN, m_cScrollBarGlobalGain);
	DDX_Control(pDX, IDC_CHECK_JPG, m_cChkIsJpg);
	DDX_Control(pDX, IDC_CHECK_AUTO_SAVE, m_cChkAutoSave);
	DDX_Control(pDX, IDC_CHECK_AUTO_LOAD, m_cChkAutoLoad);
}

BEGIN_MESSAGE_MAP(CCommWithVSTCameraDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_CMD_STOP, &CCommWithVSTCameraDlg::OnBnClickedCmdStop)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_CONFIG, &CCommWithVSTCameraDlg::OnBnClickedConfig)
	ON_BN_CLICKED(IDC_RUN, &CCommWithVSTCameraDlg::OnBnClickedRun)
	ON_BN_CLICKED(IDC_CMD_RESET, &CCommWithVSTCameraDlg::OnBnClickedCmdReset)
	ON_BN_CLICKED(IDC_CMD_INIT, &CCommWithVSTCameraDlg::OnBnClickedCmdInit)
	ON_WM_HSCROLL()
	ON_BN_CLICKED(IDC_CHECK_JPG, &CCommWithVSTCameraDlg::OnBnClickedCheckIsJpg)
	ON_BN_CLICKED(IDC_BUTTON_SET_PATH, &CCommWithVSTCameraDlg::OnBnClickedButtonLoadAFileImage)
	ON_BN_CLICKED(IDC_CHECK_AUTO_SAVE, &CCommWithVSTCameraDlg::OnBnClickedCheckAutoSave)
	ON_BN_CLICKED(IDC_BUTTON_SAVE_AN_IMAGE, &CCommWithVSTCameraDlg::OnBnClickedButtonSaveAnImage)
	ON_BN_CLICKED(IDC_CHECK_AUTO_LOAD, &CCommWithVSTCameraDlg::OnBnClickedCheckAutoLoad)
	ON_BN_CLICKED(IDC_BUTTON_LOAD_NEXT, &CCommWithVSTCameraDlg::OnBnClickedButtonLoadNext)
END_MESSAGE_MAP()


// CCommWithVSTCameraDlg �޽��� ó����

BOOL CCommWithVSTCameraDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// �ý��� �޴��� "����..." �޴� �׸��� �߰��մϴ�.

	// IDM_ABOUTBOX�� �ý��� ��� ������ �־�� �մϴ�.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// �� ��ȭ ������ �������� �����մϴ�. ���� ���α׷��� �� â�� ��ȭ ���ڰ� �ƴ� ��쿡��
	//  �����ӿ�ũ�� �� �۾��� �ڵ����� �����մϴ�.
	SetIcon(m_hIcon, TRUE);			// ū �������� �����մϴ�.
	SetIcon(m_hIcon, FALSE);		// ���� �������� �����մϴ�.


	// TODO: ���⿡ �߰� �ʱ�ȭ �۾��� �߰��մϴ�.

	// ����...

	// ���ۼ����� ������� �ʱ�ȭ�ϴ� �͵��� �켱 ó��
	SetRunEnv();

	DisplayStatus("Starting !!!");

	//////////////////////////////////////////////////////////////
	// �ʱ�ȭ ���� : �ڱ�IP ã�� - 
	//				���� ���� �о� �ڱ� ��Ʈ�� Ȯ�� -
	//				�ش� ������ �ٰŷ� UDP socket �����ϱ� - 
	//				Command ó���� �����ϱ� -
	//				�������� �ʱ�ȭ�� ������ ��� �����ϱ� 
	//////////////////////////////////////////////////////////////
	CString sTmp;

	////////////////////// �ڱ� IP ã��
	////// ������ �� CVcUdp class ���� �� �ƹ��ų� �ϳ��� �̿��Ѵ�.
	bool rtn = GetMyIpAddr();
	if(!rtn) // network card�� ��� disable�Ǹ� "127.0.0.1"�� ��ϵȴ�
	{
		sTmp.Format(_T("No Network Card !!!\n ���α׷��� �����ϰ� Network ���� �� �ٽ� �����Ͻʽÿ�."));
		AfxMessageBox(sTmp);
		m_bHardwareError = true;
		
		GetDlgItem(IDC_CMD_INIT)->EnableWindow(false);
		GetDlgItem(IDC_RUN)->EnableWindow(false);
		GetDlgItem(IDC_CMD_STOP)->EnableWindow(false);
		GetDlgItem(IDC_CMD_RESET)->EnableWindow(false);
		GetDlgItem(IDC_CONFIG)->EnableWindow(false);
	}

	////////////////////// �������� �о� �ڱ� ��Ʈ�� Ȯ���ϱ�
	// ���������� csv ���Ϸ� �����ȴ�.
	// csv file�� ������ ���� �ʱ���� �� ���� �۾��� ��ģ ����� �����Ѵ�.
	// �� ���� �� ��ǻ�Ϳ� ��Ʈ���� camera�� ��������� ��� �ִ�.
	// ����Ʈ����� �⺻������ csv file�� ���� ã��
	// �ʱ� �����۾��� �����Ѵ�.

	// config ���� �б�
	m_bMyConfigIsReady = m_clMyConfigInfo.ReadConfigInfo(m_szMyIpAddr);
	if(!m_bMyConfigIsReady)		// CONFIG file ������ ����� �������� �ʾ�����
	{
		sTmp.Format(_T("Configuration Info Err.!"));
		MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());
		sTmp.Format(_T("Default Value set.!"));
		MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());
	}

	DisplayPCInfo();
	DisplayPartnerCamInfo();

	////////////////////// UDP ���� �����ϱ�
	rtn = CreateSocketUDP();

	////////////////////// cmd ó���� ����
	m_clCmdProc = new CCmdPacket(&m_sockSendCmdUdp, m_sockResponseUdp);

	DisplayStatus("Ready !!!");
	
	////////////////////// partner Camera�� �غ�������� Ȯ�� �� �ڵ�����
	if(!rtn)
		return FALSE;	// UDP �ʱ�ȭ�۾��� �����ߴٸ� �Ʒ��� �۾����� ������� �ʴ´�.

	IsRun();	// ���������� ���� partner�� ���ǵ� ī�޶󿡰� Run�������� ���´�.
	Sleep(200); // ��ٷȴٰ�...
	if(m_sockResponseUdp->m_nIsCamStatusRun == STATUS_STOP)  // ���� �޽����� Stop �����̸�
	{
		OnBnClickedCmdInit();
		Sleep(100); // ��ٷȴٰ�...
		SetGlobalGainAndExposure(DEFAULT_GLOBAL_GAIN, DEFAULT_EXPOSURE);
		OnBnClickedRun();
		SetTimer(TIMER_UDP_DATA_COMM_DISPLAY, msec_IMAGE_DISPLAY_CYCLE, NULL);
	}
	else if(m_sockResponseUdp->m_nIsCamStatusRun == STATUS_RUN) // ���� �޽����� ���� ���̶��
	{
		SetGlobalGainAndExposure(DEFAULT_GLOBAL_GAIN, DEFAULT_EXPOSURE);
		OnBnClickedRun();
		SetTimer(TIMER_UDP_DATA_COMM_DISPLAY, msec_IMAGE_DISPLAY_CYCLE, NULL);
	}

	SetPostInit();

	return TRUE;  // ��Ŀ���� ��Ʈ�ѿ� �������� ������ TRUE�� ��ȯ�մϴ�.
}

void  CCommWithVSTCameraDlg::SetRunEnv()
{
	// �̹��� ���۵� ����
	m_bmpImageL = (byte	*)malloc(FRAME_WIDTH * FRAME_HEIGHT * 3);
	m_bmpImageR = (byte	*)malloc(FRAME_WIDTH * FRAME_HEIGHT * 3);
	m_bmpImageD = (byte	*)malloc(FRAME_WIDTH * FRAME_HEIGHT * 3);

	// ȭ�� �ʱ�ȭ
	InitDisplay();

	// �α� ���� ����
	CString cspid, sFolder;
	char szPath[256];
	memset(szPath, 0, sizeof(szPath));
	GetCurrentDirectory(sizeof(szPath), szPath);
	sFolder.Format(_T("%s\\%s"), szPath, LOG_FOLDER_NAME);
	CreateDirectory(sFolder, NULL);

	// �̹��� ���� ����
	m_cStrImageFolder.Format(_T("%s\\%s"), szPath, IMAGE_FOLDER_NAME);
	//m_cStrImageFolder.Format(_T("%s"), IMAGE_FOLDER_NAME);
	CreateDirectory(m_cStrImageFolder, NULL);

	if(dir != NULL) closedir(dir);
	dir = opendir (m_cStrImageFolder.operator const char *());
	if(dir == NULL) {
		CString sTmp;
		sTmp.Format(_T("opendir() Error!"));
		MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());
	}
}

void  CCommWithVSTCameraDlg::SetPostInit()
{
	// �߰��� ī�޶� �� ���ӵǴ� ��� üũ�� ����
	SetTimer(TIMER_CHECK_PARTNER_CAMERA, 500, NULL);

	CString str;
	str.Format("%s: %s", MODIFIED_PROGRAM_NAME, m_szMyIpAddr);
	SetWindowText(str);	// �� �� Ȱ��
}

bool CCommWithVSTCameraDlg::GetMyIpAddr()
{
	CString sTmp;

	memset(m_szMyIpAddr, 0, sizeof(m_szMyIpAddr));
	char *sMyIp;
	sMyIp = m_sockUdpForFindingCameras->GetMyIPAddress();
	if(sMyIp == NULL || strcmp(sMyIp, "127.0.0.1") == 0)
	{
		return FALSE ;
	}

	memcpy(m_szMyIpAddr, sMyIp, strlen(sMyIp));
	sTmp.Format(_T("MY Computer IP : %s"), m_szMyIpAddr);
	MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());
	return TRUE;
}

bool CCommWithVSTCameraDlg::CreateSocketUDP()
{
	bool rtn = false;

	// Create 'Command Receiving' socket as Server
	if(m_sockResponseUdp == NULL)
	{
		m_sockResponseUdp = (CVcUdp *)AfxBeginThread(RUNTIME_CLASS(CVcUdp),
			THREAD_PRIORITY_LOWEST, 0, CREATE_SUSPENDED, NULL); //������ ����
		if(m_sockResponseUdp == NULL)
		{
			TRACE("Creating Cmd-Response Socket Rcv-thread Fail\n");
			return FALSE;
		}

		rtn = m_sockResponseUdp->InitUdpSock(m_clMyConfigInfo.m_nCommPort, 
			BYTES_RESPONSE_PACKET, true);
		if(rtn == false) return FALSE;

		m_sockResponseUdp->ResumeThread();
	}

	// Create Command Sending socket as Client
	rtn = m_sockSendCmdUdp.InitUdpSock(CMD_SEND_PORT, NULL, false);
	if(rtn == false) return FALSE;

	// Create Image Data Receiving socket as Server
	if(m_sockUdpForImgData == NULL)
	{
		m_sockUdpForImgData = (CVcUdp *)AfxBeginThread(RUNTIME_CLASS(CVcUdp),
			THREAD_PRIORITY_HIGHEST, 0, CREATE_SUSPENDED, NULL); //������ ����
		if(m_sockUdpForImgData == NULL)
		{
			TRACE("Creating Image-Data Socket Rcv-thread Fail\n");
			return FALSE;
		}
		rtn = m_sockUdpForImgData->InitUdpSock(m_clMyConfigInfo.m_nCommPort+1, 
			BYTES_IMAGE_DATA_PACKET, true);
		if(rtn == false) return FALSE;

		m_sockUdpForImgData->ResumeThread();
	}

	return TRUE;
}

// cmd socket�� data socket�� port��ȣ�� ����Ǿ��� ��츦 ����Ͽ� �ٽ� ����
bool CCommWithVSTCameraDlg::RecreateSocketUDP()
{
	bool rtn = false;

	// delete
	if(m_sockResponseUdp != NULL)
	{
		m_sockResponseUdp->ExitInstance();
		delete m_sockResponseUdp;
		m_sockResponseUdp = NULL;
	}

	if(m_sockUdpForImgData != NULL)
	{
		m_sockUdpForImgData->ExitInstance();
		delete m_sockUdpForImgData;
		m_sockUdpForImgData = NULL;
	}

	// Create 'Command Receiving' socket as Server
	if(m_sockResponseUdp == NULL)
	{
		m_sockResponseUdp = (CVcUdp *)AfxBeginThread(RUNTIME_CLASS(CVcUdp),
			THREAD_PRIORITY_LOWEST, 0, CREATE_SUSPENDED, NULL); //������ ����
		if(m_sockResponseUdp == NULL)
		{
			TRACE("Creating Cmd-Response Socket Rcv-thread Fail\n");
			return FALSE;
		}

		rtn = m_sockResponseUdp->InitUdpSock(m_clMyConfigInfo.m_nCommPort, 
			BYTES_RESPONSE_PACKET, true);
		if(rtn == false) return FALSE;

		m_sockResponseUdp->ResumeThread();
	}

	// Create Image Data Receiving socket as Server
	if(m_sockUdpForImgData == NULL)
	{
		m_sockUdpForImgData = (CVcUdp *)AfxBeginThread(RUNTIME_CLASS(CVcUdp),
			THREAD_PRIORITY_HIGHEST, 0, CREATE_SUSPENDED, NULL); //������ ����
		if(m_sockUdpForImgData == NULL)
		{
			TRACE("Creating Image-Data Socket Rcv-thread Fail\n");
			return FALSE;
		}

		rtn = m_sockUdpForImgData->InitUdpSock(m_clMyConfigInfo.m_nCommPort+1, 
			BYTES_IMAGE_DATA_PACKET, true);
		if(rtn == false) return FALSE;

		m_sockUdpForImgData->ResumeThread();
	}

	// ���� ������ udp class�� �ٽ� ����
	m_clCmdProc->ReInit(m_sockResponseUdp);

	return TRUE;
}

// �� ��ǻ�Ϳ� �ΰ��� camera�� �����Ͽ� ����� ���
// Find socket(���� port 1030)�� �ΰ��� manager ���α׷� ���̿���
// ���ÿ� ����� �� ���� �ʿ��� ���� �����Ͽ� ��� ��
// delete �ϱ� ���� ������ ������.
bool CCommWithVSTCameraDlg::RecreateFindSocketUDP(bool createFlag)
{
	if(!createFlag)
	{
		if(m_sockUdpForFindingCameras != NULL)
		{
			m_sockUdpForFindingCameras->ExitInstance();
			delete m_sockUdpForFindingCameras;
			m_sockUdpForFindingCameras = NULL;
		}
	}
	else 
	{
		// Create 'Finding Camera command's results' socket as Server
		if(m_sockUdpForFindingCameras == NULL)
		{
			m_sockUdpForFindingCameras = (CVcUdp *)AfxBeginThread(RUNTIME_CLASS(CVcUdp),
				THREAD_PRIORITY_LOWEST, 0, CREATE_SUSPENDED, NULL); //������ ����
			if(m_sockUdpForFindingCameras == NULL)
			{
				TRACE("Creating Find-Result Socket Rcv-thread Fail\n");
				return FALSE;
			}
			int rtn = m_sockUdpForFindingCameras->InitUdpSock(FIND_CAMERA_RESULT_RCV_PORT, 
				BYTES_RESPONSE_PACKET, true);
			if(rtn == false) return FALSE;

			m_sockUdpForFindingCameras->ResumeThread();
		}
	}
	return TRUE;
}

void CCommWithVSTCameraDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// ��ȭ ���ڿ� �ּ�ȭ ���߸� �߰��� ��� �������� �׸�����
//  �Ʒ� �ڵ尡 �ʿ��մϴ�. ����/�� ���� ����ϴ� MFC ���� ���α׷��� ��쿡��
//  �����ӿ�ũ���� �� �۾��� �ڵ����� �����մϴ�.

void CCommWithVSTCameraDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // �׸��⸦ ���� ����̽� ���ؽ�Ʈ�Դϴ�.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Ŭ���̾�Ʈ �簢������ �������� ����� ����ϴ�.
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// �������� �׸��ϴ�.
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}


// ����ڰ� �ּ�ȭ�� â�� ���� ���ȿ� Ŀ���� ǥ�õǵ��� �ý��ۿ���
//  �� �Լ��� ȣ���մϴ�.
HCURSOR CCommWithVSTCameraDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void CCommWithVSTCameraDlg::OnTimer(UINT_PTR nIDEvent)
{
	static int lastGoodFrameCount = 0;
	bool rtn = false;

	if(nIDEvent == TIMER_IMG_DISPLAY)
	{
		// ���������� ������ �о� ȭ�鿡 ǥ��
		if(m_cChkAutoLoad.GetCheck())
			ReadAFileInOrderAndDisplay();
		// ī�޶�� ���� ���� �ǽð� ���� ǥ��
		else if(!m_sockUdpForImgData->m_bIsSignificantFault)
			ShowAndSaveImage();
	}
	else if(nIDEvent == TIMER_DISPLAY_IMG_DATA_COMM_STATUS)
	{
		static int lastMinutes = 0;

		// Frame rate, ��� ����/���� ȸ�� ǥ�� �� �α�
		CTime ct = CTime::GetCurrentTime();
		int minutes = ct.GetMinute();
		if(minutes != lastMinutes)
		{
			char msg[256];
			memset(msg, 0x00, sizeof(msg));
			sprintf(msg, "FrameRate: %.2f, Success: %d, Fail: %d", m_sockUdpForImgData->m_fFrameRate, 
				m_sockUdpForImgData->m_ulGoodFrameCount, m_sockUdpForImgData->m_ulBadFrameCount); 

			lastMinutes = minutes;
			CString sTmp;
			sTmp.Format("[MAC: %s] %s", m_clMyConfigInfo.m_bCameraMacAddr, msg);
			MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());
		}
	}
	else if(nIDEvent == TIMER_UDP_DATA_COMM_DISPLAY)
	{
		if(m_sockResponseUdp->m_bDidRx)
		{
			//MessageDisplayAndLog((BYTE*)m_sockResponseUdp->m_szRcvBuff, BYTES_RESPONSE_PACKET, 'S', 'H');
			m_sockResponseUdp->m_bDidRx = FALSE;		// ���Ȯ�� �ߴٴ� ��. �׳� ����Ȯ�ο�
		}
	}
	else if(nIDEvent == TIMER_CHECK_PARTNER_CAMERA)  // ī�޶� ����üũ �� �ڵ� �����
	{
		// Image data�� ���۵ǰ� ���� �ʴٸ�
		if(m_sockUdpForImgData->m_ulGoodFrameCount == lastGoodFrameCount)
		{
			if(m_bForcedStop) return;
			IsRun();
			Sleep(100); // ��ٷȴٰ�...
			if(m_sockResponseUdp->m_nIsCamStatusRun == STATUS_STOP)  // ���� �޽����� Stop �����̸�
			{
				OnBnClickedCmdInit();
				SetGlobalGainAndExposure(m_cScrollBarGlobalGain.GetScrollPos(), 
					m_cScrollBarExposure.GetScrollPos());
				OnBnClickedRun();
				SetTimer(TIMER_UDP_DATA_COMM_DISPLAY, msec_IMAGE_DISPLAY_CYCLE, NULL);
			}
			else if(m_sockResponseUdp->m_nIsCamStatusRun == STATUS_RUN) // ���� �޽����� ���� ���̶��
			{
				SetGlobalGainAndExposure(m_cScrollBarGlobalGain.GetScrollPos(), 
					m_cScrollBarExposure.GetScrollPos());
				OnBnClickedRun();
				SetTimer(TIMER_UDP_DATA_COMM_DISPLAY, msec_IMAGE_DISPLAY_CYCLE, NULL);
			}
		}
		else
		{
			lastGoodFrameCount = m_sockUdpForImgData->m_ulGoodFrameCount;
		}
	}
	CDialogEx::OnTimer(nIDEvent);
}


void CCommWithVSTCameraDlg::OnBnClickedConfig()
{
	CString sTmp;
	if(m_bHardwareError)
	{
		//sTmp.Format(_T("No Network Card !!!\n ���α׷��� �����ϰ� Network ���� �� �ٽ� �����Ͻʽÿ�."));
		//AfxMessageBox(sTmp);
		return;
	}
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.

	int rtn = RecreateFindSocketUDP(TRUE);
	if(rtn == FALSE)
	{
		sTmp.Format(_T("�̹� �ٸ� ���α׷����� ������Ʈ�� ������Դϴ�."));
		AfxMessageBox(sTmp);
		return;
	}

	CConfigDlg dlg;
	dlg.init(m_clCmdProc, m_sockUdpForFindingCameras, &m_clMyConfigInfo, m_szMyIpAddr);

	dlg.DoModal();

	// delete
	RecreateFindSocketUDP(FALSE);
}

void CCommWithVSTCameraDlg::OnBnClickedCmdInit()
{
	CString sTmp;
	m_bForcedStop = true;

	if(m_bHardwareError)
	{
		//sTmp.Format(_T("No Network Card !!!\n ���α׷��� �����ϰ� Network ���� �� �ٽ� �����Ͻʽÿ�."));
		//AfxMessageBox(sTmp);
		return;
	}
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	int rtn = m_clCmdProc->VstCmdInitialize(m_clMyConfigInfo.m_strCameraIpAddr);
	if(rtn == CMD_ERROR)
	{
		sTmp.Format(_T("Init Command Error!"));
		MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());
	}
	else
	{
		sTmp.Format(_T("Init Command OK!"));
		MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());
	}

	m_sockResponseUdp->m_nIsCamStatusRun = STATUS_NO_RESPONSE;
	IsRun();

	// ǥ��ȭ�� ����
	KillTimer(TIMER_IMG_DISPLAY); 
	KillTimer(TIMER_DISPLAY_IMG_DATA_COMM_STATUS);
}

void CCommWithVSTCameraDlg::OnBnClickedRun()
{
	CString sTmp;
	m_bForcedStop = false;

	if(m_bHardwareError)
	{
		//sTmp.Format(_T("No Network Card !!!\n ���α׷��� �����ϰ� Network ���� �� �ٽ� �����Ͻʽÿ�."));
		//AfxMessageBox(sTmp);
		return;
	}

	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	int rtn = m_clCmdProc->VstCmdStart(m_clMyConfigInfo.m_strCameraIpAddr);
	if(rtn == CMD_ERROR)
	{
		sTmp.Format(_T("Run Command Error!"));
		MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());
	}		
	else
	{
		sTmp.Format(_T("Run Command OK!"));
		MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());
	}

	m_sockResponseUdp->m_nIsCamStatusRun = STATUS_NO_RESPONSE;
	IsRun();

	// ǥ�� ȭ�� ����
	SetTimer(TIMER_IMG_DISPLAY, msec_IMAGE_DISPLAY_CYCLE, NULL); 
	SetTimer(TIMER_DISPLAY_IMG_DATA_COMM_STATUS, 1000, NULL); 
}

void CCommWithVSTCameraDlg::IsRun()
{
	m_clCmdProc->VstCmdGetStatus_IsStarted(m_clMyConfigInfo.m_strCameraIpAddr);
}

void CCommWithVSTCameraDlg::OnBnClickedCmdStop()
{
	CString sTmp;
	m_bForcedStop = true;

	if(m_bHardwareError)
	{
		//sTmp.Format(_T("No Network Card !!!\n ���α׷��� �����ϰ� Network ���� �� �ٽ� �����Ͻʽÿ�."));
		//AfxMessageBox(sTmp);
		return;
	}

	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	int rtn = m_clCmdProc->VstCmdStop(m_clMyConfigInfo.m_strCameraIpAddr);
	if(rtn == CMD_ERROR)
	{
		sTmp.Format(_T("Stop Command Error!"));
		MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());
	}		
	else
	{
		sTmp.Format(_T("Stop Command OK!"));
		MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());
	}

	m_sockResponseUdp->m_nIsCamStatusRun = STATUS_NO_RESPONSE;
	IsRun();

	// ǥ��ȭ�� ����
	KillTimer(TIMER_IMG_DISPLAY); 
	KillTimer(TIMER_DISPLAY_IMG_DATA_COMM_STATUS);
}

void CCommWithVSTCameraDlg::OnBnClickedCmdReset()
{
	CString sTmp;
	m_bForcedStop = false;

	if(m_bHardwareError)
	{
		//sTmp.Format(_T("No Network Card !!!\n ���α׷��� �����ϰ� Network ���� �� �ٽ� �����Ͻʽÿ�."));
		//AfxMessageBox(sTmp);
		return;
	}

	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	int rtn = m_clCmdProc->VstCmdReset(m_clMyConfigInfo.m_strCameraIpAddr);
	if(rtn == CMD_ERROR)
	{
		sTmp.Format(_T("Reset Command Error!"));
		MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());
	}		
	else
	{
		sTmp.Format(_T("Reset Command OK!"));
		MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());
	}

	Sleep(500);
	m_sockResponseUdp->m_nIsCamStatusRun = STATUS_NO_RESPONSE;
	IsRun();

	// ǥ��ȭ�� ����
	KillTimer(TIMER_IMG_DISPLAY); 
	KillTimer(TIMER_DISPLAY_IMG_DATA_COMM_STATUS);
}

void CCommWithVSTCameraDlg::InitDisplay()
{
	// ����ȭ�� ũ�� 
	this->SetWindowPos(NULL, 0, 0, 1280+22, 720+40, SWP_NOMOVE);

	// ȭ��ǥ�� ��Ʈ�ѵ� ǥ����ġ ������
	GetDlgItem(IDC_STATIC_IMAGE1        )->SetWindowPos(&CWnd::wndTop,   0,   0, 640, 360, SWP_FRAMECHANGED);
	GetDlgItem(IDC_STATIC_IMAGE2        )->SetWindowPos(&CWnd::wndTop, 645,   0, 640, 360, SWP_FRAMECHANGED);
	GetDlgItem(IDC_STATIC_IMAGE3        )->SetWindowPos(&CWnd::wndTop,   0, 365, 640, 360, SWP_FRAMECHANGED);
	GetDlgItem(IDC_LOG_MSG              )->SetWindowPos(&CWnd::wndTop, 645, 365, 640, 210, SWP_FRAMECHANGED);

	GetDlgItem(IDC_CMD_INIT             )->SetWindowPos(&CWnd::wndTop, 650, 580,   0,   0, SWP_NOSIZE);
	GetDlgItem(IDC_RUN                  )->SetWindowPos(&CWnd::wndTop, 700, 580,   0,   0, SWP_NOSIZE);
	GetDlgItem(IDC_CMD_STOP             )->SetWindowPos(&CWnd::wndTop, 770, 580,   0,   0, SWP_NOSIZE);
	GetDlgItem(IDC_CMD_RESET            )->SetWindowPos(&CWnd::wndTop, 820, 580,   0,   0, SWP_NOSIZE);
	GetDlgItem(IDC_CONFIG               )->SetWindowPos(&CWnd::wndTop, 890, 580,   0,   0, SWP_NOSIZE);

	GetDlgItem(IDC_STATIC_GLOBAL_GAIN   )->SetWindowPos(&CWnd::wndTop, 650, 630,   0,   0, SWP_NOSIZE);
	GetDlgItem(IDC_SCROLLBAR_GLOBAL_GAIN)->SetWindowPos(&CWnd::wndTop, 760, 630,   0,   0, SWP_NOSIZE);
	GetDlgItem(IDC_STATIC_EXPOSURE      )->SetWindowPos(&CWnd::wndTop, 650, 660,   0,   0, SWP_NOSIZE);
	GetDlgItem(IDC_SCROLLBAR_EXPOSURE   )->SetWindowPos(&CWnd::wndTop, 760, 660,   0,   0, SWP_NOSIZE);
	GetDlgItem(IDCANCEL                 )->SetWindowPos(&CWnd::wndTop,1210, 690,   0,   0, SWP_NOSIZE);

	GetDlgItem(IDC_CHECK_JPG            )->SetWindowPos(&CWnd::wndTop, 970, 590,   0,   0, SWP_NOSIZE);
	GetDlgItem(IDC_BUTTON_SAVE_AN_IMAGE )->SetWindowPos(&CWnd::wndTop,1035, 590,   0,   0, SWP_NOSIZE);
	GetDlgItem(IDC_CHECK_AUTO_SAVE      )->SetWindowPos(&CWnd::wndTop,1085, 590,   0,   0, SWP_NOSIZE);
	GetDlgItem(IDC_STATIC_LOAD          )->SetWindowPos(&CWnd::wndTop, 970, 630,   0,   0, SWP_NOSIZE);
	GetDlgItem(IDC_BUTTON_SET_PATH      )->SetWindowPos(&CWnd::wndTop,1035, 627,   0,   0, SWP_NOSIZE);
	GetDlgItem(IDC_CHECK_AUTO_LOAD      )->SetWindowPos(&CWnd::wndTop,1150, 630,   0,   0, SWP_NOSIZE);
	GetDlgItem(IDC_EDIT_LOAD_IMAGE_PATH )->SetWindowPos(&CWnd::wndTop, 950, 650,   0,   0, SWP_NOSIZE);
	GetDlgItem(IDC_BUTTON_LOAD_NEXT     )->SetWindowPos(&CWnd::wndTop,1240, 650,   0,   0, SWP_NOSIZE);
	
	m_cChkIsJpg.SetCheck(1);			// JPG file��� operation

	// Set Bitmap Header
	m_szBitmapinfo.bmiHeader.biSize=sizeof(BITMAPINFOHEADER);
	m_szBitmapinfo.bmiHeader.biWidth=FRAME_WIDTH;
	m_szBitmapinfo.bmiHeader.biHeight=FRAME_HEIGHT;
	m_szBitmapinfo.bmiHeader.biPlanes=1;  
	m_szBitmapinfo.bmiHeader.biBitCount=24;	
	m_szBitmapinfo.bmiHeader.biCompression=BI_RGB;	  
	m_szBitmapinfo.bmiHeader.biSizeImage=FRAME_WIDTH*FRAME_HEIGHT*3;
	m_szBitmapinfo.bmiHeader.biXPelsPerMeter=0;
	m_szBitmapinfo.bmiHeader.biYPelsPerMeter=0;
	m_szBitmapinfo.bmiHeader.biClrUsed=0;
	m_szBitmapinfo.bmiHeader.biClrImportant=0;

	// ���÷��� �ڵ�� �ʱ�ȭ
	GetDlgItem( IDC_STATIC_IMAGE1, &hwndFrame0 );
	hdcFrame0 = ::GetDC( hwndFrame0 );
	::GetWindowRect( hwndFrame0, &rc0 );
	::BeginPaint(hwndFrame0, &ps0);
	::SetStretchBltMode(hdcFrame0, COLORONCOLOR);

	GetDlgItem( IDC_STATIC_IMAGE2, &hwndFrame1 );
	hdcFrame1 = ::GetDC( hwndFrame1 );
	::GetWindowRect( hwndFrame1, &rc1 );
	::BeginPaint(hwndFrame1, &ps1);
	::SetStretchBltMode(hdcFrame1, COLORONCOLOR);

	GetDlgItem( IDC_STATIC_IMAGE3, &hwndFrame2 );
	hdcFrame2 = ::GetDC( hwndFrame2 );
	::GetWindowRect( hwndFrame2, &rc2 );
	::BeginPaint(hwndFrame2, &ps2);
	::SetStretchBltMode(hdcFrame2, COLORONCOLOR);

	// ȭ�� �Ʒ� Exposure�� Global Gain ���� ��Ʈ�� �ʱ�ȭ
	m_cScrollBarGlobalGain.SetScrollRange(1, 255);
	m_cScrollBarExposure.SetScrollRange(1, 1920);
}

void CCommWithVSTCameraDlg::SetGlobalGainAndExposure(int globalGain, int exposure)
{
	SetGlobalGain(globalGain);
	SetExposure(exposure);
}

void CCommWithVSTCameraDlg::ShowAndSaveImage()
{
	static unsigned long lastCount = 0;

	if(!m_sockUdpForImgData->m_bDidRx)
		return;

	// Raw data ������ ��� ȭ�� ���÷��� ����.
	if(!(m_cChkAutoSave.GetCheck() && !m_cChkIsJpg.GetCheck()))
	{
		// raw format file���� ���� ������ ó���� ������ ������ ī��Ʈ ������ �����Ƿ�
		// ���� �̹��� �ߺ� ǥ�� üũ�� ���⼭ ���� �ʰ� �׳� �Ѹ���.

		unsigned char LR, LG, LB, LD;
		unsigned char RR, RG, RB, RD;
		byte *videoBuf = m_sockUdpForImgData->m_ucpImgCopy;

		for(int y=0; y<FRAME_HEIGHT; y++){
		for(int x=0; x<FRAME_WIDTH; x++){

			//L
			LG = videoBuf[(y*FRAME_WIDTH + x) * BYTES_1_PIXEL + 0];
			LB = videoBuf[(y*FRAME_WIDTH + x) * BYTES_1_PIXEL + 1];
			LR = videoBuf[(y*FRAME_WIDTH + x) * BYTES_1_PIXEL + 5];

			//R
			RG = videoBuf[(y*FRAME_WIDTH + x) * BYTES_1_PIXEL + 2];
			RB = videoBuf[(y*FRAME_WIDTH + x) * BYTES_1_PIXEL + 3];
			RR = videoBuf[(y*FRAME_WIDTH + x) * BYTES_1_PIXEL + 7];

			//Disparity Map
			LD = videoBuf[(y*FRAME_WIDTH + x) * BYTES_1_PIXEL + 4];

			//L color
			m_bmpImageL[3*(FRAME_WIDTH*(FRAME_HEIGHT-y-1) + x) + 0] = LB;
			m_bmpImageL[3*(FRAME_WIDTH*(FRAME_HEIGHT-y-1) + x) + 1] = LG;
			m_bmpImageL[3*(FRAME_WIDTH*(FRAME_HEIGHT-y-1) + x) + 2] = LR;

			//R color, H flip
			m_bmpImageR[3*(FRAME_WIDTH*(FRAME_HEIGHT-y-1) +(FRAME_WIDTH-1-x)) + 0] = RB;
			m_bmpImageR[3*(FRAME_WIDTH*(FRAME_HEIGHT-y-1) +(FRAME_WIDTH-1-x)) + 1] = RG;
			m_bmpImageR[3*(FRAME_WIDTH*(FRAME_HEIGHT-y-1) +(FRAME_WIDTH-1-x)) + 2] = RR;

			//DISP
			m_bmpImageD[3*(FRAME_WIDTH*(FRAME_HEIGHT-y-1) + x) + 0] = LD;
			m_bmpImageD[3*(FRAME_WIDTH*(FRAME_HEIGHT-y-1) + x) + 1] = LD;
			m_bmpImageD[3*(FRAME_WIDTH*(FRAME_HEIGHT-y-1) + x) + 2] = LD;
		}
		}

		::StretchDIBits(hdcFrame0, 0, 0,
			//lFrame1Width, lFrame1Height, 						
			FRAME_WIDTH, FRAME_HEIGHT,
			0, 0, FRAME_WIDTH, FRAME_HEIGHT, 
			m_bmpImageL,
			&m_szBitmapinfo, 
			DIB_RGB_COLORS,
			SRCCOPY);

		::StretchDIBits(hdcFrame1, 0, 0,
			//lFrame1Width, lFrame1Height, 						
			FRAME_WIDTH, FRAME_HEIGHT,
			0, 0, FRAME_WIDTH, FRAME_HEIGHT, 
			m_bmpImageR,
			&m_szBitmapinfo, 
			DIB_RGB_COLORS,
			SRCCOPY);

		::StretchDIBits(hdcFrame2, 0, 0,
			//lFrame1Width, lFrame1Height, 						
			FRAME_WIDTH, FRAME_HEIGHT,
			0, 0, FRAME_WIDTH, FRAME_HEIGHT, 
			m_bmpImageD,
			&m_szBitmapinfo, 
			DIB_RGB_COLORS,
			SRCCOPY);
	}

	if(m_cChkAutoSave.GetCheck())
	{
		if(lastCount == m_sockUdpForImgData->m_ulGoodFrameCount)
			return;
		lastCount = m_sockUdpForImgData->m_ulGoodFrameCount;

		OnBnClickedButtonSaveAnImage();
	}

	// ȭ�� ǥ�ð� �������Ƿ� data�� copy�� �� �ְ� �Ѵ�.
	m_sockUdpForImgData->m_bDidRx = false;
}

void CCommWithVSTCameraDlg::DisplayStatus(char *str)
{
	// SetDlgItemText(IDC_CAMERA_STATUS_MSG, str);
}

void CCommWithVSTCameraDlg::DisplayPCInfo()
{
	//SetDlgItemText(IDC_STATIC_PC_IP_ADDR, m_szMyIpAddr);
	//CString s;
	//s.Format("%d", m_clMyConfigInfo.m_nCommPort);
	//SetDlgItemText(IDC_STATIC_PC_PORT_NO, s);
}

void CCommWithVSTCameraDlg::DisplayPartnerCamInfo()
{
	//CString s;
	//s.Format("CAMERA : %s, %s", m_clMyConfigInfo.m_strCameraNick1, m_clMyConfigInfo.m_strCameraNick2);
	//SetDlgItemText(IDC_STATIC_CAM_NICKNAME, s);

	//SetDlgItemText(IDC_STATIC_CAM_IP_ADDR, m_clMyConfigInfo.m_strCameraIpAddr);
	//SetDlgItemText(IDC_STATIC_CAM_SUBNET_MASK, m_clMyConfigInfo.m_strCameraSubnetMask);
	//s.Format("%d", m_clMyConfigInfo.m_nCommPort);
	//SetDlgItemText(IDC_STATIC_CAM_PORT_NO, s);
	//SetDlgItemText(IDC_STATIC_CAM_MAC_ADDR, m_clMyConfigInfo.m_bCameraMacAddr);
	//SetDlgItemText(IDC_STATIC_CAM_HOST_IP_ADDR, m_clMyConfigInfo.m_strHostIpAddr);
}

char hbuff[1024];
void CCommWithVSTCameraDlg::MessageDisplayAndLog(BYTE *bData, int nLength, char cSel, char cMode)
{
	CString sTmp, sTot = "";
	
	if(cMode == 'H'){
		sTot.Format(_T("<x%02X>"), bData[1]);

		for(int i=0;i<nLength;i++){
			sTmp.Format(_T(" %02X"), bData[i]);
			sTot += sTmp;
		}
	}
	else{
		sTot.Format("%s", bData);
	}

	CTime ct = CTime::GetCurrentTime();
	if(cSel == 'R'){
		sTot = ct.Format("[%Y-%m-%d %H:%M:%S] ") + " [��û] " + sTot;
	}
	else if(cSel == 'S'){
		sTot = ct.Format("[%Y-%m-%d %H:%M:%S] ") + " [����] " + sTot;
	}
	else{
		sTot = ct.Format("[%Y-%m-%d %H:%M:%S] ") + sTot;
	}

	if(m_cListMsgLog.GetCount() > 200){
		m_cListMsgLog.ResetContent();
	}
	
	int nTemp1 = 90;
	int nPos, i;
	int nSum = sTot.GetLength() / nTemp1;
	int nGap = sTot.GetLength() % nTemp1;
	
	CString stt;
	for(i=0;i<nSum;i++){
		stt = sTot.Mid(i*nTemp1, nTemp1);
		nPos = m_cListMsgLog.AddString(stt);
	}
	if(nGap >0){
		stt = sTot.Mid(i*nTemp1, nGap);
		nPos = m_cListMsgLog.AddString(stt);
	}
	
	m_cListMsgLog.SetCurSel(nPos);

	CString m_strFileName;
	FILE	*fp;

	char szPath[256];

	CString sDay = ct.Format("%Y-%m-%d %H");
	memset(szPath, 0x00, sizeof(szPath));
	GetCurrentDirectory(sizeof(szPath), szPath);

	m_strFileName.Empty();
	m_strFileName.Format(_T("%s\\%s\\%s%s"), szPath, LOG_FOLDER_NAME, sDay, ".log.txt");

	fp = fopen((LPSTR)(LPCTSTR)m_strFileName, "a");
	if(fp != NULL){
		memset(hbuff, 0x00, sizeof(hbuff));
		memcpy(hbuff , (LPSTR)(LPCTSTR)sTot, sTot.GetLength());

		fprintf(fp, "%s\n", hbuff);
		fclose(fp);
		::SetCurrentDirectory(szPath);
	}
}

void CCommWithVSTCameraDlg::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
	// TODO: Add your message handler code here and/or call default
	int pos=pScrollBar->GetScrollPos();
	int min=0; 
	int max=0;
	pScrollBar->GetScrollRange(&min, &max);
	switch(nSBCode){
			case SB_LINEDOWN: 
				pos += 1;
				if(pos > max) pos = max;
				pScrollBar->SetScrollPos(pos);
				break;
			case SB_LINEUP:
				pos -= 1;
				if(pos < min) pos = min;
				pScrollBar->SetScrollPos(pos);				
				break;
			case SB_PAGEDOWN: 
				pos += 10;
				if(pos > max) pos = max;
				pScrollBar->SetScrollPos(pos);
				break;
			case SB_PAGEUP: 
				pos -= 10;
				if(pos < min) pos = min;
				pScrollBar->SetScrollPos(pos);
				break;
			case SB_THUMBTRACK: {
					SCROLLINFO si = {0};
					si.cbSize = sizeof(SCROLLINFO);
					si.fMask = SIF_TRACKPOS;
					::GetScrollInfo(pScrollBar->m_hWnd, SB_CTL, &si);
					pos=si.nTrackPos;
					pScrollBar->SetScrollPos(pos);						
				}
				break;
			default: 
				break;
	}	
	if(pScrollBar->GetDlgCtrlID() == IDC_SCROLLBAR_EXPOSURE) SetExposure(pos);
	else if(pScrollBar->GetDlgCtrlID() == IDC_SCROLLBAR_GLOBAL_GAIN) SetGlobalGain(pos);

	CDialogEx::OnHScroll(nSBCode, nPos, pScrollBar);
}

//////////////////////////////////////////////////
void CCommWithVSTCameraDlg::SetExposure(int pos)
{	
	CString sTmp;
	if(m_bHardwareError)
	{
		//sTmp.Format(_T("No Network Card !!!\n ���α׷��� �����ϰ� Network ���� �� �ٽ� �����Ͻʽÿ�."));
		//AfxMessageBox(sTmp);
		return;
	}

	CString str;
	str.Format("Exposure: %d", pos);
	SetDlgItemText(IDC_STATIC_EXPOSURE, str);
	m_cScrollBarExposure.SetScrollPos(pos);
	m_clCmdProc->VstCmdSetExposure(pos, m_clMyConfigInfo.m_strCameraIpAddr);
	return;
}
//////////////////////////////////////////////////
void CCommWithVSTCameraDlg::SetGlobalGain(int pos)
{	
	CString sTmp;
	if(m_bHardwareError)
	{
		//sTmp.Format(_T("No Network Card !!!\n ���α׷��� �����ϰ� Network ���� �� �ٽ� �����Ͻʽÿ�."));
		//AfxMessageBox(sTmp);
		return;
	}

	CString str;
	str.Format("Global gain: %d", pos);
	SetDlgItemText(IDC_STATIC_GLOBAL_GAIN, str);	
	m_cScrollBarGlobalGain.SetScrollPos(pos);
	m_clCmdProc->VstCmdSetGlobalGain(pos, m_clMyConfigInfo.m_strCameraIpAddr);
	return;
}

BOOL CCommWithVSTCameraDlg::OnWndMsg(UINT message, WPARAM wParam, LPARAM lParam, LRESULT* pResult)
{
	// TODO: Add your specialized code here and/or call the base class
	// ImageDisplay dialog ������ �����Ѵ�.

	// Config Dialog ���� partner ī�޶� ����Ǿ����� �˷��� ������
	// ���õ� Class �� �������� �ٽ� �����Ѵ�.
	if(message == WM_USER_SET_MY_NEW_PARTNER_CAM)
	{
		DisplayPartnerCamInfo();

		KillTimer(TIMER_IMG_DISPLAY);
		KillTimer(TIMER_DISPLAY_IMG_DATA_COMM_STATUS);
		KillTimer(TIMER_UDP_DATA_COMM_DISPLAY);

		// socket�� �ٽ� �����Ѵ�.
		RecreateSocketUDP();
		m_clMyConfigInfo.WriteConfigInfo();

		////////////////////// partner Camera�� �غ�������� Ȯ�� �� �ڵ�����
		IsRun(); IsRun();
		Sleep(100);	// Udp ���� thread���� Response message�� ���ŵ� ������ ��� 
		if(m_sockResponseUdp->m_nIsCamStatusRun == STATUS_STOP)
		{
			OnBnClickedCmdInit();
			SetGlobalGainAndExposure(m_cScrollBarGlobalGain.GetScrollPos(), 
				m_cScrollBarExposure.GetScrollPos());
			OnBnClickedRun();
			SetTimer(TIMER_UDP_DATA_COMM_DISPLAY, msec_IMAGE_DISPLAY_CYCLE, NULL);
		}
		else if(m_sockResponseUdp->m_nIsCamStatusRun == STATUS_RUN)
		{
			SetGlobalGainAndExposure(m_cScrollBarGlobalGain.GetScrollPos(), 
				m_cScrollBarExposure.GetScrollPos());
			OnBnClickedRun();
			SetTimer(TIMER_UDP_DATA_COMM_DISPLAY, msec_IMAGE_DISPLAY_CYCLE, NULL);
		}
	}

	return CDialogEx::OnWndMsg(message, wParam, lParam, pResult);
}


void CCommWithVSTCameraDlg::OnBnClickedButtonSaveAnImage()
{
	if(m_cChkIsJpg.GetCheck())
	{
		CString strFileName, strGetFileName;
		CLSID clsid;

		GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

		HDC memdcL;
		HBITMAP membitL;
		memdcL = CreateCompatibleDC(hdcFrame0);
		membitL = CreateCompatibleBitmap(hdcFrame0, FRAME_WIDTH, FRAME_HEIGHT);
		HBITMAP hOldBitmap =(HBITMAP) SelectObject(memdcL, membitL);
		BitBlt(memdcL, 0, 0, FRAME_WIDTH, FRAME_HEIGHT, hdcFrame0, 0, 0, SRCCOPY);

		Gdiplus::Bitmap bitmap(membitL, NULL);
		GetEncoderClsid(L"image/jpeg", &clsid);

		strGetFileName = GetNextImageFileName();
		strFileName.Format("%s\\%sl.jpg", m_cStrImageFolder.operator const char *(), 
			strGetFileName.operator const char *());
		WCHAR wcharPath[300];
		mbstowcs(wcharPath, _T(strFileName), strFileName.GetLength()+1);
		bitmap.Save(wcharPath, &clsid);

		HDC memdcR;
		HBITMAP membitR;
		memdcR = CreateCompatibleDC(hdcFrame1);
		membitR = CreateCompatibleBitmap(hdcFrame1, FRAME_WIDTH, FRAME_HEIGHT);
		HBITMAP hOldBitmapR =(HBITMAP) SelectObject(memdcR, membitR);
		BitBlt(memdcR, 0, 0, FRAME_WIDTH, FRAME_HEIGHT, hdcFrame1, 0, 0, SRCCOPY);

		Gdiplus::Bitmap bitmap2(membitR, NULL);
		strFileName.Format("%s\\%sr.jpg", m_cStrImageFolder.operator const char *(), 
			strGetFileName.operator const char *());
		mbstowcs(wcharPath, _T(strFileName), strFileName.GetLength()+1);
		bitmap2.Save(wcharPath, &clsid);

		SelectObject(memdcL, hOldBitmap);
		DeleteObject(memdcL);
		DeleteObject(membitL);
		SelectObject(memdcR, hOldBitmapR);
		DeleteObject(memdcR);
		DeleteObject(membitR);

		//GdiplusShutdown(gdiplusToken);
	}
	else
	{
		byte *videoBuf = m_sockUdpForImgData->m_ucpImgCopy;
		CString filename;

		filename.Format("%s\\%s.raw", m_cStrImageFolder.operator const char *(), 
			GetNextImageFileName().operator const char *());
		FILE *fp = fopen(filename.operator const char *(), "wb");
		if(fp==NULL)
		{
			CString sTmp;
			sTmp.Format(_T("File - Write Error!"));
			MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());		
		}
		else{
			fwrite(videoBuf, FRAME_WIDTH*FRAME_HEIGHT*BYTES_1_PIXEL, 1, fp);
			fclose(fp);
		}
	}
}

void CCommWithVSTCameraDlg::OnBnClickedCheckAutoSave()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	CString sTmp;
	if( m_cChkAutoSave.GetCheck() ){
		sTmp.Format(_T("Start saving images!"));
		MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());	
	}
	else{
		sTmp.Format(_T("Stop saving images!"));
		MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());
	}
}


void CCommWithVSTCameraDlg::OnBnClickedCheckIsJpg()
{
	// TODO: Add your control notification handler code here
	CString sTmp;
	if( m_cChkIsJpg.GetCheck() ){
		sTmp.Format(_T("Start managing with JPG images!"));
		MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());	
	}
	else{
		sTmp.Format(_T("Start managing with RAW images!"));
		MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());
	}
}

CString CCommWithVSTCameraDlg::GetNextImageFileName(void)
{
	static int lastmm=255, lasts=255, count=0;
	CString str;
	int d, m, h, mm, s;

	CTime CurTime = CTime::GetCurrentTime();	

	d = CurTime.GetDay();
	m = CurTime.GetMonth();

	h = CurTime.GetHour();
	mm = CurTime.GetMinute();
	s = CurTime.GetSecond();

	if(mm != lastmm || s != lasts)
	{
		count = 1;
		lastmm = mm;
		lasts = s;
	}
	else
		count++;

	str.Format("%02d%02d-%02d%02d%02d-%02d",m, d, h, mm, s, count);
	return str;	
}


void CCommWithVSTCameraDlg::OnBnClickedButtonLoadAFileImage()
{
	CString loadFile="";
	CFileDialog *filedlg;
	if(m_cChkIsJpg.GetCheck())
		filedlg = new CFileDialog (true, 0, 0, OFN_HIDEREADONLY, "jpg file(*.jpg) | *.jpg|", NULL, 0);
	else	
		filedlg = new CFileDialog (true, 0, 0, OFN_HIDEREADONLY, "raw file(*.raw) | *.raw|", NULL, 0);
	
	filedlg->m_ofn.lpstrInitialDir = m_cStrImageFolder; 
	if(filedlg->DoModal() != IDOK){
		filedlg->DestroyWindow();
		return ;
	}

	OnBnClickedCmdStop();

	//m_cStrImageFolder = filedlg->GetFolderPath();	// �ѱ������̸� ���� ����.
													// �ѱ������̸� ������ ����ϴ� �� ����
	loadFile = filedlg->GetPathName();
	SetDlgItemText(IDC_EDIT_LOAD_IMAGE_PATH, loadFile);
	
	ReadAImageFileAndDisplay(loadFile);
	m_cStrCurFileName = filedlg->GetFileName();	// ���� ǥ�õ� ���ϳ��� ���
	if(m_cChkIsJpg.GetCheck())					// ������ left image file�� ���
		m_cStrCurFileName.Replace(_T("r.jpg"), _T("l.jpg"));

	// �ٲ� ���丮 ����
	if(dir != NULL) closedir(dir);
	dir = opendir (m_cStrImageFolder.operator const char *());
	if(dir == NULL) {
		CString sTmp;
		sTmp.Format(_T("opendir() Error!"));
		MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());
	}

	return;
}

void CCommWithVSTCameraDlg::ReadAImageFileAndDisplay(CString loadFile)
{
	if(!strncmp("jpg", loadFile.Right(3), 3))
	{
		IStream *m_pStreamBmp;
		CLSID clsidBmp;
		ULONG readSize=1, offset = 0;

		GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

		CreateStreamOnHGlobal(NULL, TRUE, &m_pStreamBmp);
		GetEncoderClsid(L"image/bmp", &clsidBmp);

#if 1	// �����͸� bmp format�� �ִ� ��.
		// ���� left image file ó��
		loadFile.Replace(_T("r.jpg"), _T("l.jpg"));
		WCHAR wcharPath[300];
		mbstowcs(wcharPath, _T(loadFile), loadFile.GetLength()+1);
		Image imageL(wcharPath);
		imageL.Save(m_pStreamBmp, &clsidBmp);

		while(readSize)
		{
			// ������ file ������� ū ���� ������ ���� �߻�
			m_pStreamBmp->Read(&m_bmpImageL[offset], 1024, &readSize);
			offset += readSize;
		}

		// �ڿ� right image file ó��
		loadFile.Replace(_T("l.jpg"), _T("r.jpg"));
		mbstowcs(wcharPath, _T(loadFile), loadFile.GetLength()+1);
		Image imageR(wcharPath);
		imageR.Save(m_pStreamBmp, &clsidBmp);

		readSize=1; offset = 0;
		while(readSize)
		{
			// ������ file ������� ū ���� ������ ���� �߻�
			m_pStreamBmp->Read(&m_bmpImageR[offset], 1024, &readSize);
			offset += readSize;
		}

		// m_bmpImageL, m_bmpImageR�� �̹��������� ä�������Ƿ�
		// ���⼭ ���μ���

		::StretchDIBits(hdcFrame0, 0, 0,
			FRAME_WIDTH, FRAME_HEIGHT,
			0, 0, FRAME_WIDTH, FRAME_HEIGHT, 
			m_bmpImageL,
			&m_szBitmapinfo, 
			DIB_RGB_COLORS,
			SRCCOPY);

		::StretchDIBits(hdcFrame1, 0, 0,
			FRAME_WIDTH, FRAME_HEIGHT,
			0, 0, FRAME_WIDTH, FRAME_HEIGHT, 
			m_bmpImageR,
			&m_szBitmapinfo, 
			DIB_RGB_COLORS,
			SRCCOPY);

#else	// ȭ�鿡 �Ѹ��⸸ �ϴ� ��
		loadFile.Replace(_T("l.jpg"), _T("r.jpg"));
		USES_CONVERSION;
		WCHAR *wstr = T2W(loadFile.GetBuffer(0));
		Image imageL(wstr);
		Graphics graphicsL(hdcFrame0);
		graphicsL.DrawImage(&imageL, 0, 0, FRAME_WIDTH, FRAME_HEIGHT);

		loadFile.Replace(_T("l.jpg"), _T("r.jpg"));
		wstr = T2W(loadFile.GetBuffer(0));
		Image imageR(wstr);
		Graphics graphicsR(hdcFrame1);
		graphicsR.DrawImage(&imageR, 0, 0, FRAME_WIDTH, FRAME_HEIGHT);

#endif
		//GdiplusShutdown(gdiplusToken); // Ǯ�� �ι�°���� ����ó������ ���� �޽����� ��Ÿ����.

	}
	else if(!strncmp("raw", loadFile.Right(3), 3))
	{
		FILE *fp = fopen(loadFile.operator const char *(), "rb");
		if(fp == NULL){
			CString sTmp;
			sTmp.Format(_T("File - Open Error!"));
			MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());		
			return;
		}

		byte *videoBuf = m_sockUdpForImgData->m_ucpImgCopy;

		int size = FRAME_HEIGHT * FRAME_WIDTH * BYTES_1_PIXEL;
		int rdsz = fread(videoBuf, 1, size, fp);
		if(rdsz != size){
			fclose(fp);
			CString sTmp;
			sTmp.Format(_T("File - Read Error!"));
			MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());		
			return;
		}

		m_sockUdpForImgData->m_bDidRx = true;
		ShowAndSaveImage();
		fclose(fp);
	}

	CString str;
	str.Format("%s: %s %s", MODIFIED_PROGRAM_NAME, m_szMyIpAddr, loadFile);
	SetWindowText(str);
}

void CCommWithVSTCameraDlg::OnBnClickedCheckAutoLoad()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	CString sTmp;
	if( m_cChkAutoLoad.GetCheck() )
	{
		GetDlgItem(IDC_RUN)->EnableWindow(false);
		OnBnClickedCmdStop();

		sTmp.Format(_T("Start loading images periodically!"));
		MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());

		rewinddir(dir);
		SetTimer(TIMER_IMG_DISPLAY, 50, NULL);
	}
	else{
		GetDlgItem(IDC_RUN)->EnableWindow(true);
		sTmp.Format(_T("Stop loading images!"));
		MessageDisplayAndLog((BYTE*)(LPSTR)(LPCTSTR)sTmp, sTmp.GetLength());

		KillTimer(TIMER_IMG_DISPLAY);
	}
}

void CCommWithVSTCameraDlg::ReadAFileInOrderAndDisplay()
{
	do{
		ent = readdir (dir);
		if(ent == NULL)
		{
			break;
		}
	}while(ent->d_type != DT_REG || 
		m_cChkIsJpg.GetCheck() ? strncmp("l.jpg", &ent->d_name[ent->d_namlen - 5], 5) :
		strncmp("raw", &ent->d_name[ent->d_namlen - 3], 3));

	if(ent != NULL)
	{
		char fileName[256];
		sprintf(fileName, "%s\\%s\0", m_cStrImageFolder.operator const char *(), ent->d_name); 
	
		ReadAImageFileAndDisplay(fileName);	

		m_cStrCurFileName.Format("%s", ent->d_name);		// ���� ǥ�õ� ���ϳ��� ���
	}
}

void CCommWithVSTCameraDlg::OnBnClickedButtonLoadNext()
{
	// ��ó������ m_cStrCurFileName �����Ͱ� �����Ƿ� ���ذ�����
	if(strcmp(m_cStrCurFileName.operator const char *(), ""))
	{
		// ���� ������ ã�� ���ؼ��� ó������ �ٽ� ã�´�
		rewinddir(dir);
		do{
			ent = readdir (dir);
			if(ent == NULL)
			{
				return;
			}
		}while(strcmp(m_cStrCurFileName.operator const char *(), ent->d_name));
	}

	ReadAFileInOrderAndDisplay();

	m_cStrCurFileName.Format("%s", ent->d_name);		// ���� ǥ�õ� ���ϳ��� ���
}

int CCommWithVSTCameraDlg::GetEncoderClsid(const WCHAR* format, CLSID* pClsid)
{
	using namespace Gdiplus;
	UINT  num = 0;          // number of image encoders
	UINT  size = 0;         // size of the image encoder array in bytes

	ImageCodecInfo* pImageCodecInfo = NULL;

	GetImageEncodersSize(&num, &size);
	if(size == 0)
		return -1;  // Failure

	pImageCodecInfo = (ImageCodecInfo*)(malloc(size));
	if(pImageCodecInfo == NULL)
		return -1;  // Failure

	GetImageEncoders(num, size, pImageCodecInfo);

	for(UINT j = 0; j < num; ++j)
	{
		if( wcscmp(pImageCodecInfo[j].MimeType, format) == 0 )
		{
			*pClsid = pImageCodecInfo[j].Clsid;
			free(pImageCodecInfo);
			return j;  // Success
		}    
	}

	free(pImageCodecInfo);
	return 0;
}

