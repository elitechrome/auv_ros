
// CommWithVSTCamera.h : PROJECT_NAME ���� ���α׷��� ���� �� ��� �����Դϴ�.
//

#pragma once

#ifndef __AFXWIN_H__
	#error "PCH�� ���� �� ������ �����ϱ� ���� 'stdafx.h'�� �����մϴ�."
#endif

#include "resource.h"		// �� ��ȣ�Դϴ�.


// CCommWithVSTCameraApp:
// �� Ŭ������ ������ ���ؼ��� CommWithVSTCamera.cpp�� �����Ͻʽÿ�.
//

class CCommWithVSTCameraApp : public CWinApp
{
public:
	CCommWithVSTCameraApp();

// �������Դϴ�.
public:
	virtual BOOL InitInstance();

// �����Դϴ�.

	DECLARE_MESSAGE_MAP()
};

extern CCommWithVSTCameraApp theApp;