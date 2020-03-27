#include "display_code.h"
#include "rw_spiflash.h"

const char Code_Warning[][12] = {	
	{"�Ƿ�ɾ����"},
	{"�Ƿ�������"},
	{"������..."},
};

const char Code_Message[][12] = {	
	{"�������!"},
	{"U��δ����!"},
	{"������..."},
	{"��Ч����"},
};

void SaveUIEditInfor(void)
{
	appdis.pUI->editinfo.screen_id = appdis.pUI->screen_id;
	appdis.pUI->editinfo.ctrl_id = appdis.pUI->ctrl_id;
}

void DisplayUIIDAndBackup(u8 id)
{
	appdis.pUI->screen_idbk = appdis.pUI->screen_id;
//	LIFOBuffer_Insert(&ScreenIDLIFO, (u8 *)&appdis.pUI->screen_id);
	appdis.pUI->ctrl_idbk = appdis.pUI->ctrl_id;
	appdis.pUI->screen_id = id;
	DaCai_SwitchUI(appdis.pUI);
}

void DisplayBackupUIID(void)
{
	appdis.pUI->screen_id = appdis.pUI->screen_idbk;
//	if(LIFOBuffer_Pop(&ScreenIDLIFO, (INT8U *)&appdis.pUI->screen_id)==1)	
	{//��ȡ
		appdis.pUI->ctrl_id = appdis.pUI->ctrl_idbk;
		DaCai_SwitchUI(appdis.pUI);
	}
}

void DisplayEditUI(void)
{
	appdis.pUI->screen_id = appdis.pUI->editinfo.screen_id;
	appdis.pUI->ctrl_id = appdis.pUI->editinfo.ctrl_id;
	DaCai_SwitchUI(appdis.pUI);
}

void DisplayKeyboardUI(void)
{
//	s8 id;
	DisplayUIIDAndBackup(Keyboard_UIID);
//	id = appdis.pUI->ctrl_id;
	appdis.pUI->ctrl_id = 43;
	DaCai_ClearTXT(appdis.pUI);
}

void DisplayMessageUI(char *pbuf)
{	
	DisplayUIIDAndBackup(Message_UIID);
//	appdis.pUI->sub_screen_id = Message_UIID;
//	DaCai_SwitchSubUI(appdis.pUI);
	appdis.pUI->ctrl_id  = 4;	
	appdis.pUI->datlen = sprintf((char *)appdis.pUI->pdata,"%s", pbuf);
	DaCai_UpdateTXT(appdis.pUI);
}

void DisplayLogUI(void)
{
	u32 ret;

	appdis.pUI->screen_id = Log_UIID;//LOG����					
	DaCai_SwitchUI(appdis.pUI);
	ret = read_log(appdis.pUI->pdata);
	if(ret)	{//��ȡlog���Ȳ�Ϊ0
		appdis.pUI->ctrl_id = 3;
		appdis.pUI->datlen = ret;
		DaCai_UpdateTXT(appdis.pUI);
	}
}
