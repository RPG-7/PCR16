#ifndef __DACAI_API_H__
#define __DACAI_API_H__

#include "DaCai_ProtoAnalysis.h"

#define	SCREEN_BUFSIZE	4

typedef struct _UI_EDIT	{
	s8 screen_id;
	u8 ctrl_id;
}_UI_EDIT_T;

typedef struct _UI	{
	s8 screen_id;//µ±Ç°»­Ãæid
	s8 screen_idbk;
	s8 sub_screen_id;
	u8 ctrl_id;
	u8 ctrl_idbk;
	_UI_EDIT_T editinfo;
	s32 button_id;
	s8 index;	
	char *pdata;
	u16 datlen;
}_UI_t;

typedef struct _coordinate	{
	u16 x;
	u16 y;
}_coordinate_t;

extern _UI_t UI;

void DaCaiAPI_Init(void);
void DaCai_CheckDevice(void);
void DaCai_SwitchUI(_UI_t *pUI);
void DaCai_DynamicCtrl(_UI_t *pUI, u8 flag);
void DaCai_UpdateTXT(_UI_t *pUI);
void DaCai_ReadTXT(_UI_t *pUI);
void DaCai_GetScreenID(void);
void DaCai_StandbyMode(u8 state);
void DaCai_ButtonCtrl(_UI_t *pUI,u8 state);
void DaCai_ChooseCtrl(_UI_t *pUI);
void DaCai_SetCursorPos(_UI_t *pUI,u8 state);
void DaCai_DisKeyboard(u16 x,u16 y);
void DaCai_TimeGet(void);
void DaCai_TimeModeSet(void);
void DaCai_CtrlONOFF(_UI_t *pUI, u8 flag);
void DaCai_SwitchSubUI(_UI_t *pUI);
void DaCai_UpdateMultiButton(_UI_t *pUI,u8 *pButtonID, u8 *pVal, u8 num);
void DaCai_UpdateMultiTXT(_UI_t *pUI,_MultiDat *pMultiTXT, u8 num);
void DaCai_UpdateMultiIcon(_UI_t *pUI,_MultiDat *pMultiIcon, u8 num);
void DaCai_ScreenSaveMode(u8 enable, u16 standy_t, u8 standy_brightness, u8 brightness);
void DaCai_AdjustBrightness(u8 brightness);
void DaCai_ClearScreenAuto(u8 enable);
void DaCai_ClearScreen(u8 screen);
void DaCai_ClearTXT(_UI_t *pUI);
void DaCai_SetTXTBackgroundNoColor(_UI_t *pUI);
void DaCai_SetForeColor(_UI_t *pUI,u16 color);
void DaCai_SetTXTBackgroundColor(_UI_t *pUI, u16 color);
void DaCai_SetTXTtwinkle(_UI_t *pUI,u16 period);
void DaCai_EnterStandby(void);
void DaCai_ExitStandby(void);
void DaCai_PaintRectangle(u16 x, u16 y, u16 w, u16 h);
void DaCai_DisplayTXT(_UI_t *pUI, u16 x, u16 y, u8 font);
void  DaCai_DisplayCutPic(u16 x, u16 y, u8 Image_ID, u16 Image_X, u16 Image_Y, u16 Image_W, u16 Image_H);
void  DaCai_DisplayCutPicInBasicGraph(_UI_t *pUI, u8 Image_ID, u16 x, u16 y, u16 Image_X, u16 Image_Y, u16 Image_W, u16 Image_H);
void DaCai_PaintLine(_coordinate_t *pCoo, u8 size);
void DaCai_PaintLineInBasicGraph(_UI_t *pUI, u16 color, _coordinate_t *pCoo, u8 size);
void  DaCai_ClearBasicGraph(_UI_t *pUI);
void DaCai_IconCtrl(_UI_t *pUI, u8 icon);
void DaCai_PaintHollowRectangle(u16 x, u16 y, u16 w, u16 h);
void DaCai_DisplayUpdateOnOff(u8 flag);
void DaCai_SetFontColor(u16 color);
#endif
