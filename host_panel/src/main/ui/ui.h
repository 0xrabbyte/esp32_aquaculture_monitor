// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.3
// Project name: panel

#ifndef _PANEL_UI_H
#define _PANEL_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

#include "ui_helpers.h"
#include "ui_events.h"
// SCREEN: ui_QR_Code
void ui_QR_Code_screen_init(void);
void ui_event_QR_Code(lv_event_t * e);
extern lv_obj_t * ui_QR_Code;
void ui_event_QR_Code_Back_Button(lv_event_t * e);
extern lv_obj_t * ui_QR_Code_Back_Button;
extern lv_obj_t * ui_QR_Code_Image;
extern lv_obj_t * ui_UpdateDropdown;
// SCREEN: ui_Main_Screen
void ui_Main_Screen_screen_init(void);
void ui_event_Main_Screen(lv_event_t * e);
extern lv_obj_t * ui_Main_Screen;
void ui_event_Slave_Dropdown(lv_event_t * e);
extern lv_obj_t * ui_Slave_Dropdown;
extern lv_obj_t * ui_Info_Panel;
extern lv_obj_t * ui_Control_Panel;
extern lv_obj_t * ui_TDS_Text;
extern lv_obj_t * ui_pH_Text;
extern lv_obj_t * ui_NTU_Text;
extern lv_obj_t * ui_Water_Level_Text;
extern lv_obj_t * ui_Water_Temperature;
extern lv_obj_t * ui_TDS_Value;
extern lv_obj_t * ui_pH_Value;
extern lv_obj_t * ui_NTH_Value;
extern lv_obj_t * ui_Water_Level_Value;
extern lv_obj_t * ui_Water_Temperature_Value;
extern lv_obj_t * ui_Pump;
extern lv_obj_t * ui_Camera_Text;
extern lv_obj_t * ui_Feed_Text;
void ui_event_Pump_Switch(lv_event_t * e);
extern lv_obj_t * ui_Pump_Switch;
void ui_event_Camera_Button(lv_event_t * e);
extern lv_obj_t * ui_Camera_Button;
void ui_event_Feed_Button(lv_event_t * e);
extern lv_obj_t * ui_Feed_Button;
extern lv_obj_t * ui_TDS_Icon;
extern lv_obj_t * ui_pH_Icon;
extern lv_obj_t * ui_NTH_Icon;
extern lv_obj_t * ui_Water_Level_Icon;
extern lv_obj_t * ui_Water_Temperature_Icon;
extern lv_obj_t * ui_Pump_Icon;
extern lv_obj_t * ui_Camera_Icon;
extern lv_obj_t * ui_Feed_Icon;
void ui_event_Wifi_Button(lv_event_t * e);
extern lv_obj_t * ui_Wifi_Button;
void ui_event_Feed_Dropdown(lv_event_t * e);
extern lv_obj_t * ui_Feed_Dropdown;
// SCREEN: ui_Camera
void ui_Camera_screen_init(void);
void ui_event_Camera(lv_event_t * e);
extern lv_obj_t * ui_Camera;
void ui_event_Camera_Back_Button(lv_event_t * e);
extern lv_obj_t * ui_Camera_Image;
extern lv_obj_t * ui_Camera_Back_Button;
void ui_event_Refresh_Button(lv_event_t * e);
extern lv_obj_t * ui_Refresh_Button;
extern lv_obj_t * ui____initial_actions0;

LV_IMG_DECLARE(ui_img_906722072);    // assets\arrow-right.png
LV_IMG_DECLARE(ui_img_tds_png);    // assets\TDS.png
LV_IMG_DECLARE(ui_img_ph_png);    // assets\pH.png
LV_IMG_DECLARE(ui_img_nth_png);    // assets\NTH.png
LV_IMG_DECLARE(ui_img_water_level_png);    // assets\water_level.png
LV_IMG_DECLARE(ui_img_water_temperature_png);    // assets\water_temperature.png
LV_IMG_DECLARE(ui_img_pump_png);    // assets\pump.png
LV_IMG_DECLARE(ui_img_camera_png);    // assets\camera.png
LV_IMG_DECLARE(ui_img_feeding_png);    // assets\feeding.png
LV_IMG_DECLARE(ui_img_wifi_png);    // assets\wifi.png
LV_IMG_DECLARE(ui_img_1608143951);    // assets\arrow-left.png
LV_IMG_DECLARE(ui_img_refresh_png);    // assets\refresh.png

void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
