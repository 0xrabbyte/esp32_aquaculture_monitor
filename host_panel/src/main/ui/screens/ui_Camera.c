// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.3
// Project name: panel

#include "../ui.h"
#include "../ui_helpers.h"
#include "esp_log.h"
#include "lvgl.h"

void ui_Camera_screen_init(void)
{
    ui_Camera = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Camera, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Camera_Back_Button = lv_imgbtn_create(ui_Camera);
    lv_imgbtn_set_src(ui_Camera_Back_Button, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_1608143951, NULL);
    lv_obj_set_width(ui_Camera_Back_Button, 50);
    lv_obj_set_height(ui_Camera_Back_Button, 50);
    lv_obj_set_x(ui_Camera_Back_Button, lv_pct(5));
    lv_obj_set_y(ui_Camera_Back_Button, lv_pct(5));

    ui_Camera_Image = lv_img_create(ui_Camera);
    load_image_from_file(ui_Camera_Image, "/spiffs/anime.jpg", 400, 300);
    lv_obj_set_width(ui_Camera_Image, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Camera_Image, LV_SIZE_CONTENT);
    lv_obj_set_x(ui_Camera_Image, 0);
    lv_obj_set_y(ui_Camera_Image, 0);
    lv_obj_set_align(ui_Camera_Image, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Camera_Image, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    ui_Refresh_Button = lv_imgbtn_create(ui_Camera);
    lv_imgbtn_set_src(ui_Refresh_Button, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_refresh_png, NULL);
    lv_obj_set_width(ui_Refresh_Button, 50);
    lv_obj_set_height(ui_Refresh_Button, 50);
    lv_obj_set_x(ui_Refresh_Button, lv_pct(-5));
    lv_obj_set_y(ui_Refresh_Button, lv_pct(5));
    lv_obj_set_align(ui_Refresh_Button, LV_ALIGN_TOP_RIGHT);

    lv_obj_add_event_cb(ui_Camera_Back_Button, ui_event_Camera_Back_Button, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Refresh_Button, ui_event_Refresh_Button, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Camera, ui_event_Camera, LV_EVENT_ALL, NULL);

}
