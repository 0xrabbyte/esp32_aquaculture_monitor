// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.3
// Project name: panel

#ifndef _PANEL_UI_HELPERS_H
#define _PANEL_UI_HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ui.h"

#define _UI_BASIC_PROPERTY_POSITION_X 0
#define _UI_BASIC_PROPERTY_POSITION_Y 1
#define _UI_BASIC_PROPERTY_WIDTH 2
#define _UI_BASIC_PROPERTY_HEIGHT 3
void _ui_basic_set_property(lv_obj_t * target, int id, int val);

#define _UI_DROPDOWN_PROPERTY_SELECTED 0
void _ui_dropdown_set_property(lv_obj_t * target, int id, int val);

#define _UI_IMAGE_PROPERTY_IMAGE 0
void _ui_image_set_property(lv_obj_t * target, int id, uint8_t * val);

#define _UI_LABEL_PROPERTY_TEXT 0
void _ui_label_set_property(lv_obj_t * target, int id, const char * val);

void _ui_screen_change(lv_obj_t * target, lv_scr_load_anim_t fademode, int spd, int delay);

#define _UI_MODIFY_FLAG_ADD 0
#define _UI_MODIFY_FLAG_REMOVE 1
#define _UI_MODIFY_FLAG_TOGGLE 2
void _ui_flag_modify(lv_obj_t * target, int32_t flag, int value);

#define _UI_MODIFY_STATE_ADD 0
#define _UI_MODIFY_STATE_REMOVE 1
#define _UI_MODIFY_STATE_TOGGLE 2
void _ui_state_modify(lv_obj_t * target, int32_t state, int value);

void _ui_opacity_set(lv_obj_t * target, int val);

/** Describes an animation*/
typedef struct _ui_anim_user_data_t {
    lv_obj_t * target;
    lv_img_dsc_t ** imgset;
    int32_t imgset_size;
    int32_t val;
} ui_anim_user_data_t;
void _ui_anim_callback_free_user_data(lv_anim_t * a);

void _ui_anim_callback_set_x(lv_anim_t * a, int32_t v);

void _ui_anim_callback_set_y(lv_anim_t * a, int32_t v);

void _ui_anim_callback_set_width(lv_anim_t * a, int32_t v);

void _ui_anim_callback_set_height(lv_anim_t * a, int32_t v);

void _ui_anim_callback_set_opacity(lv_anim_t * a, int32_t v);

void _ui_anim_callback_set_image_zoom(lv_anim_t * a, int32_t v);

void _ui_anim_callback_set_image_angle(lv_anim_t * a, int32_t v);

void _ui_anim_callback_set_image_frame(lv_anim_t * a, int32_t v);

int32_t _ui_anim_callback_get_x(lv_anim_t * a);

int32_t _ui_anim_callback_get_y(lv_anim_t * a);

int32_t _ui_anim_callback_get_width(lv_anim_t * a);

int32_t _ui_anim_callback_get_height(lv_anim_t * a);

int32_t _ui_anim_callback_get_opacity(lv_anim_t * a);

int32_t _ui_anim_callback_get_image_zoom(lv_anim_t * a);

int32_t _ui_anim_callback_get_image_angle(lv_anim_t * a);

int32_t _ui_anim_callback_get_image_frame(lv_anim_t * a);

void _ui_checked_set_text_value(lv_obj_t * trg, lv_obj_t * src, const char * txt_on, const char * txt_off);

void load_image_from_file(lv_obj_t * img, const char * file, int img_w, int img_h);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
