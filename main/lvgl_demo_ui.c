/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/widgets/extra/meter.html#simple-meter

#include "lvgl.h"
#include "bsp/esp-bsp.h"

#include "clockwidget.h"

static lv_obj_t *meter;
static lv_obj_t * btn;
static lv_disp_rot_t rotation = LV_DISP_ROT_NONE;


//Objetos que forman parte del reloj
static lv_obj_t * lvMinute;
static lv_obj_t * lvHour;
static lv_obj_t * lvSecond ;



void ui_update_clock(uint8_t hour, uint8_t minute, uint8_t second )
{
	//Apply the limits.
	minute=minute%60;
	second=second%60;
	hour=hour%12;

    if(lvHour !=NULL)
    {
        lv_img_set_angle(  lvHour, hour*30*10);
        lv_obj_align(  lvHour, LV_ALIGN_CENTER, 0, 0);
    }

    if(lvMinute !=NULL)
    {
        lv_img_set_angle(  lvMinute, minute*6*10);
        lv_obj_align(  lvMinute, LV_ALIGN_CENTER, 0, 0);
    }
    if(lvSecond !=NULL)
    {
        lv_img_set_angle(  lvSecond, second*6*10);
        lv_obj_align(  lvSecond, LV_ALIGN_CENTER, 0, 0);
    }
}

void ui_example_lvgl_demo_init(lv_disp_t *disp)
{
    lv_obj_t *scr = lv_disp_get_scr_act(disp);


    lv_disp_set_rotation(disp, LV_DISP_ROT_90);


    //*******************************Crea un contenedor y lo añade a la pantalla
    lv_obj_t * cont = lv_obj_create(scr);
    lv_obj_set_size(cont, BSP_LCD_V_RES, BSP_LCD_H_RES);
    lv_obj_center(cont);
    /* Configura un estilo para el contenedor para determinar cómo se alinean y muestran los objetos que se agreguen al contenedor */
    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_flex_flow(&style, LV_FLEX_FLOW_ROW_WRAP);
    lv_style_set_flex_main_place(&style, LV_FLEX_ALIGN_SPACE_EVENLY);
    lv_style_set_layout(&style, LV_LAYOUT_FLEX);

    lv_obj_add_style(cont, &style, 0);

    //******** Crea la etiqueta y la agrega al contenedor
    lv_obj_t *label = lv_label_create(cont);
    lv_label_set_recolor(label, true);
    lv_obj_set_width(label, BSP_LCD_V_RES);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "#FF9400 "LV_SYMBOL_BELL" Ejemplo con BSP y LVGL "LV_SYMBOL_WARNING" #");

    //********* añade el medidor al contenedor y configura el medidor (añade escalas, etc)
    meter = lv_meter_create(cont);
    lv_obj_center(meter);
    lv_obj_set_size(meter, 125, 125);

    /*Add a scale first*/
    lv_meter_scale_t *scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_ticks(meter, scale, 41, 2, 4, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter, scale, 8, 4, 8, lv_color_black(), 10);

    lv_meter_indicator_t *indic;

    /*Add a blue arc to the start*/
    indic = lv_meter_add_arc(meter, scale, 3, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_meter_set_indicator_start_value(meter, indic, 0);
    lv_meter_set_indicator_end_value(meter, indic, 20);

    /*Make the tick lines blue at the start of the scale*/
    indic = lv_meter_add_scale_lines(meter, scale, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_BLUE), false, 0);
    lv_meter_set_indicator_start_value(meter, indic, 0);
    lv_meter_set_indicator_end_value(meter, indic, 20);

    /*Add a red arc to the end*/
    indic = lv_meter_add_arc(meter, scale, 3, lv_palette_main(LV_PALETTE_RED), 0);
    lv_meter_set_indicator_start_value(meter, indic, 80);
    lv_meter_set_indicator_end_value(meter, indic, 100);

    /*Make the tick lines red at the end of the scale*/
    indic = lv_meter_add_scale_lines(meter, scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false, 0);
    lv_meter_set_indicator_start_value(meter, indic, 80);
    lv_meter_set_indicator_end_value(meter, indic, 100);

    /*Add a needle line indicator*/
    indic = lv_meter_add_needle_line(meter, scale, 3, lv_palette_main(LV_PALETTE_GREY), -10);


    /*********** Añade el reloj al contenedor. El reloj está formado por un conjunto de imágenes superpuestas. */
    lv_obj_t * img = lv_img_create(cont);
    lv_img_set_src(img, &watch_bg_img);
    //lv_obj_set_size(img, 200, 200);
    lv_img_set_size_mode(img, LV_IMG_SIZE_MODE_REAL);
    lv_img_set_zoom(img, LV_IMG_ZOOM_NONE*0.625);
    lv_obj_align(  img, LV_ALIGN_CENTER, 0, 0);
    //lv_obj_set_pos(img,0,0);

    lvHour = lv_img_create(img);
    lv_img_set_src( lvHour, &hour_img);
    lv_img_set_size_mode(lvHour, LV_IMG_SIZE_MODE_REAL);
    lv_img_set_zoom(lvHour, LV_IMG_ZOOM_NONE*0.625);
    lv_img_set_angle(  lvHour, 750);
    lv_obj_align(  lvHour, LV_ALIGN_CENTER, 0, 0);

    lvMinute = lv_img_create(img);
    lv_img_set_src( lvMinute, &minute_img);
    lv_img_set_size_mode(lvMinute, LV_IMG_SIZE_MODE_REAL);
    lv_img_set_zoom(lvMinute, LV_IMG_ZOOM_NONE*0.625);
    lv_img_set_angle(  lvMinute, 1950);
    lv_obj_align(  lvMinute,LV_ALIGN_CENTER, 0, 0);

    lvSecond = lv_img_create(img);
    lv_img_set_src( lvSecond, &second_img);
    lv_img_set_size_mode(lvSecond, LV_IMG_SIZE_MODE_REAL);
    lv_img_set_zoom(lvSecond,LV_IMG_ZOOM_NONE *0.625);
    lv_img_set_angle(  lvSecond, 600);
    lv_obj_align(  lvSecond, LV_ALIGN_CENTER, 0, 0);

    /******************** crea otro contenedor y lo agrega a primero ***** */
    lv_obj_t * cont2 = lv_obj_create(cont);
    //lv_obj_set_width(cont2, BSP_LCD_V_RES);
    lv_obj_center(cont2);
    lv_obj_add_style(cont2, &style, 0);

    /* ****** crea 3 LEDs y los agrega al contenedor 2 */
    lv_obj_t * led1  = lv_led_create(cont2);
    lv_obj_center(led1);
    lv_led_set_color(led1, lv_palette_main(LV_PALETTE_RED));
    lv_led_on(led1);

    lv_obj_t * led2  = lv_led_create(cont2);
    lv_obj_center(led2);
    lv_led_set_color(led2, lv_palette_main(LV_PALETTE_BLUE));
    lv_led_on(led2);


    lv_obj_t * led3  = lv_led_create(cont2);
    lv_obj_center(led3);
    lv_led_set_color(led3, lv_palette_main(LV_PALETTE_GREEN));
    lv_led_on(led3);

}
