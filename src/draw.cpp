#include "init.h"
#include "liblvgl/lvgl.h"

lv_obj_t * img = lv_img_create(lv_scr_act());

void drawAutonSelector() {
    lv_img_set_src(img, "D:Vex Brain Screen 2024-2025 [RED RING AWP] (Resized).bin");
}