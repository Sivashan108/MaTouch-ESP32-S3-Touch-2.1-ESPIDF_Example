#include <string.h>

#include "screens.h"
#include "images.h"
#include "fonts.h"
#include "actions.h"
#include "vars.h"
#include "styles.h"
#include "ui.h"

#include <string.h>

objects_t objects;
lv_obj_t *tick_value_change_obj;

void create_screen_main() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.main = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 480);
    lv_obj_set_style_bg_color(obj, lv_color_hex(0xffd97a45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(obj, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_blend_mode(obj, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);
    {
        lv_obj_t *parent_obj = obj;
        {
            // hello_label
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.hello_label = obj;
            lv_obj_set_pos(obj, 91, 202);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "Hello, world!");
            lv_obj_set_style_text_font(obj, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_letter_space(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
}

void tick_screen_main() {
}


void create_screens() {
    lv_disp_t *dispp = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    
    create_screen_main();
}

typedef void (*tick_screen_func_t)();

tick_screen_func_t tick_screen_funcs[] = {
    tick_screen_main,
};

void tick_screen(int screen_index) {
    tick_screen_funcs[screen_index]();
}
