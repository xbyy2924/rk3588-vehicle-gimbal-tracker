#ifndef RK3588_GIMBAL_IMAGE_UTILS_H
#define RK3588_GIMBAL_IMAGE_UTILS_H

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int x_pad;
    int y_pad;
    float scale;
} letterbox_t;

int get_image_size(image_buffer_t* image);
int convert_image(image_buffer_t* src_image, image_buffer_t* dst_image,
                  image_rect_t* src_box, image_rect_t* dst_box, char color);
int convert_image_with_letterbox(image_buffer_t* src_image,
                                 image_buffer_t* dst_image,
                                 letterbox_t* letterbox, char color);

#ifdef __cplusplus
}
#endif

#endif
