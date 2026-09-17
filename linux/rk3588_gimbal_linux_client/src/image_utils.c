#include "image_utils.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static uint8_t clamp_u8(int value)
{
    if (value < 0) return 0;
    if (value > 255) return 255;
    return (uint8_t)value;
}

int get_image_size(image_buffer_t* image)
{
    if (image == NULL || image->width <= 0 || image->height <= 0) return 0;
    switch (image->format) {
    case IMAGE_FORMAT_GRAY8: return image->width * image->height;
    case IMAGE_FORMAT_RGB888: return image->width * image->height * 3;
    case IMAGE_FORMAT_RGBA8888: return image->width * image->height * 4;
    case IMAGE_FORMAT_YUV420SP_NV12:
    case IMAGE_FORMAT_YUV420SP_NV21:
        return image->width * image->height * 3 / 2;
    default: return 0;
    }
}

static void nv_to_rgb(uint8_t y, uint8_t u, uint8_t v, uint8_t* rgb)
{
    int c = (int)y - 16;
    const int d = (int)u - 128;
    const int e = (int)v - 128;
    if (c < 0) c = 0;
    rgb[0] = clamp_u8((298 * c + 409 * e + 128) >> 8);
    rgb[1] = clamp_u8((298 * c - 100 * d - 208 * e + 128) >> 8);
    rgb[2] = clamp_u8((298 * c + 516 * d + 128) >> 8);
}

static int nv12_to_rgb888(const image_buffer_t* src,
                          image_buffer_t* dst,
                          const image_rect_t* src_box,
                          const image_rect_t* dst_box,
                          uint8_t color)
{
    if (src == NULL || dst == NULL || src->virt_addr == NULL ||
        dst->virt_addr == NULL || dst->format != IMAGE_FORMAT_RGB888) return -1;
    if (src->format != IMAGE_FORMAT_YUV420SP_NV12 &&
        src->format != IMAGE_FORMAT_YUV420SP_NV21) return -1;

    const int src_stride = src->width_stride > 0 ? src->width_stride : src->width;
    const int src_hstride = src->height_stride > 0 ? src->height_stride : src->height;
    const int dst_stride = dst->width_stride > 0 ? dst->width_stride : dst->width;
    const int sx0 = src_box ? src_box->left : 0;
    const int sy0 = src_box ? src_box->top : 0;
    const int sw = src_box ? src_box->right - src_box->left + 1 : src->width;
    const int sh = src_box ? src_box->bottom - src_box->top + 1 : src->height;
    const int dx0 = dst_box ? dst_box->left : 0;
    const int dy0 = dst_box ? dst_box->top : 0;
    const int dw = dst_box ? dst_box->right - dst_box->left + 1 : dst->width;
    const int dh = dst_box ? dst_box->bottom - dst_box->top + 1 : dst->height;
    if (sw <= 0 || sh <= 0 || dw <= 0 || dh <= 0) return -1;

    memset(dst->virt_addr, color, (size_t)dst_stride * dst->height * 3U);
    const uint8_t* y_plane = src->virt_addr;
    const uint8_t* uv_plane = src->virt_addr + (size_t)src_stride * src_hstride;

    for (int dy = 0; dy < dh; ++dy) {
        int sy = sy0 + (int)(((int64_t)dy * sh) / dh);
        if (sy < 0) sy = 0;
        if (sy >= src->height) sy = src->height - 1;
        uint8_t* row = dst->virt_addr + ((size_t)(dy0 + dy) * dst_stride + dx0) * 3U;
        for (int dx = 0; dx < dw; ++dx) {
            int sx = sx0 + (int)(((int64_t)dx * sw) / dw);
            if (sx < 0) sx = 0;
            if (sx >= src->width) sx = src->width - 1;
            const uint8_t y = y_plane[(size_t)sy * src_stride + sx];
            const size_t uv_index = (size_t)(sy / 2) * src_stride + (size_t)(sx & ~1);
            uint8_t u = uv_plane[uv_index];
            uint8_t v = uv_plane[uv_index + 1U];
            if (src->format == IMAGE_FORMAT_YUV420SP_NV21) {
                const uint8_t tmp = u; u = v; v = tmp;
            }
            nv_to_rgb(y, u, v, row + (size_t)dx * 3U);
        }
    }
    return 0;
}

int convert_image(image_buffer_t* src, image_buffer_t* dst,
                  image_rect_t* src_box, image_rect_t* dst_box, char color)
{
    return nv12_to_rgb888(src, dst, src_box, dst_box, (uint8_t)color);
}

int convert_image_with_letterbox(image_buffer_t* src, image_buffer_t* dst,
                                 letterbox_t* letterbox, char color)
{
    if (src == NULL || dst == NULL || src->width <= 0 || src->height <= 0 ||
        dst->width <= 0 || dst->height <= 0) return -1;

    const float scale_w = (float)dst->width / (float)src->width;
    const float scale_h = (float)dst->height / (float)src->height;
    const float scale = scale_w < scale_h ? scale_w : scale_h;
    int resize_w = (int)floorf(src->width * scale);
    int resize_h = (int)floorf(src->height * scale);
    if (resize_w < 1) resize_w = 1;
    if (resize_h < 1) resize_h = 1;

    const int left = (dst->width - resize_w) / 2;
    const int top = (dst->height - resize_h) / 2;
    image_rect_t src_rect = {0, 0, src->width - 1, src->height - 1};
    image_rect_t dst_rect = {left, top, left + resize_w - 1, top + resize_h - 1};

    if (letterbox != NULL) {
        letterbox->scale = scale;
        letterbox->x_pad = left;
        letterbox->y_pad = top;
    }
    if (dst->virt_addr == NULL) {
        dst->size = get_image_size(dst);
        dst->virt_addr = (unsigned char*)malloc((size_t)dst->size);
        if (dst->virt_addr == NULL) return -1;
    }
    return convert_image(src, dst, &src_rect, &dst_rect, color);
}
