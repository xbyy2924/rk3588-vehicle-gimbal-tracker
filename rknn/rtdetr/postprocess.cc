// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rtdetr.h"

#include <math.h>
#include <algorithm>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include <set>
#include <vector>
#define LABEL_NALE_TXT_PATH "./model/coco_80_labels_list.txt"

static char *labels[OBJ_CLASS_NUM];

inline static int clamp(float val, int min, int max) { return val > min ? (val < max ? val : max) : min; }

static char *readLine(FILE *fp, char *buffer, int *len)
{
    int ch;
    int i = 0;
    size_t buff_len = 0;

    buffer = (char *)malloc(buff_len + 1);
    if (!buffer)
        return NULL; // Out of memory

    while ((ch = fgetc(fp)) != '\n' && ch != EOF)
    {
        buff_len++;
        void *tmp = realloc(buffer, buff_len + 1);
        if (tmp == NULL)
        {
            free(buffer);
            return NULL; // Out of memory
        }
        buffer = (char *)tmp;

        buffer[i] = (char)ch;
        i++;
    }
    buffer[i] = '\0';

    *len = buff_len;

    // Detect end
    if (ch == EOF && (i == 0 || ferror(fp)))
    {
        free(buffer);
        return NULL;
    }
    return buffer;
}

static int readLines(const char *fileName, char *lines[], int max_line)
{
    FILE *file = fopen(fileName, "r");
    char *s;
    int i = 0;
    int n = 0;

    if (file == NULL)
    {
        printf("Open %s fail!\n", fileName);
        return -1;
    }

    while ((s = readLine(file, s, &n)) != NULL)
    {
        lines[i++] = s;
        if (i >= max_line)
            break;
    }
    fclose(file);
    return i;
}

static int loadLabelName(const char *locationFilename, char *label[])
{
    printf("load lable %s\n", locationFilename);
    readLines(locationFilename, label, OBJ_CLASS_NUM);
    return 0;
}

static float sigmoid(float x) { return 1.0 / (1.0 + expf(-x)); }

static float unsigmoid(float y) { return -1.0 * logf((1.0 / y) - 1.0); }

inline static int32_t __clip(float val, float min, float max)
{
    float f = val <= min ? min : (val >= max ? max : val);
    return f;
}

static int8_t qnt_f32_to_affine(float f32, int32_t zp, float scale)
{
    float dst_val = (f32 / scale) + zp;
    int8_t res = (int8_t)__clip(dst_val, -128, 127);
    return res;
}

static uint8_t qnt_f32_to_affine_u8(float f32, int32_t zp, float scale)
{
    float dst_val = (f32 / scale) + zp;
    uint8_t res = (uint8_t)__clip(dst_val, 0, 255);
    return res;
}

static float deqnt_affine_to_f32(int8_t qnt, int32_t zp, float scale) { return ((float)qnt - (float)zp) * scale; }

static float deqnt_affine_u8_to_f32(uint8_t qnt, int32_t zp, float scale) { return ((float)qnt - (float)zp) * scale; }

// RKNN_RTDETR_DYNAMIC_QUERY_COUNT
static int process_i8(int8_t *pred_logits, int32_t score_zp, float score_scale,
                      int8_t *pred_boxes, int32_t box_zp, float box_scale,
                      std::vector<float> &boxes, 
                      std::vector<float> &objProbs, 
                      std::vector<int> &classId, 
                      int query_count,
                      float threshold)
{
    int validCount = 0;
    int8_t score_thres_i8 = qnt_f32_to_affine(threshold, score_zp, score_scale);
    for (int i = 0; i < query_count; i++)
    {
        int max_class_id = -1;
        int8_t max_score = -score_zp;
        for (int c= 0; c < OBJ_CLASS_NUM; c++){
            int offset = i * OBJ_CLASS_NUM + c;
            if (pred_logits[offset] > max_score)
            {
                max_score = pred_logits[offset];
                max_class_id = c;
            }
        }
        // compute box
        if (max_score > score_thres_i8){
            float cx = deqnt_affine_to_f32(pred_boxes[i * 4 + 0], box_zp, box_scale);
            float cy = deqnt_affine_to_f32(pred_boxes[i * 4 + 1], box_zp, box_scale);
            float w = deqnt_affine_to_f32(pred_boxes[i * 4 + 2], box_zp, box_scale);
            float h = deqnt_affine_to_f32(pred_boxes[i * 4 + 3], box_zp, box_scale);

            boxes.push_back(cx - 0.5 * w);
            boxes.push_back(cy - 0.5 * h);
            boxes.push_back(cx + 0.5 * w);
            boxes.push_back(cy + 0.5 * h);

            objProbs.push_back(deqnt_affine_to_f32(max_score, score_zp, score_scale));
            classId.push_back(max_class_id);
            validCount ++;
        }
    }
    return validCount;
}

static int process_fp32(float *pred_logits, float *pred_boxes,
                        std::vector<float> &boxes, 
                        std::vector<float> &objProbs, 
                        std::vector<int> &classId, 
                        int query_count,
                      float threshold)
{
    int validCount = 0;

    for (int i = 0; i < query_count; i++)
    {
        int max_class_id = -1;
        float max_score = 0;
        for (int c= 0; c< OBJ_CLASS_NUM; c++){
            int offset = i * OBJ_CLASS_NUM + c;
            if (pred_logits[offset] > max_score)
            {
                max_score = pred_logits[offset];
                max_class_id = c;
            }
        }

        // box
        if (max_score > threshold){
            float cx = pred_boxes[i * 4 + 0];
            float cy = pred_boxes[i * 4 + 1];
            float w = pred_boxes[i * 4 + 2];
            float h = pred_boxes[i * 4 + 3];

            boxes.push_back(cx - 0.5 * w);
            boxes.push_back(cy - 0.5 * h);
            boxes.push_back(cx + 0.5 * w);
            boxes.push_back(cy + 0.5 * h);

            objProbs.push_back(max_score);
            classId.push_back(max_class_id);
            validCount ++;
        }
    }
    return validCount;
}

int post_process(rknn_app_context_t *app_ctx, void *outputs, letterbox_t *letter_box, float conf_threshold, object_detect_result_list *od_results)
{
    rknn_output *_outputs = (rknn_output *)outputs;
    std::vector<float> boxes;
    std::vector<float> objProbs;
    std::vector<int> classId;
    int validCount = 0;
    int stride = 0;
    int grid_h = 0;
    int grid_w = 0;
    int model_in_w = app_ctx->model_width;
    int model_in_h = app_ctx->model_height;
    memset(od_results, 0, sizeof(object_detect_result_list));

    //  1*300*80    1*300*4
    if (app_ctx->is_quant)
    {
        validCount += process_i8((int8_t *)_outputs[0].buf, app_ctx->output_attrs[0].zp, app_ctx->output_attrs[0].scale,
                                    (int8_t *)_outputs[1].buf, app_ctx->output_attrs[1].zp, app_ctx->output_attrs[1].scale,
                                    boxes, objProbs, classId,
                                    app_ctx->output_attrs[0].dims[1], conf_threshold);
    }
    else
    {
        validCount += process_fp32((float *)_outputs[0].buf, (float *)_outputs[1].buf, boxes, objProbs, classId,
                                    app_ctx->output_attrs[0].dims[1], conf_threshold);
    }


    // RKNN_RTDETR_CLASS_AWARE_NMS
    // 混合量化可能产生高度重叠的重复查询框，按类别执行 NMS。
    if (validCount > 1)
    {
        const float nms_iou_threshold = 0.65f;
        const int count_before_nms = validCount;

        std::vector<int> nms_order;
        nms_order.reserve(validCount);

        for (int i = 0; i < validCount; ++i)
        {
            nms_order.push_back(i);
        }

        // 按置信度从高到低排序
        std::sort(
            nms_order.begin(),
            nms_order.end(),
            [&](int a, int b)
            {
                return objProbs[a] > objProbs[b];
            });

        std::vector<unsigned char> nms_removed(validCount, 0);
        std::vector<int> nms_keep;
        nms_keep.reserve(validCount);

        for (size_t oi = 0; oi < nms_order.size(); ++oi)
        {
            int i = nms_order[oi];

            if (nms_removed[i])
            {
                continue;
            }

            nms_keep.push_back(i);

            float ax1 = boxes[i * 4 + 0];
            float ay1 = boxes[i * 4 + 1];
            float ax2 = boxes[i * 4 + 2];
            float ay2 = boxes[i * 4 + 3];

            float area_a =
                std::max(0.0f, ax2 - ax1) *
                std::max(0.0f, ay2 - ay1);

            for (size_t oj = oi + 1; oj < nms_order.size(); ++oj)
            {
                int j = nms_order[oj];

                if (nms_removed[j] || classId[i] != classId[j])
                {
                    continue;
                }

                float bx1 = boxes[j * 4 + 0];
                float by1 = boxes[j * 4 + 1];
                float bx2 = boxes[j * 4 + 2];
                float by2 = boxes[j * 4 + 3];

                float inter_w = std::max(
                    0.0f,
                    std::min(ax2, bx2) - std::max(ax1, bx1));

                float inter_h = std::max(
                    0.0f,
                    std::min(ay2, by2) - std::max(ay1, by1));

                float inter_area = inter_w * inter_h;

                float area_b =
                    std::max(0.0f, bx2 - bx1) *
                    std::max(0.0f, by2 - by1);

                float union_area = area_a + area_b - inter_area;
                float iou = union_area > 0.0f
                                ? inter_area / union_area
                                : 0.0f;

                if (iou > nms_iou_threshold)
                {
                    nms_removed[j] = 1;
                }
            }
        }

        std::vector<float> filtered_boxes;
        std::vector<float> filtered_probs;
        std::vector<int> filtered_classes;

        filtered_boxes.reserve(nms_keep.size() * 4);
        filtered_probs.reserve(nms_keep.size());
        filtered_classes.reserve(nms_keep.size());

        for (int index : nms_keep)
        {
            filtered_boxes.push_back(boxes[index * 4 + 0]);
            filtered_boxes.push_back(boxes[index * 4 + 1]);
            filtered_boxes.push_back(boxes[index * 4 + 2]);
            filtered_boxes.push_back(boxes[index * 4 + 3]);

            filtered_probs.push_back(objProbs[index]);
            filtered_classes.push_back(classId[index]);
        }

        boxes.swap(filtered_boxes);
        objProbs.swap(filtered_probs);
        classId.swap(filtered_classes);
        validCount = static_cast<int>(nms_keep.size());

        printf(
            "class-aware NMS: %d -> %d, IoU=%.2f\n",
            count_before_nms,
            validCount,
            nms_iou_threshold);
    }

    // no object detect
    if (validCount <= 0)
    {
        printf("no object detect\n");
        return 0;
    }

    int last_count = 0;
    od_results->count = 0;
    for (int i = 0; i < validCount; ++i)
    {
        float x1 = boxes[i * 4 + 0] * model_in_w - letter_box->x_pad;
        float y1 = boxes[i * 4 + 1] * model_in_h - letter_box->y_pad;
        float x2 = boxes[i * 4 + 2] * model_in_w - letter_box->x_pad;
        float y2 = boxes[i * 4 + 3] * model_in_h - letter_box->y_pad;
        int id = classId[i];
        float obj_conf = objProbs[i];

        od_results->results[last_count].box.left = (int)(clamp(x1, 0, model_in_w) / letter_box->scale);
        od_results->results[last_count].box.top = (int)(clamp(y1, 0, model_in_h) / letter_box->scale);
        od_results->results[last_count].box.right = (int)(clamp(x2, 0, model_in_w) / letter_box->scale);
        od_results->results[last_count].box.bottom = (int)(clamp(y2, 0, model_in_h) / letter_box->scale);
        od_results->results[last_count].prop = obj_conf;
        od_results->results[last_count].cls_id = id;
        last_count++;
    }
    od_results->count = last_count;
    return 0;
}

int init_post_process()
{
    int ret = 0;
    ret = loadLabelName(LABEL_NALE_TXT_PATH, labels);
    if (ret < 0)
    {
        printf("Load %s failed!\n", LABEL_NALE_TXT_PATH);
        return -1;
    }
    return 0;
}

char *coco_cls_to_name(int cls_id)
{

    if (cls_id >= OBJ_CLASS_NUM)
    {
        return "null";
    }

    if (labels[cls_id])
    {
        return labels[cls_id];
    }

    return "null";
}

void deinit_post_process()
{
    for (int i = 0; i < OBJ_CLASS_NUM; i++)
    {
        if (labels[i] != nullptr)
        {
            free(labels[i]);
            labels[i] = nullptr;
        }
    }
}
