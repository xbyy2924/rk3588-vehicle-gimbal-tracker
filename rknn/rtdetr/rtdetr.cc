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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <algorithm>
#include <functional>
#include <array>
#include <iostream>

#include "rtdetr.h"
#include "common.h"
#include "file_utils.h"
#include "image_utils.h"

#include "Float16.h"
#include "rknn_custom_op.h"

#include "dma_alloc.cpp"

static void dump_tensor_attr(rknn_tensor_attr *attr) {
    char dims[128] = {0};
    for (int i = 0; i < attr->n_dims; ++i) {
        int idx = strlen(dims);
        sprintf(&dims[idx], "%d%s", attr->dims[i], (i == attr->n_dims - 1) ? "" : ", ");
    }
    printf("  index=%d, name=%s, n_dims=%d, dims=[%s], n_elems=%d, size=%d, w_stride = %d, size_with_stride = %d, "
           "fmt=%s, type=%s, qnt_type=%s, "
           "zp=%d, scale=%f\n",
           attr->index, attr->name, attr->n_dims, dims, attr->n_elems, attr->size, attr->w_stride, attr->size_with_stride,
           get_format_string(attr->fmt), get_type_string(attr->type), get_qnt_type_string(attr->qnt_type), attr->zp,
           attr->scale);
}


template <typename T>
static void topk_indices(const T* input,
                         std::vector<int>& indices,
                         int n,
                         int k)
{
    std::vector<std::pair<T, int>> values;
    values.reserve(n);

    for (int i = 0; i < n; ++i) {
        values.push_back(std::make_pair(input[i], i));
    }

    std::partial_sort(
        values.begin(),
        values.begin() + k,
        values.end(),
        [](const std::pair<T, int>& a,
           const std::pair<T, int>& b) {
            return a.first > b.first;
        });

    indices.resize(k);
    for (int i = 0; i < k; ++i) {
        indices[i] = values[i].second;
    }
}

/*
 * 支持 FP32、INT8、UINT8 输入的自定义 TopK。
 * TopK 的索引输出根据实际输出类型写回。
 */
int compute_custom_topk_fp(rknn_custom_op_context* op_ctx,
                           rknn_custom_op_tensor* inputs,
                           uint32_t n_inputs,
                           rknn_custom_op_tensor* outputs,
                           uint32_t n_outputs)
{
    if (n_inputs < 1 || n_outputs < 2) {
        printf("custom TopK: invalid input/output count\n");
        return -1;
    }

    unsigned char* in_ptr =
        (unsigned char*)inputs[0].mem.virt_addr +
        inputs[0].mem.offset;

    unsigned char* out_ptr =
        (unsigned char*)outputs[1].mem.virt_addr +
        outputs[1].mem.offset;

    const int n = inputs[0].attr.n_elems;
    const int k = outputs[1].attr.n_elems;

    if (n <= 0 || k <= 0 || k > n) {
        printf("custom TopK: invalid N=%d K=%d\n", n, k);
        return -1;
    }

    static bool printed_once = false;
    if (!printed_once) {
        printf("custom TopK: input=%s, output=%s, N=%d, K=%d, "
               "input_zp=%d, input_scale=%f\n",
               get_type_string(inputs[0].attr.type),
               get_type_string(outputs[1].attr.type),
               n, k,
               inputs[0].attr.zp,
               inputs[0].attr.scale);
        printed_once = true;
    }

    std::vector<int> indices;

    switch (inputs[0].attr.type) {
    case RKNN_TENSOR_FLOAT32:
        topk_indices(
            reinterpret_cast<const float*>(in_ptr),
            indices, n, k);
        break;

    case RKNN_TENSOR_INT8:
        topk_indices(
            reinterpret_cast<const int8_t*>(in_ptr),
            indices, n, k);
        break;

    case RKNN_TENSOR_UINT8:
        topk_indices(
            reinterpret_cast<const uint8_t*>(in_ptr),
            indices, n, k);
        break;

    case RKNN_TENSOR_INT16:
        topk_indices(
            reinterpret_cast<const int16_t*>(in_ptr),
            indices, n, k);
        break;

    default:
        printf("custom TopK: unsupported input type=%s\n",
               get_type_string(inputs[0].attr.type));
        return -1;
    }

    switch (outputs[1].attr.type) {
    case RKNN_TENSOR_FLOAT32: {
        float* dst = reinterpret_cast<float*>(out_ptr);
        for (int i = 0; i < k; ++i) {
            dst[i] = static_cast<float>(indices[i]);
        }
        break;
    }

    case RKNN_TENSOR_INT32: {
        int32_t* dst = reinterpret_cast<int32_t*>(out_ptr);
        for (int i = 0; i < k; ++i) {
            dst[i] = static_cast<int32_t>(indices[i]);
        }
        break;
    }

    case RKNN_TENSOR_INT64: {
        int64_t* dst = reinterpret_cast<int64_t*>(out_ptr);
        for (int i = 0; i < k; ++i) {
            dst[i] = static_cast<int64_t>(indices[i]);
        }
        break;
    }

    default:
        printf("custom TopK: unsupported output type=%s\n",
               get_type_string(outputs[1].attr.type));
        return -1;
    }

    return 0;
}

int init_rtdetr_model(const char *model_path, rknn_app_context_t *app_ctx)
{
    int ret;
    int model_len = 0;
    char *model;
    rknn_context ctx = 0;

    // Load RKNN Model
    model_len = read_data_from_file(model_path, &model);
    if (model == NULL)
    {
        printf("load_model fail!\n");
        return -1;
    }

    ret = rknn_init(&ctx, model, model_len, 0, NULL);
    free(model);
    if (ret < 0)
    {
        printf("rknn_init fail! ret=%d\n", ret);
        return -1;
    }

    // RK3588: use all three NPU cores
    ret = rknn_set_core_mask(ctx, RKNN_NPU_CORE_0_1_2);
    if (ret != RKNN_SUCC)
    {
        printf("warning: rknn_set_core_mask failed, ret=%d, use default core mask\n", ret);
    }
    else
    {
        printf("NPU core mask: CORE_0_1_2\n");
    }

    // register a custom op
    rknn_custom_op user_op[1];
    memset(user_op, 0, sizeof(rknn_custom_op));
    strncpy(user_op[0].op_type, "TopK", RKNN_MAX_NAME_LEN - 1);
    user_op[0].version = 1;
    user_op[0].target  = RKNN_TARGET_TYPE_CPU;
    user_op[0].compute = compute_custom_topk_fp;
    ret = rknn_register_custom_ops(ctx, user_op, 1);
    if (ret < 0) {
        printf("rknn_register_custom_op fail! ret = %d\n", ret);
        return -1;
    }

    // Get Model Input Output Number
    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC)
    {
        printf("rknn_query fail! ret=%d\n", ret);
        return -1;
    }
    printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

    // Get Model Input Info
    printf("input tensors:\n");
    rknn_tensor_attr input_attrs[io_num.n_input];
    memset(input_attrs, 0, sizeof(input_attrs));
    for (int i = 0; i < io_num.n_input; i++)
    {
        input_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            return -1;
        }
        dump_tensor_attr(&(input_attrs[i]));
    }

    // Get Model Output Info
    printf("output tensors:\n");
    rknn_tensor_attr output_attrs[io_num.n_output];
    memset(output_attrs, 0, sizeof(output_attrs));
    for (int i = 0; i < io_num.n_output; i++)
    {
        output_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            return -1;
        }
        dump_tensor_attr(&(output_attrs[i]));
    }

    // Set to context
    app_ctx->rknn_ctx = ctx;

    // TODO
    if (output_attrs[0].qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC && output_attrs[0].type == RKNN_TENSOR_INT8)
    {
        app_ctx->is_quant = true;
    }
    else
    {
        app_ctx->is_quant = false;
    }

    app_ctx->io_num = io_num;
    app_ctx->input_attrs = (rknn_tensor_attr *)malloc(io_num.n_input * sizeof(rknn_tensor_attr));
    memcpy(app_ctx->input_attrs, input_attrs, io_num.n_input * sizeof(rknn_tensor_attr));
    app_ctx->output_attrs = (rknn_tensor_attr *)malloc(io_num.n_output * sizeof(rknn_tensor_attr));
    memcpy(app_ctx->output_attrs, output_attrs, io_num.n_output * sizeof(rknn_tensor_attr));

    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW)
    {
        printf("model is NCHW input fmt\n");
        app_ctx->model_channel = input_attrs[0].dims[1];
        app_ctx->model_height = input_attrs[0].dims[2];
        app_ctx->model_width = input_attrs[0].dims[3];
    }
    else
    {
        printf("model is NHWC input fmt\n");
        app_ctx->model_height = input_attrs[0].dims[1];
        app_ctx->model_width = input_attrs[0].dims[2];
        app_ctx->model_channel = input_attrs[0].dims[3];
    }
    printf("model input height=%d, width=%d, channel=%d\n",
           app_ctx->model_height, app_ctx->model_width, app_ctx->model_channel);

    return 0;
}

int release_rtdetr_model(rknn_app_context_t *app_ctx)
{
    if (app_ctx->input_attrs != NULL)
    {
        free(app_ctx->input_attrs);
        app_ctx->input_attrs = NULL;
    }
    if (app_ctx->output_attrs != NULL)
    {
        free(app_ctx->output_attrs);
        app_ctx->output_attrs = NULL;
    }
    if (app_ctx->rknn_ctx != 0)
    {
        rknn_destroy(app_ctx->rknn_ctx);
        app_ctx->rknn_ctx = 0;
    }
    return 0;
}

int inference_rtdetr_model(rknn_app_context_t *app_ctx, image_buffer_t *img, object_detect_result_list *od_results)
{
    int ret;
    image_buffer_t dst_img;
    letterbox_t letter_box;
    rknn_input inputs[app_ctx->io_num.n_input];
    rknn_output outputs[app_ctx->io_num.n_output];
    // const float nms_threshold = NMS_THRESH;      // 默认的NMS阈值
    const float box_conf_threshold = BOX_THRESH; // 默认的置信度阈值
    int bg_color = 114;

    if ((!app_ctx) || !(img) || (!od_results))
    {
        return -1;
    }

    memset(od_results, 0x00, sizeof(*od_results));
    memset(&letter_box, 0, sizeof(letterbox_t));
    memset(&dst_img, 0, sizeof(image_buffer_t));
    memset(inputs, 0, sizeof(inputs));
    memset(outputs, 0, sizeof(outputs));

    // Pre Process
    dst_img.width = app_ctx->model_width;
    dst_img.height = app_ctx->model_height;
    dst_img.format = IMAGE_FORMAT_RGB888;
    dst_img.size = get_image_size(&dst_img);
#if defined(DMA_ALLOC_DMA32)
    /*
     * Allocate dma_buf within 4G from dma32_heap,
     * return dma_fd and virtual address.
     */
    ret = dma_buf_alloc(DMA_HEAP_DMA32_UNCACHE_PATCH, dst_img.size, &dst_img.fd, (void **)&dst_img.virt_addr);
    if (ret < 0) {
        printf("alloc dma32_heap buffer failed!\n");
        return -1;
    }
#else
    dst_img.virt_addr = (unsigned char *)malloc(dst_img.size);
    if (dst_img.virt_addr == NULL)
    {
        printf("malloc buffer size:%d fail!\n", dst_img.size);
        return -1;
    }
#endif

    // letterbox
    ret = convert_image_with_letterbox(img, &dst_img, &letter_box, bg_color);
    if (ret < 0)
    {
        printf("convert_image_with_letterbox fail! ret=%d\n", ret);
        return -1;
    }

    // Set Input Data
    inputs[0].index = 0;
    inputs[0].type =  RKNN_TENSOR_UINT8;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].pass_through = 0;
    inputs[0].size = app_ctx->model_width * app_ctx->model_height * app_ctx->model_channel;
    inputs[0].buf = dst_img.virt_addr;

    ret = rknn_inputs_set(app_ctx->rknn_ctx, app_ctx->io_num.n_input, inputs);
    if (ret < 0)
    {
        printf("rknn_input_set fail! ret=%d\n", ret);
        return -1;
    }

    // Run
    printf("rknn_run\n");
    ret = rknn_run(app_ctx->rknn_ctx, nullptr);
    if (ret < 0)
    {
        printf("rknn_run fail! ret=%d\n", ret);
        return -1;
    }

    // Get Output
    for (int i = 0; i < app_ctx->io_num.n_output; i++)
    {
        outputs[i].index = i;
        outputs[i].want_float = (!app_ctx->is_quant);
        // outputs[i].is_prealloc = 0;
    }
    ret = rknn_outputs_get(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs, NULL);
    if (ret < 0)
    {
        printf("rknn_outputs_get fail! ret=%d\n", ret);
        goto out;
    }

    // Post Process
    // printf("post_process\n");
    post_process(app_ctx, outputs, &letter_box, box_conf_threshold, od_results);

    // Remeber to release rknn output
    rknn_outputs_release(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs);

out:
    if (dst_img.virt_addr != NULL)
    {
        #if defined(DMA_ALLOC_DMA32)
        dma_buf_free(dst_img.size, &dst_img.fd, dst_img.virt_addr);
        #else
        free(dst_img.virt_addr);
        #endif
    }

    return ret;
}

