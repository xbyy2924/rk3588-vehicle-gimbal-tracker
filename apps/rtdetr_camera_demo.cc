// Real-time RT-DETR camera demo for RK3588.
// Capture/display runs at camera rate; inference always consumes the latest frame.

#include <SDL2/SDL.h>

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rtdetr.h"

namespace {

constexpr int kRequestedWidth = 1280;
constexpr int kRequestedHeight = 720;
constexpr int kRequestedBufferCount = 4;
constexpr int kInitialWindowWidth = 960;
constexpr int kInitialWindowHeight = 540;

volatile sig_atomic_t g_stop_requested = 0;

void signal_handler(int)
{
    g_stop_requested = 1;
}

int xioctl(int fd, unsigned long request, void* arg)
{
    int ret;
    do {
        ret = ioctl(fd, request, arg);
    } while (ret < 0 && errno == EINTR);
    return ret;
}

struct MappedBuffer {
    void* address = MAP_FAILED;
    size_t length = 0;
};

struct CapturedFrame {
    const uint8_t* data = nullptr;
    size_t bytes = 0;
    uint32_t buffer_index = 0;
};

class V4L2Camera {
public:
    ~V4L2Camera()
    {
        shutdown();
    }

    bool open_device(const char* device, int requested_width, int requested_height)
    {
        fd_ = ::open(device, O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) {
            std::fprintf(stderr, "open %s failed: %s\n", device, std::strerror(errno));
            return false;
        }

        v4l2_capability cap{};
        if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
            std::fprintf(stderr, "VIDIOC_QUERYCAP failed: %s\n", std::strerror(errno));
            return false;
        }
        const uint32_t capabilities =
            (cap.capabilities & V4L2_CAP_DEVICE_CAPS) ? cap.device_caps : cap.capabilities;
        if (!(capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) ||
            !(capabilities & V4L2_CAP_STREAMING)) {
            std::fprintf(stderr, "%s is not a streaming multiplanar capture device\n", device);
            return false;
        }

        v4l2_format fmt{};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        fmt.fmt.pix_mp.width = requested_width;
        fmt.fmt.pix_mp.height = requested_height;
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
        fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
        fmt.fmt.pix_mp.num_planes = 1;

        if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
            std::fprintf(stderr, "VIDIOC_S_FMT failed: %s\n", std::strerror(errno));
            return false;
        }
        if (fmt.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_NV12 ||
            fmt.fmt.pix_mp.num_planes != 1) {
            std::fprintf(stderr, "camera did not accept single-plane NV12\n");
            return false;
        }

        width_ = static_cast<int>(fmt.fmt.pix_mp.width);
        height_ = static_cast<int>(fmt.fmt.pix_mp.height);
        stride_ = static_cast<int>(fmt.fmt.pix_mp.plane_fmt[0].bytesperline);
        size_image_ = static_cast<size_t>(fmt.fmt.pix_mp.plane_fmt[0].sizeimage);
        if (stride_ <= 0) {
            stride_ = width_;
        }

        const size_t required_nv12 =
            static_cast<size_t>(stride_) * static_cast<size_t>(height_) * 3U / 2U;
        if (size_image_ < required_nv12) {
            std::fprintf(stderr,
                         "invalid NV12 buffer: sizeimage=%zu required=%zu\n",
                         size_image_, required_nv12);
            return false;
        }

        v4l2_requestbuffers request{};
        request.count = kRequestedBufferCount;
        request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        request.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd_, VIDIOC_REQBUFS, &request) < 0) {
            std::fprintf(stderr, "VIDIOC_REQBUFS failed: %s\n", std::strerror(errno));
            return false;
        }
        if (request.count < 3) {
            std::fprintf(stderr, "camera returned only %u mmap buffers\n", request.count);
            return false;
        }

        buffers_.resize(request.count);
        for (uint32_t i = 0; i < request.count; ++i) {
            v4l2_plane plane{};
            v4l2_buffer buffer{};
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.index = i;
            buffer.length = 1;
            buffer.m.planes = &plane;

            if (xioctl(fd_, VIDIOC_QUERYBUF, &buffer) < 0) {
                std::fprintf(stderr, "VIDIOC_QUERYBUF[%u] failed: %s\n",
                             i, std::strerror(errno));
                return false;
            }

            buffers_[i].length = plane.length;
            buffers_[i].address = mmap(nullptr, plane.length,
                                       PROT_READ | PROT_WRITE, MAP_SHARED,
                                       fd_, plane.m.mem_offset);
            if (buffers_[i].address == MAP_FAILED) {
                std::fprintf(stderr, "mmap[%u] failed: %s\n", i, std::strerror(errno));
                return false;
            }
        }

        for (uint32_t i = 0; i < buffers_.size(); ++i) {
            if (!queue_buffer(i)) {
                return false;
            }
        }

        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
            std::fprintf(stderr, "VIDIOC_STREAMON failed: %s\n", std::strerror(errno));
            return false;
        }
        streaming_ = true;

        std::printf("camera: %s, %dx%d NV12, stride=%d, sizeimage=%zu, buffers=%zu\n",
                    device, width_, height_, stride_, size_image_, buffers_.size());
        return true;
    }

    // Returns 1 for a frame, 0 for timeout/interruption, and -1 for a fatal error.
    int dequeue(CapturedFrame* frame, int timeout_ms)
    {
        pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN | POLLPRI;

        int poll_ret;
        do {
            poll_ret = poll(&pfd, 1, timeout_ms);
        } while (poll_ret < 0 && errno == EINTR && !g_stop_requested);

        if (poll_ret == 0 || (poll_ret < 0 && errno == EINTR)) {
            return 0;
        }
        if (poll_ret < 0) {
            std::fprintf(stderr, "camera poll failed: %s\n", std::strerror(errno));
            return -1;
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            std::fprintf(stderr, "camera poll revents=0x%x\n", pfd.revents);
            return -1;
        }

        v4l2_plane plane{};
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.length = 1;
        buffer.m.planes = &plane;

        if (xioctl(fd_, VIDIOC_DQBUF, &buffer) < 0) {
            if (errno == EAGAIN) {
                return 0;
            }
            std::fprintf(stderr, "VIDIOC_DQBUF failed: %s\n", std::strerror(errno));
            return -1;
        }
        if (buffer.index >= buffers_.size()) {
            std::fprintf(stderr, "invalid V4L2 buffer index %u\n", buffer.index);
            return -1;
        }

        const size_t used = plane.bytesused ? plane.bytesused : buffers_[buffer.index].length;
        const size_t required_nv12 =
            static_cast<size_t>(stride_) * static_cast<size_t>(height_) * 3U / 2U;
        if (used < required_nv12 || buffers_[buffer.index].length < required_nv12) {
            std::fprintf(stderr, "short NV12 frame: used=%zu required=%zu\n",
                         used, required_nv12);
            queue_buffer(buffer.index);
            return 0;
        }

        frame->data = static_cast<const uint8_t*>(buffers_[buffer.index].address);
        frame->bytes = required_nv12;
        frame->buffer_index = buffer.index;
        return 1;
    }

    bool queue_buffer(uint32_t index)
    {
        v4l2_plane plane{};
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = index;
        buffer.length = 1;
        buffer.m.planes = &plane;
        if (xioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
            std::fprintf(stderr, "VIDIOC_QBUF[%u] failed: %s\n",
                         index, std::strerror(errno));
            return false;
        }
        return true;
    }

    void shutdown()
    {
        if (streaming_ && fd_ >= 0) {
            v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            xioctl(fd_, VIDIOC_STREAMOFF, &type);
            streaming_ = false;
        }
        for (MappedBuffer& buffer : buffers_) {
            if (buffer.address != MAP_FAILED) {
                munmap(buffer.address, buffer.length);
                buffer.address = MAP_FAILED;
            }
        }
        buffers_.clear();
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    int width() const { return width_; }
    int height() const { return height_; }
    int stride() const { return stride_; }
    size_t frame_bytes() const
    {
        return static_cast<size_t>(stride_) * static_cast<size_t>(height_) * 3U / 2U;
    }

private:
    int fd_ = -1;
    bool streaming_ = false;
    int width_ = 0;
    int height_ = 0;
    int stride_ = 0;
    size_t size_image_ = 0;
    std::vector<MappedBuffer> buffers_;
};

struct LatestFrame {
    std::mutex mutex;
    std::condition_variable condition;
    std::vector<uint8_t> data;
    uint64_t sequence = 0;
};

struct LatestDetection {
    std::mutex mutex;
    object_detect_result_list results{};
    uint64_t source_sequence = 0;
    double inference_ms = 0.0;
    uint64_t completed_count = 0;
};

SDL_Rect aspect_fit_rect(int source_width, int source_height,
                         int output_width, int output_height)
{
    const double source_aspect = static_cast<double>(source_width) / source_height;
    const double output_aspect = static_cast<double>(output_width) / output_height;
    SDL_Rect rect{};

    if (output_aspect > source_aspect) {
        rect.h = output_height;
        rect.w = static_cast<int>(output_height * source_aspect + 0.5);
        rect.x = (output_width - rect.w) / 2;
        rect.y = 0;
    } else {
        rect.w = output_width;
        rect.h = static_cast<int>(output_width / source_aspect + 0.5);
        rect.x = 0;
        rect.y = (output_height - rect.h) / 2;
    }
    return rect;
}

void draw_boxes(SDL_Renderer* renderer,
                const object_detect_result_list& detections,
                const SDL_Rect& video_rect,
                int source_width,
                int source_height)
{
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, 0, 255, 80, 255);

    const double scale_x = static_cast<double>(video_rect.w) / source_width;
    const double scale_y = static_cast<double>(video_rect.h) / source_height;

    for (int i = 0; i < detections.count; ++i) {
        const object_detect_result& detection = detections.results[i];
        const int left = std::max(0, std::min(source_width - 1, detection.box.left));
        const int top = std::max(0, std::min(source_height - 1, detection.box.top));
        const int right = std::max(left + 1, std::min(source_width, detection.box.right));
        const int bottom = std::max(top + 1, std::min(source_height, detection.box.bottom));

        SDL_Rect box{};
        box.x = video_rect.x + static_cast<int>(left * scale_x + 0.5);
        box.y = video_rect.y + static_cast<int>(top * scale_y + 0.5);
        box.w = std::max(1, static_cast<int>((right - left) * scale_x + 0.5));
        box.h = std::max(1, static_cast<int>((bottom - top) * scale_y + 0.5));

        // Three nested rectangles provide a readable 3-pixel box without SDL_ttf.
        for (int thickness = 0; thickness < 3; ++thickness) {
            SDL_Rect line = box;
            line.x += thickness;
            line.y += thickness;
            line.w = std::max(1, line.w - 2 * thickness);
            line.h = std::max(1, line.h - 2 * thickness);
            SDL_RenderDrawRect(renderer, &line);
        }
    }
}

void inference_loop(rknn_app_context_t* app_ctx,
                    LatestFrame* latest_frame,
                    LatestDetection* latest_detection,
                    std::atomic<bool>* running,
                    int width,
                    int height,
                    int stride)
{
    std::vector<uint8_t> local_frame;
    uint64_t consumed_sequence = 0;

    while (running->load(std::memory_order_acquire)) {
        uint64_t frame_sequence = 0;
        {
            std::unique_lock<std::mutex> lock(latest_frame->mutex);
            latest_frame->condition.wait(lock, [&] {
                return !running->load(std::memory_order_acquire) ||
                       latest_frame->sequence != consumed_sequence;
            });
            if (!running->load(std::memory_order_acquire)) {
                break;
            }
            local_frame = latest_frame->data;
            frame_sequence = latest_frame->sequence;
            consumed_sequence = frame_sequence;
        }

        image_buffer_t source{};
        source.width = width;
        source.height = height;
        source.width_stride = stride;
        source.height_stride = height;
        source.format = IMAGE_FORMAT_YUV420SP_NV12;
        source.virt_addr = local_frame.data();
        source.size = static_cast<int>(local_frame.size());
        source.fd = -1;

        object_detect_result_list results{};
        const auto begin = std::chrono::steady_clock::now();
        const int ret = inference_rtdetr_model(app_ctx, &source, &results);
        const auto end = std::chrono::steady_clock::now();
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(end - begin).count();

        if (ret != 0) {
            std::fprintf(stderr, "inference_rtdetr_model failed: ret=%d\n", ret);
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(latest_detection->mutex);
            latest_detection->results = results;
            latest_detection->source_sequence = frame_sequence;
            latest_detection->inference_ms = elapsed_ms;
            ++latest_detection->completed_count;
        }
    }
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 2 || argc > 3) {
        std::printf("Usage: %s <model_path> [video_device]\n", argv[0]);
        return EXIT_FAILURE;
    }

    const char* model_path = argv[1];
    const char* video_device = argc == 3 ? argv[2] : "/dev/video11";
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    int exit_code = EXIT_FAILURE;
    rknn_app_context_t app_ctx{};
    bool postprocess_initialized = false;
    bool model_initialized = false;
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* texture = nullptr;
    std::atomic<bool> running{false};
    std::thread inference_thread;
    LatestFrame latest_frame;
    LatestDetection latest_detection;
    V4L2Camera camera;

    if (init_post_process() != 0) {
        std::fprintf(stderr, "init_post_process failed\n");
        goto cleanup;
    }
    postprocess_initialized = true;

    if (init_rtdetr_model(model_path, &app_ctx) != 0) {
        std::fprintf(stderr, "init_rtdetr_model failed: %s\n", model_path);
        goto cleanup;
    }
    model_initialized = true;

    if (!camera.open_device(video_device, kRequestedWidth, kRequestedHeight)) {
        goto cleanup;
    }

    latest_frame.data.resize(camera.frame_bytes());

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        goto cleanup;
    }

    window = SDL_CreateWindow("RT-DETR Camera",
                              SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                              kInitialWindowWidth, kInitialWindowHeight,
                              SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    if (!window) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        goto cleanup;
    }

    renderer = SDL_CreateRenderer(window, -1,
                                  SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer) {
        std::fprintf(stderr, "accelerated SDL renderer unavailable, using software: %s\n",
                     SDL_GetError());
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
    }
    if (!renderer) {
        std::fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
        goto cleanup;
    }

    texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_NV12,
                                SDL_TEXTUREACCESS_STREAMING,
                                camera.width(), camera.height());
    if (!texture) {
        std::fprintf(stderr, "SDL_CreateTexture(NV12) failed: %s\n", SDL_GetError());
        goto cleanup;
    }

    running.store(true, std::memory_order_release);
    inference_thread = std::thread(inference_loop, &app_ctx,
                                   &latest_frame, &latest_detection, &running,
                                   camera.width(), camera.height(), camera.stride());

    std::printf("controls: ESC/Q/window close to quit\n");
    std::printf("pipeline: camera/display asynchronous from RKNN inference\n");

    {
        uint64_t capture_count = 0;
        uint64_t last_capture_count = 0;
        uint64_t last_inference_count = 0;
        auto last_title_time = std::chrono::steady_clock::now();

        while (!g_stop_requested && running.load(std::memory_order_acquire)) {
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                if (event.type == SDL_QUIT ||
                    (event.type == SDL_KEYDOWN &&
                     (event.key.keysym.sym == SDLK_ESCAPE ||
                      event.key.keysym.sym == SDLK_q))) {
                    running.store(false, std::memory_order_release);
                }
            }
            if (!running.load(std::memory_order_acquire)) {
                break;
            }

            CapturedFrame frame;
            const int capture_ret = camera.dequeue(&frame, 100);
            if (capture_ret < 0) {
                running.store(false, std::memory_order_release);
                break;
            }
            if (capture_ret == 0) {
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(latest_frame.mutex);
                std::memcpy(latest_frame.data.data(), frame.data, frame.bytes);
                ++latest_frame.sequence;
            }
            latest_frame.condition.notify_one();

            const uint8_t* y_plane = frame.data;
            const uint8_t* uv_plane =
                frame.data + static_cast<size_t>(camera.stride()) * camera.height();
            if (SDL_UpdateNVTexture(texture, nullptr,
                                    y_plane, camera.stride(),
                                    uv_plane, camera.stride()) != 0) {
                std::fprintf(stderr, "SDL_UpdateNVTexture failed: %s\n", SDL_GetError());
                camera.queue_buffer(frame.buffer_index);
                running.store(false, std::memory_order_release);
                break;
            }

            object_detect_result_list detections{};
            double inference_ms = 0.0;
            uint64_t inference_count = 0;
            {
                std::lock_guard<std::mutex> lock(latest_detection.mutex);
                detections = latest_detection.results;
                inference_ms = latest_detection.inference_ms;
                inference_count = latest_detection.completed_count;
            }

            int output_width = 0;
            int output_height = 0;
            SDL_GetRendererOutputSize(renderer, &output_width, &output_height);
            const SDL_Rect video_rect = aspect_fit_rect(
                camera.width(), camera.height(), output_width, output_height);

            SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
            SDL_RenderClear(renderer);
            SDL_RenderCopy(renderer, texture, nullptr, &video_rect);
            draw_boxes(renderer, detections, video_rect,
                       camera.width(), camera.height());
            SDL_RenderPresent(renderer);

            if (!camera.queue_buffer(frame.buffer_index)) {
                running.store(false, std::memory_order_release);
                break;
            }
            ++capture_count;

            const auto now = std::chrono::steady_clock::now();
            const double title_interval =
                std::chrono::duration<double>(now - last_title_time).count();
            if (title_interval >= 1.0) {
                const double capture_fps =
                    static_cast<double>(capture_count - last_capture_count) / title_interval;
                const double inference_fps =
                    static_cast<double>(inference_count - last_inference_count) / title_interval;

                char title[256];
                std::snprintf(title, sizeof(title),
                              "RT-DETR | camera %.1f FPS | inference %.1f FPS / %.1f ms | objects %d",
                              capture_fps, inference_fps, inference_ms, detections.count);
                SDL_SetWindowTitle(window, title);

                last_capture_count = capture_count;
                last_inference_count = inference_count;
                last_title_time = now;
            }
        }
    }

    exit_code = EXIT_SUCCESS;

cleanup:
    running.store(false, std::memory_order_release);
    latest_frame.condition.notify_all();
    if (inference_thread.joinable()) {
        inference_thread.join();
    }

    if (texture) {
        SDL_DestroyTexture(texture);
    }
    if (renderer) {
        SDL_DestroyRenderer(renderer);
    }
    if (window) {
        SDL_DestroyWindow(window);
    }
    SDL_Quit();
    camera.shutdown();

    if (model_initialized) {
        release_rtdetr_model(&app_ctx);
    }
    if (postprocess_initialized) {
        deinit_post_process();
    }
    return exit_code;
}
