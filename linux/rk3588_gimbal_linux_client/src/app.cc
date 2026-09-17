// Real-time YOLOv8n INT8 selectable-class object tracker for RK3588.
// Detection runs asynchronously; a constant-velocity tracker updates the box
// on every camera frame. No OpenCV or external tracking library is required.

#include <SDL2/SDL.h>

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <poll.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
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

#include "yolov8.h"
#include "coco_classes.h"
#include "gimbal_command_console.h"
#include "gimbal_control_state.h"
#include "gimbal_udp_controller.h"

namespace {

constexpr int kRequestedWidth = 1280;
constexpr int kRequestedHeight = 720;
constexpr int kRequestedBufferCount = 4;
constexpr int kInitialWindowWidth = 960;
constexpr int kInitialWindowHeight = 540;

constexpr float kTargetMinConfidence = 0.25F;
constexpr double kTrackHoldSeconds = 1.20;

constexpr double kGimbalControlPeriodSeconds = 1.0 / 30.0;
constexpr double kGimbalTrackFreshSeconds = 0.30;
constexpr double kNormalizedDeadband = 0.02;
constexpr double kGimbalErrorGain = 2.0;
constexpr double kGimbalCommandLeadSeconds = 0.10;
constexpr double kKalmanAccelerationNoise = 3.0;
constexpr double kKalmanFullControlAgeSeconds = 0.08;

volatile sig_atomic_t g_stop_requested = 0;

void signal_handler(int)
{
    g_stop_requested = 1;
}

std::string executable_directory()
{
    char path[PATH_MAX]{};
    const ssize_t length = ::readlink("/proc/self/exe", path, sizeof(path) - 1U);
    if (length <= 0) return ".";
    path[length] = '\0';
    const char* slash = std::strrchr(path, '/');
    if (!slash) return ".";
    return std::string(path, static_cast<size_t>(slash - path));
}

bool readable_file(const std::string& path)
{
    return ::access(path.c_str(), R_OK) == 0;
}

bool parse_port(const char* text, uint16_t* result)
{
    if (!text || !result || *text == '\0') return false;
    char* end = nullptr;
    errno = 0;
    const unsigned long value = std::strtoul(text, &end, 10);
    if (errno != 0 || !end || *end != '\0' || value == 0 || value > 65535UL) return false;
    *result = static_cast<uint16_t>(value);
    return true;
}

void print_usage(const char* program)
{
    std::fprintf(stderr,
        "Usage: %s [options]\n"
        "  --model PATH              RKNN model (default: app/model/yolov8n_rk3588_int8.rknn)\n"
        "  --labels PATH             COCO labels (default: app/model/coco_80_labels_list.txt)\n"
        "  --camera DEVICE           V4L2 camera (default: /dev/video11)\n"
        "  --bind-ip ADDRESS         Linux gimbal IP (default: 192.168.10.1)\n"
        "  --stm32-ip ADDRESS        STM32 IP (default: 192.168.10.2)\n"
        "  --command-port PORT       STM32 command port (default: 5000)\n"
        "  --telemetry-port PORT     Linux telemetry port (default: 5001)\n"
        "  --no-gimbal               run vision only\n"
        "  --help                    show this message\n",
        program);
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

struct GimbalControlStatus {
    std::mutex mutex;
    bool available = false;
    bool enabled = false;
    bool target_fresh = false;
    GimbalMode mode = GimbalMode::Hold;
    double m1_command_deg = 0.0;
    double m2_command_deg = 0.0;
    uint64_t telemetry_age_ms = UINT64_MAX;
};

struct FloatBox {
    float left = 0.0F;
    float top = 0.0F;
    float right = 0.0F;
    float bottom = 0.0F;
};

FloatBox to_float_box(const object_detect_result& detection)
{
    FloatBox box;
    box.left = static_cast<float>(detection.box.left);
    box.top = static_cast<float>(detection.box.top);
    box.right = static_cast<float>(detection.box.right);
    box.bottom = static_cast<float>(detection.box.bottom);
    return box;
}

float box_width(const FloatBox& box)
{
    return std::max(1.0F, box.right - box.left);
}

float box_height(const FloatBox& box)
{
    return std::max(1.0F, box.bottom - box.top);
}

float box_center_x(const FloatBox& box)
{
    return 0.5F * (box.left + box.right);
}

float box_center_y(const FloatBox& box)
{
    return 0.5F * (box.top + box.bottom);
}

float box_iou(const FloatBox& a, const FloatBox& b)
{
    const float intersection_left = std::max(a.left, b.left);
    const float intersection_top = std::max(a.top, b.top);
    const float intersection_right = std::min(a.right, b.right);
    const float intersection_bottom = std::min(a.bottom, b.bottom);
    const float intersection_width = std::max(0.0F, intersection_right - intersection_left);
    const float intersection_height = std::max(0.0F, intersection_bottom - intersection_top);
    const float intersection_area = intersection_width * intersection_height;
    const float union_area = box_width(a) * box_height(a) +
                             box_width(b) * box_height(b) - intersection_area;
    return union_area > 0.0F ? intersection_area / union_area : 0.0F;
}

void clamp_box(FloatBox* box, int width, int height)
{
    const float w = std::max(4.0F, std::min(static_cast<float>(width), box_width(*box)));
    const float h = std::max(4.0F, std::min(static_cast<float>(height), box_height(*box)));
    const float cx = std::max(w * 0.5F,
                              std::min(static_cast<float>(width) - w * 0.5F,
                                       box_center_x(*box)));
    const float cy = std::max(h * 0.5F,
                              std::min(static_cast<float>(height) - h * 0.5F,
                                       box_center_y(*box)));
    box->left = cx - w * 0.5F;
    box->right = cx + w * 0.5F;
    box->top = cy - h * 0.5F;
    box->bottom = cy + h * 0.5F;
}

class SingleObjectTracker {
public:
    bool set_target_class(int class_id)
    {
        if (class_id == target_class_id_) return false;
        reset();
        target_class_id_ = class_id;
        return true;
    }

    void predict(double dt_seconds, int frame_width, int frame_height)
    {
        if (!active_) {
            return;
        }
        // Limit dt so a temporary display stall cannot throw the prediction away.
        const float dt = static_cast<float>(std::max(0.0, std::min(0.10, dt_seconds)));
        box_.left += velocity_left_ * dt;
        box_.top += velocity_top_ * dt;
        box_.right += velocity_right_ * dt;
        box_.bottom += velocity_bottom_ * dt;
        clamp_box(&box_, frame_width, frame_height);

        // A missed detector update should gradually reduce velocity rather than
        // letting a stale estimate drift forever.
        velocity_left_ *= 0.985F;
        velocity_top_ *= 0.985F;
        velocity_right_ *= 0.985F;
        velocity_bottom_ *= 0.985F;
    }

    void update(const object_detect_result_list& detections,
                int frame_width,
                int frame_height,
                std::chrono::steady_clock::time_point now)
    {
        int matched_index = -1;

        if (active_) {
            float best_score = -1.0F;
            const float diagonal = std::sqrt(static_cast<float>(frame_width * frame_width +
                                                                  frame_height * frame_height));
            for (int i = 0; i < detections.count; ++i) {
                const object_detect_result& detection = detections.results[i];
                if (detection.cls_id != target_class_id_ ||
                    detection.prop < kTargetMinConfidence) {
                    continue;
                }

                const FloatBox candidate = to_float_box(detection);
                const float overlap = box_iou(box_, candidate);
                const float dx = box_center_x(candidate) - box_center_x(box_);
                const float dy = box_center_y(candidate) - box_center_y(box_);
                const float normalized_distance = std::sqrt(dx * dx + dy * dy) / diagonal;

                // Reject unrelated objects. The distance gate keeps association
                // working when fast motion makes IoU temporarily small.
                if (overlap < 0.05F && normalized_distance > 0.18F) {
                    continue;
                }
                const float association_score = 1.8F * overlap - normalized_distance +
                                                0.35F * detection.prop;
                if (association_score > best_score) {
                    best_score = association_score;
                    matched_index = i;
                }
            }
        } else {
            // Initial target: confidence first, box area second. This avoids
            // locking a tiny distant false positive when a clear target exists.
            float best_score = -1.0F;
            const float frame_area = static_cast<float>(frame_width * frame_height);
            for (int i = 0; i < detections.count; ++i) {
                const object_detect_result& detection = detections.results[i];
                if (detection.cls_id != target_class_id_ ||
                    detection.prop < kTargetMinConfidence) {
                    continue;
                }
                const FloatBox candidate = to_float_box(detection);
                const float relative_area = box_width(candidate) * box_height(candidate) /
                                            frame_area;
                const float selection_score = detection.prop +
                                              0.20F * std::sqrt(std::max(0.0F, relative_area));
                if (selection_score > best_score) {
                    best_score = selection_score;
                    matched_index = i;
                }
            }
        }

        if (matched_index >= 0) {
            correct(detections.results[matched_index], frame_width, frame_height, now);
            return;
        }

        if (active_ &&
            std::chrono::duration<double>(now - last_match_time_).count() >
                kTrackHoldSeconds) {
            reset();
        }
    }

    bool active() const { return active_; }
    int track_id() const { return track_id_; }
    int class_id() const { return class_id_; }
    float confidence() const { return confidence_; }
    const FloatBox& box() const { return box_; }

    double seconds_since_match(std::chrono::steady_clock::time_point now) const
    {
        return active_ ? std::chrono::duration<double>(now - last_match_time_).count() : 0.0;
    }

private:
    void correct(const object_detect_result& detection,
                 int frame_width,
                 int frame_height,
                 std::chrono::steady_clock::time_point now)
    {
        const FloatBox measured = to_float_box(detection);
        if (!active_) {
            active_ = true;
            ++next_track_id_;
            track_id_ = next_track_id_;
            box_ = measured;
            last_measured_box_ = measured;
            velocity_left_ = velocity_top_ = velocity_right_ = velocity_bottom_ = 0.0F;
        } else {
            const double elapsed = std::chrono::duration<double>(now - last_correction_time_).count();
            const float dt = static_cast<float>(std::max(0.02, std::min(0.50, elapsed)));

            const float measured_v_left = (measured.left - last_measured_box_.left) / dt;
            const float measured_v_top = (measured.top - last_measured_box_.top) / dt;
            const float measured_v_right = (measured.right - last_measured_box_.right) / dt;
            const float measured_v_bottom = (measured.bottom - last_measured_box_.bottom) / dt;

            constexpr float kPositionCorrection = 0.85F;
            constexpr float kVelocityCorrection = 0.50F;
            box_.left += kPositionCorrection * (measured.left - box_.left);
            box_.top += kPositionCorrection * (measured.top - box_.top);
            box_.right += kPositionCorrection * (measured.right - box_.right);
            box_.bottom += kPositionCorrection * (measured.bottom - box_.bottom);
            velocity_left_ += kVelocityCorrection * (measured_v_left - velocity_left_);
            velocity_top_ += kVelocityCorrection * (measured_v_top - velocity_top_);
            velocity_right_ += kVelocityCorrection * (measured_v_right - velocity_right_);
            velocity_bottom_ += kVelocityCorrection * (measured_v_bottom - velocity_bottom_);
            last_measured_box_ = measured;
        }

        clamp_box(&box_, frame_width, frame_height);
        class_id_ = detection.cls_id;
        confidence_ = detection.prop;
        last_match_time_ = now;
        last_correction_time_ = now;
    }

    void reset()
    {
        active_ = false;
        class_id_ = -1;
        confidence_ = 0.0F;
        velocity_left_ = velocity_top_ = velocity_right_ = velocity_bottom_ = 0.0F;
    }

    bool active_ = false;
    int track_id_ = 0;
    int next_track_id_ = 0;
    int class_id_ = -1;
    int target_class_id_ = 2;
    float confidence_ = 0.0F;
    FloatBox box_{};
    FloatBox last_measured_box_{};
    float velocity_left_ = 0.0F;
    float velocity_top_ = 0.0F;
    float velocity_right_ = 0.0F;
    float velocity_bottom_ = 0.0F;
    std::chrono::steady_clock::time_point last_match_time_{};
    std::chrono::steady_clock::time_point last_correction_time_{};
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

void draw_tracker(SDL_Renderer* renderer,
                  const SingleObjectTracker& tracker,
                  const SDL_Rect& video_rect,
                  int source_width,
                  int source_height,
                  std::chrono::steady_clock::time_point now)
{
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    const double scale_x = static_cast<double>(video_rect.w) / source_width;
    const double scale_y = static_cast<double>(video_rect.h) / source_height;

    // Red cross: camera optical/image center, used by the later gimbal controller.
    const int frame_center_x = video_rect.x + video_rect.w / 2;
    const int frame_center_y = video_rect.y + video_rect.h / 2;
    SDL_SetRenderDrawColor(renderer, 255, 60, 60, 220);
    SDL_RenderDrawLine(renderer, frame_center_x - 10, frame_center_y,
                      frame_center_x + 10, frame_center_y);
    SDL_RenderDrawLine(renderer, frame_center_x, frame_center_y - 10,
                      frame_center_x, frame_center_y + 10);

    if (!tracker.active()) {
        return;
    }

    const FloatBox& tracked = tracker.box();
    SDL_Rect box{};
    box.x = video_rect.x + static_cast<int>(tracked.left * scale_x + 0.5);
    box.y = video_rect.y + static_cast<int>(tracked.top * scale_y + 0.5);
    box.w = std::max(1, static_cast<int>(box_width(tracked) * scale_x + 0.5));
    box.h = std::max(1, static_cast<int>(box_height(tracked) * scale_y + 0.5));

    // Green means a recent detector correction; amber means prediction-only.
    if (tracker.seconds_since_match(now) <= 0.30) {
        SDL_SetRenderDrawColor(renderer, 0, 255, 80, 255);
    } else {
        SDL_SetRenderDrawColor(renderer, 255, 190, 0, 255);
    }

    for (int thickness = 0; thickness < 3; ++thickness) {
        SDL_Rect line = box;
        line.x += thickness;
        line.y += thickness;
        line.w = std::max(1, line.w - 2 * thickness);
        line.h = std::max(1, line.h - 2 * thickness);
        SDL_RenderDrawRect(renderer, &line);
    }

    const int target_x = box.x + box.w / 2;
    const int target_y = box.y + box.h / 2;
    SDL_RenderDrawLine(renderer, target_x - 7, target_y, target_x + 7, target_y);
    SDL_RenderDrawLine(renderer, target_x, target_y - 7, target_x, target_y + 7);
    SDL_RenderDrawLine(renderer, frame_center_x, frame_center_y, target_x, target_y);
}

void draw_detections(SDL_Renderer* renderer,
                     const object_detect_result_list& detections,
                     const SDL_Rect& video_rect,
                     int source_width,
                     int source_height,
                     int target_class_id)
{
    const double scale_x = static_cast<double>(video_rect.w) / source_width;
    const double scale_y = static_cast<double>(video_rect.h) / source_height;
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    for (int i = 0; i < detections.count; ++i) {
        const object_detect_result& detection = detections.results[i];
        if (detection.prop < kTargetMinConfidence) continue;

        const FloatBox detected = to_float_box(detection);
        SDL_Rect box{};
        box.x = video_rect.x + static_cast<int>(detected.left * scale_x + 0.5);
        box.y = video_rect.y + static_cast<int>(detected.top * scale_y + 0.5);
        box.w = std::max(1, static_cast<int>(box_width(detected) * scale_x + 0.5));
        box.h = std::max(1, static_cast<int>(box_height(detected) * scale_y + 0.5));

        // Candidates of the selected class are green; all other detections are cyan.
        if (detection.cls_id == target_class_id) {
            SDL_SetRenderDrawColor(renderer, 0, 255, 80, 230);
        } else {
            SDL_SetRenderDrawColor(renderer, 0, 210, 255, 210);
        }
        for (int thickness = 0; thickness < 2; ++thickness) {
            SDL_Rect line = box;
            line.x += thickness;
            line.y += thickness;
            line.w = std::max(1, line.w - 2 * thickness);
            line.h = std::max(1, line.h - 2 * thickness);
            SDL_RenderDrawRect(renderer, &line);
        }
    }
}

void capture_loop(V4L2Camera* camera,
                  LatestFrame* latest_frame,
                  std::atomic<uint64_t>* capture_count,
                  std::atomic<bool>* running)
{
    while (running->load(std::memory_order_acquire)) {
        CapturedFrame frame;
        const int capture_ret = camera->dequeue(&frame, 100);
        if (capture_ret < 0) {
            running->store(false, std::memory_order_release);
            latest_frame->condition.notify_all();
            break;
        }
        if (capture_ret == 0) {
            continue;
        }

        {
            // Copy the newest image once, then immediately return the V4L2
            // buffer. Slow SSH/X11 rendering can no longer stall capture.
            std::lock_guard<std::mutex> lock(latest_frame->mutex);
            std::memcpy(latest_frame->data.data(), frame.data, frame.bytes);
            ++latest_frame->sequence;
        }

        if (!camera->queue_buffer(frame.buffer_index)) {
            running->store(false, std::memory_order_release);
            latest_frame->condition.notify_all();
            break;
        }

        capture_count->fetch_add(1, std::memory_order_relaxed);
        latest_frame->condition.notify_all();
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
        const int ret = inference_yolov8_model(app_ctx, &source, &results);
        const auto end = std::chrono::steady_clock::now();
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(end - begin).count();

        if (ret != 0) {
            std::fprintf(stderr, "inference_yolov8_model failed: ret=%d\n", ret);
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

double apply_normalized_deadband(double value)
{
    const double magnitude = std::fabs(value);
    if (magnitude <= kNormalizedDeadband) return 0.0;
    const double scaled = (magnitude - kNormalizedDeadband) /
                          (1.0 - kNormalizedDeadband);
    return std::copysign(std::min(1.0, scaled), value);
}

double gimbal_control_error(double value)
{
    return std::max(-1.0, std::min(1.0,
        kGimbalErrorGain * apply_normalized_deadband(value)));
}

// One-dimensional constant-velocity Kalman filter. Two instances form the
// image-plane filter [x, vx] and [y, vy]. Coordinates are normalized so that
// -1/+1 correspond to the two image borders and zero is the optical center.
class KalmanAxisFilter {
public:
    void reset(double measured_position)
    {
        position_ = measured_position;
        velocity_ = 0.0;
        p00_ = 0.0025;
        p01_ = 0.0;
        p10_ = 0.0;
        p11_ = 0.25;
        initialized_ = true;
    }

    void clear()
    {
        initialized_ = false;
    }

    void predict(double dt)
    {
        if (!initialized_) return;
        dt = std::max(0.001, std::min(0.10, dt));

        position_ += velocity_ * dt;

        const double q = kKalmanAccelerationNoise * kKalmanAccelerationNoise;
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt2 * dt2;
        const double old_p00 = p00_;
        const double old_p01 = p01_;
        const double old_p10 = p10_;
        const double old_p11 = p11_;

        p00_ = old_p00 + dt * (old_p01 + old_p10) +
               dt2 * old_p11 + 0.25 * dt4 * q;
        p01_ = old_p01 + dt * old_p11 + 0.5 * dt3 * q;
        p10_ = old_p10 + dt * old_p11 + 0.5 * dt3 * q;
        p11_ = old_p11 + dt2 * q;
    }

    void update(double measured_position, double confidence)
    {
        if (!initialized_) {
            reset(measured_position);
            return;
        }

        confidence = std::max(0.0, std::min(1.0, confidence));
        const double measurement_sigma = 0.012 + 0.025 * (1.0 - confidence);
        const double measurement_variance = measurement_sigma * measurement_sigma;
        const double innovation_variance = p00_ + measurement_variance;
        if (innovation_variance <= 1e-12) return;

        const double gain_position = p00_ / innovation_variance;
        const double gain_velocity = p10_ / innovation_variance;
        const double innovation = measured_position - position_;
        position_ += gain_position * innovation;
        velocity_ += gain_velocity * innovation;

        const double old_p00 = p00_;
        const double old_p01 = p01_;
        const double old_p10 = p10_;
        const double old_p11 = p11_;
        p00_ = (1.0 - gain_position) * old_p00;
        p01_ = (1.0 - gain_position) * old_p01;
        p10_ = old_p10 - gain_velocity * old_p00;
        p11_ = old_p11 - gain_velocity * old_p01;

        // Suppress tiny numerical asymmetry in the 2x2 covariance matrix.
        const double cross = 0.5 * (p01_ + p10_);
        p01_ = cross;
        p10_ = cross;
    }

    bool initialized() const { return initialized_; }
    double position() const { return position_; }

private:
    bool initialized_ = false;
    double position_ = 0.0;
    double velocity_ = 0.0;
    double p00_ = 0.0;
    double p01_ = 0.0;
    double p10_ = 0.0;
    double p11_ = 0.0;
};

void gimbal_control_loop(GimbalUdpController* gimbal,
                         LatestDetection* latest_detection,
                         GimbalControlStatus* control_status,
                         GimbalControlState* control_state,
                         std::atomic<bool>* running,
                         int frame_width,
                         int frame_height)
{
    SingleObjectTracker tracker;
    KalmanAxisFilter kalman_x;
    KalmanAxisFilter kalman_y;
    int kalman_track_id = 0;
    uint64_t processed_inference_count = 0;
    uint64_t last_control_revision = UINT64_MAX;
    GimbalMode previous_mode = GimbalMode::Hold;
    bool hold_sent = false;
    auto previous_filter_time = std::chrono::steady_clock::now();

    while (running->load(std::memory_order_acquire)) {
        const auto loop_begin = std::chrono::steady_clock::now();
        const double filter_dt = std::max(0.001, std::min(0.10,
            std::chrono::duration<double>(loop_begin - previous_filter_time).count()));
        previous_filter_time = loop_begin;
        kalman_x.predict(filter_dt);
        kalman_y.predict(filter_dt);

        GimbalControlSnapshot control = control_state->snapshot();
        if (tracker.set_target_class(control.target_class_id)) {
            kalman_x.clear();
            kalman_y.clear();
            kalman_track_id = 0;
            processed_inference_count = 0;
        }

        object_detect_result_list detections{};
        uint64_t inference_count = 0;
        {
            std::lock_guard<std::mutex> lock(latest_detection->mutex);
            detections = latest_detection->results;
            inference_count = latest_detection->completed_count;
        }

        if (inference_count != processed_inference_count) {
            tracker.update(detections, frame_width, frame_height, loop_begin);
            processed_inference_count = inference_count;
            const bool measured_now = tracker.active() && tracker.seconds_since_match(loop_begin) <= 1e-9;
            if (measured_now) {
                const double measured_x = box_center_x(tracker.box()) / (0.5 * frame_width) - 1.0;
                const double measured_y = box_center_y(tracker.box()) / (0.5 * frame_height) - 1.0;
                if (!kalman_x.initialized() || !kalman_y.initialized() || tracker.track_id() != kalman_track_id) {
                    kalman_x.reset(measured_x);
                    kalman_y.reset(measured_y);
                    kalman_track_id = tracker.track_id();
                } else {
                    kalman_x.update(measured_x, tracker.confidence());
                    kalman_y.update(measured_y, tracker.confidence());
                }
            }
        }

        const GimbalTelemetrySnapshot telemetry = gimbal->telemetry();
        bool available = telemetry.telemetry_fresh && telemetry.command_link_ok;
        if (control.mode != GimbalMode::Hold && (!available || telemetry.watchdog_expired)) {
            control_state->hold();
            control = control_state->snapshot();
            if (!hold_sent) gimbal->hold();
            hold_sent = true;
        }

        const bool mode_changed = control.mode != previous_mode;
        double m1_command = telemetry.m1_target_deg;
        double m2_command = telemetry.m2_target_deg;
        bool target_fresh = false;

        if (control.mode == GimbalMode::Hold) {
            if (!hold_sent || mode_changed) gimbal->hold();
            hold_sent = true;
            if (mode_changed) {
                kalman_x.clear();
                kalman_y.clear();
                kalman_track_id = 0;
            }
        } else if (control.mode == GimbalMode::Manual) {
            m1_command = std::clamp(control.manual_m1_deg,
                                    control.config.m1_min_deg,
                                    control.config.m1_max_deg);
            m2_command = std::clamp(control.manual_m2_deg,
                                    control.config.m2_min_deg,
                                    control.config.m2_max_deg);
            if (mode_changed || control.revision != last_control_revision)
                gimbal->set_targets(m1_command, m2_command);
            hold_sent = false;
        } else {
            const double target_age = tracker.seconds_since_match(loop_begin);
            target_fresh = tracker.active() && kalman_x.initialized() &&
                           kalman_y.initialized() && target_age <= kGimbalTrackFreshSeconds;
            if (target_fresh) {
                const double normalized_x = gimbal_control_error(kalman_x.position());
                const double normalized_y = gimbal_control_error(kalman_y.position());
                const double freshness_scale = target_age <= kKalmanFullControlAgeSeconds ? 1.0 :
                    std::max(0.0, std::min(1.0,
                        (kGimbalTrackFreshSeconds - target_age) /
                        (kGimbalTrackFreshSeconds - kKalmanFullControlAgeSeconds)));
                const double m1_rate = freshness_scale * normalized_y *
                                       control.config.m1_tracking_speed_dps;
                const double m2_rate = -freshness_scale * normalized_x *
                                       control.config.m2_tracking_speed_dps;
                m1_command = std::clamp(telemetry.m1_position_deg +
                                        m1_rate * kGimbalCommandLeadSeconds,
                                        control.config.m1_min_deg,
                                        control.config.m1_max_deg);
                m2_command = std::clamp(telemetry.m2_position_deg +
                                        m2_rate * kGimbalCommandLeadSeconds,
                                        control.config.m2_min_deg,
                                        control.config.m2_max_deg);
                gimbal->set_targets(m1_command, m2_command);
                hold_sent = false;
            } else if (!hold_sent) {
                gimbal->hold();
                hold_sent = true;
                kalman_x.clear();
                kalman_y.clear();
                kalman_track_id = 0;
            }
        }

        {
            std::lock_guard<std::mutex> lock(control_status->mutex);
            control_status->available = available;
            control_status->enabled = control.mode == GimbalMode::Tracking;
            control_status->target_fresh = target_fresh;
            control_status->mode = control.mode;
            control_status->m1_command_deg = m1_command;
            control_status->m2_command_deg = m2_command;
            control_status->telemetry_age_ms = telemetry.telemetry_age_ms;
        }

        previous_mode = control.mode;
        last_control_revision = control.revision;
        const auto elapsed = std::chrono::steady_clock::now() - loop_begin;
        const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(kGimbalControlPeriodSeconds));
        if (elapsed < period) std::this_thread::sleep_for(period - elapsed);
    }

    gimbal->disarm();
}

}  // namespace

int main(int argc, char** argv)
{
    const std::string app_directory = executable_directory();
    std::string model_path = app_directory + "/model/yolov8n_rk3588_int8.rknn";
    std::string labels_path = app_directory + "/model/coco_80_labels_list.txt";
    std::string video_device = "/dev/video11";
    std::string bind_ip = "192.168.10.1";
    std::string stm32_ip = "192.168.10.2";
    uint16_t command_port = 5000;
    uint16_t telemetry_port = 5001;
    bool enable_gimbal = true;
    for (int i = 1; i < argc; ++i) {
        const std::string option = argv[i];
        if (option == "--model" && i + 1 < argc) model_path = argv[++i];
        else if (option == "--labels" && i + 1 < argc) labels_path = argv[++i];
        else if (option == "--camera" && i + 1 < argc) video_device = argv[++i];
        else if (option == "--bind-ip" && i + 1 < argc) bind_ip = argv[++i];
        else if (option == "--stm32-ip" && i + 1 < argc) stm32_ip = argv[++i];
        else if (option == "--command-port" && i + 1 < argc &&
                 parse_port(argv[i + 1], &command_port)) ++i;
        else if (option == "--telemetry-port" && i + 1 < argc &&
                 parse_port(argv[i + 1], &telemetry_port)) ++i;
        else if (option == "--no-gimbal") enable_gimbal = false;
        else if (option == "--help") { print_usage(argv[0]); return EXIT_SUCCESS; }
        else {
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }
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
    std::atomic<uint64_t> capture_count{0};
    std::thread capture_thread;
    std::thread inference_thread;
    std::thread gimbal_control_thread;
    LatestFrame latest_frame;
    LatestDetection latest_detection;
    GimbalControlStatus gimbal_status;
    V4L2Camera camera;
    GimbalControlState gimbal_control;
    GimbalUdpController gimbal(bind_ip, stm32_ip, command_port, telemetry_port);
    GimbalCommandConsole console(gimbal_control, gimbal, running);
    bool gimbal_started = false;

    if (!readable_file(labels_path)) {
        std::fprintf(stderr, "labels file is missing or unreadable: %s\n", labels_path.c_str());
        goto cleanup;
    }
    if (!readable_file(model_path)) {
        std::fprintf(stderr,
                     "RKNN model is missing or unreadable: %s\n"
                     "Copy yolov8n_rk3588_int8.rknn to the app/model directory, then run ./app again.\n",
                     model_path.c_str());
        goto cleanup;
    }

    if (init_post_process(labels_path.c_str()) != 0) {
        std::fprintf(stderr, "init_post_process failed: %s\n", labels_path.c_str());
        goto cleanup;
    }
    postprocess_initialized = true;

    if (init_yolov8_model(model_path.c_str(), &app_ctx) != 0) {
        std::fprintf(stderr, "init_yolov8_model failed: %s\n", model_path.c_str());
        goto cleanup;
    }
    model_initialized = true;

    if (!camera.open_device(video_device.c_str(), kRequestedWidth, kRequestedHeight)) {
        goto cleanup;
    }

    latest_frame.data.resize(camera.frame_bytes());

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        goto cleanup;
    }

    window = SDL_CreateWindow("YOLOv8n INT8 Object Tracker",
                              SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                              kInitialWindowWidth, kInitialWindowHeight,
                              SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    if (!window) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        goto cleanup;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
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

    gimbal_started = enable_gimbal && gimbal.start();
    if (enable_gimbal && !gimbal_started) {
        std::fprintf(stderr,
                     "gimbal UDP unavailable: %s\n"
                     "Vision remains active. Check the Ethernet interface/IP, then restart app.\n",
                     gimbal.last_error().c_str());
    }

    running.store(true, std::memory_order_release);
    std::fprintf(stderr,
                 "\nApplication ready: camera=%s, gimbal=%s\n"
                 "UDP: %s:%u -> %s:%u, initial mode=HOLD, target=car(2)\n"
                 "Detection boxes are displayed in the SDL window; commands are accepted below.\n",
                 video_device.c_str(), gimbal_started ? "CONNECTED" : "OFFLINE",
                 bind_ip.c_str(), telemetry_port, stm32_ip.c_str(), command_port);
    console.start();
    capture_thread = std::thread(capture_loop, &camera, &latest_frame,
                                 &capture_count, &running);
    inference_thread = std::thread(inference_loop, &app_ctx,
                                   &latest_frame, &latest_detection, &running,
                                   camera.width(), camera.height(), camera.stride());
    if (gimbal_started) {
        gimbal_control_thread = std::thread(
            gimbal_control_loop, &gimbal, &latest_detection, &gimbal_status,
            &gimbal_control, &running, camera.width(), camera.height());
    }

    {
        SingleObjectTracker tracker;
        std::vector<uint8_t> display_frame(camera.frame_bytes());
        uint64_t display_sequence = 0;
        uint64_t display_count = 0;
        uint64_t last_capture_count = 0;
        uint64_t last_display_count = 0;
        uint64_t last_inference_count = 0;
        uint64_t processed_inference_count = 0;
        auto last_title_time = std::chrono::steady_clock::now();
        auto last_frame_time = last_title_time;

        while (!g_stop_requested && running.load(std::memory_order_acquire)) {
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                if (event.type == SDL_QUIT ||
                    (event.type == SDL_KEYDOWN &&
                     (event.key.keysym.sym == SDLK_ESCAPE ||
                      event.key.keysym.sym == SDLK_q))) {
                    running.store(false, std::memory_order_release);
                } else if (event.type == SDL_KEYDOWN &&
                           event.key.keysym.sym == SDLK_g && gimbal_started) {
                    const GimbalControlSnapshot state = gimbal_control.snapshot();
                    if (state.mode == GimbalMode::Tracking) {
                        gimbal_control.hold();
                        std::fprintf(stderr, "[GIMBAL] mode=HOLD\n");
                    } else {
                        gimbal_control.track();
                        const GimbalControlSnapshot selected = gimbal_control.snapshot();
                        std::fprintf(stderr, "[GIMBAL] mode=TRACK target=%s(%d)\n",
                                     coco_class_name(selected.target_class_id), selected.target_class_id);
                    }
                } else if (event.type == SDL_KEYDOWN &&
                           event.key.keysym.sym == SDLK_h && gimbal_started) {
                    gimbal_control.hold();
                    std::fprintf(stderr, "[GIMBAL] mode=HOLD\n");
                }
            }
            if (!running.load(std::memory_order_acquire)) {
                break;
            }

            bool have_new_display_frame = false;
            {
                std::lock_guard<std::mutex> lock(latest_frame.mutex);
                if (latest_frame.sequence != display_sequence) {
                    display_frame = latest_frame.data;
                    display_sequence = latest_frame.sequence;
                    have_new_display_frame = true;
                }
            }

            if (!have_new_display_frame) {
                SDL_Delay(1);
                continue;
            }

            const uint8_t* y_plane = display_frame.data();
            const uint8_t* uv_plane =
                display_frame.data() +
                static_cast<size_t>(camera.stride()) * camera.height();
            if (SDL_UpdateNVTexture(texture, nullptr,
                                    y_plane, camera.stride(),
                                    uv_plane, camera.stride()) != 0) {
                std::fprintf(stderr, "SDL_UpdateNVTexture failed: %s\n", SDL_GetError());
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

            const auto now = std::chrono::steady_clock::now();
            const GimbalControlSnapshot selected_target = gimbal_control.snapshot();
            tracker.set_target_class(selected_target.target_class_id);
            const double frame_dt =
                std::chrono::duration<double>(now - last_frame_time).count();
            last_frame_time = now;
            tracker.predict(frame_dt, camera.width(), camera.height());
            if (inference_count != processed_inference_count) {
                tracker.update(detections, camera.width(), camera.height(), now);
                processed_inference_count = inference_count;
            }

            int output_width = 0;
            int output_height = 0;
            SDL_GetRendererOutputSize(renderer, &output_width, &output_height);
            const SDL_Rect video_rect = aspect_fit_rect(
                camera.width(), camera.height(), output_width, output_height);

            SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
            SDL_RenderClear(renderer);
            SDL_RenderCopy(renderer, texture, nullptr, &video_rect);
            draw_detections(renderer, detections, video_rect,
                            camera.width(), camera.height(), selected_target.target_class_id);
            draw_tracker(renderer, tracker, video_rect,
                         camera.width(), camera.height(), now);
            SDL_RenderPresent(renderer);
            ++display_count;

            const double title_interval =
                std::chrono::duration<double>(now - last_title_time).count();
            if (title_interval >= 1.0) {
                const uint64_t capture_total =
                    capture_count.load(std::memory_order_relaxed);
                const double capture_fps =
                    static_cast<double>(capture_total - last_capture_count) / title_interval;
                const double display_fps =
                    static_cast<double>(display_count - last_display_count) / title_interval;
                const double inference_fps =
                    static_cast<double>(inference_count - last_inference_count) / title_interval;

                bool gimbal_available = false;
                bool gimbal_auto = false;
                bool gimbal_target_fresh = false;
                GimbalMode current_gimbal_mode = GimbalMode::Hold;
                double gimbal_m1 = 0.0;
                double gimbal_m2 = 0.0;
                {
                    std::lock_guard<std::mutex> lock(gimbal_status.mutex);
                    gimbal_available = gimbal_status.available;
                    gimbal_auto = gimbal_status.enabled;
                    gimbal_target_fresh = gimbal_status.target_fresh;
                    current_gimbal_mode = gimbal_status.mode;
                    gimbal_m1 = gimbal_status.m1_command_deg;
                    gimbal_m2 = gimbal_status.m2_command_deg;
                }
                const char* gimbal_mode = "OFFLINE";
                if (gimbal_started && !gimbal_available) gimbal_mode = "NO-TEL";
                else if (gimbal_started && current_gimbal_mode == GimbalMode::Manual) gimbal_mode = "MANUAL";
                else if (gimbal_started && !gimbal_auto) gimbal_mode = "HOLD";
                else if (gimbal_started) gimbal_mode = gimbal_target_fresh ? "AUTO" : "WAIT";

                char title[320];
                if (tracker.active()) {
                    const int error_x = static_cast<int>(box_center_x(tracker.box()) -
                                                         camera.width() * 0.5F);
                    const int error_y = static_cast<int>(box_center_y(tracker.box()) -
                                                         camera.height() * 0.5F);
                    std::snprintf(title, sizeof(title),
                                  "Object Track [%s] | cap %.1f display %.1f det %.1f / %.1fms | ID%d %.2f ex%+d ey%+d | G:%s M1%.1f M2%.1f",
                                  coco_class_name(selected_target.target_class_id),
                                  capture_fps, display_fps, inference_fps, inference_ms,
                                  tracker.track_id(), tracker.confidence(),
                                  error_x, error_y, gimbal_mode, gimbal_m1, gimbal_m2);
                } else {
                    std::snprintf(title, sizeof(title),
                                  "Object Track [%s] | cap %.1f display %.1f det %.1f / %.1fms | SEARCHING | G:%s M1%.1f M2%.1f",
                                  coco_class_name(selected_target.target_class_id),
                                  capture_fps, display_fps, inference_fps, inference_ms,
                                  gimbal_mode, gimbal_m1, gimbal_m2);
                }
                SDL_SetWindowTitle(window, title);

                last_capture_count = capture_total;
                last_display_count = display_count;
                last_inference_count = inference_count;
                last_title_time = now;
            }
        }
    }

    exit_code = EXIT_SUCCESS;

cleanup:
    running.store(false, std::memory_order_release);
    gimbal_control.hold();
    console.stop();
    latest_frame.condition.notify_all();
    if (capture_thread.joinable()) {
        capture_thread.join();
    }
    if (inference_thread.joinable()) {
        inference_thread.join();
    }
    if (gimbal_control_thread.joinable()) {
        gimbal_control_thread.join();
    }
    if (gimbal_started) {
        gimbal.stop();
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
        release_yolov8_model(&app_ctx);
    }
    if (postprocess_initialized) {
        deinit_post_process();
    }
    return exit_code;
}
