// Real-time YOLOv8n INT8 single-vehicle tracker for RK3588.
// Detection runs asynchronously; a constant-velocity tracker updates the box
// on every camera frame. No OpenCV or external tracking library is required.

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
#include "gimbal_udp_controller.h"

namespace {

constexpr int kRequestedWidth = 1280;
constexpr int kRequestedHeight = 720;
constexpr int kRequestedBufferCount = 4;
constexpr int kInitialWindowWidth = 960;
constexpr int kInitialWindowHeight = 540;

// COCO zero-based class IDs used by the bundled labels/model.
constexpr int kClassCar = 2;
constexpr int kClassMotorcycle = 3;
constexpr int kClassBus = 5;
constexpr int kClassTruck = 7;
constexpr float kVehicleMinConfidence = 0.25F;
constexpr double kTrackHoldSeconds = 1.20;

// Gimbal convention supplied by the hardware setup:
// M1 pitch: negative=up, positive=down, limits [-70, +55] degrees.
// M2 yaw:   positive=left, negative=right, limits [-110, +110] degrees.
constexpr double kM1MinimumDeg = -70.0;
constexpr double kM1MaximumDeg = 55.0;
constexpr double kM2MinimumDeg = -110.0;
constexpr double kM2MaximumDeg = 110.0;
constexpr double kGimbalControlPeriodSeconds = 1.0 / 30.0;
constexpr double kGimbalTelemetryTimeoutMs = 300.0;
constexpr double kGimbalTrackFreshSeconds = 0.30;
constexpr double kNormalizedDeadband = 0.02;
constexpr double kGimbalErrorGain = 2.0;
constexpr double kGimbalCommandLeadSeconds = 0.10;
constexpr double kM1MaximumRateDegPerSecond = 40.0;
constexpr double kM2MaximumRateDegPerSecond = 50.0;

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

struct GimbalControlStatus {
    std::mutex mutex;
    bool available = false;
    bool enabled = false;
    bool target_fresh = false;
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

bool is_vehicle_class(int class_id)
{
    return class_id == kClassCar || class_id == kClassMotorcycle ||
           class_id == kClassBus || class_id == kClassTruck;
}

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

class SingleVehicleTracker {
public:
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
                if (!is_vehicle_class(detection.cls_id) ||
                    detection.prop < kVehicleMinConfidence) {
                    continue;
                }

                const FloatBox candidate = to_float_box(detection);
                const float overlap = box_iou(box_, candidate);
                const float dx = box_center_x(candidate) - box_center_x(box_);
                const float dy = box_center_y(candidate) - box_center_y(box_);
                const float normalized_distance = std::sqrt(dx * dx + dy * dy) / diagonal;

                // Reject unrelated vehicles. The distance gate keeps association
                // working when fast motion makes IoU temporarily small.
                if (overlap < 0.05F && normalized_distance > 0.18F) {
                    continue;
                }
                const float class_bonus = detection.cls_id == class_id_ ? 0.10F : 0.0F;
                const float association_score = 1.8F * overlap - normalized_distance +
                                                0.35F * detection.prop + class_bonus;
                if (association_score > best_score) {
                    best_score = association_score;
                    matched_index = i;
                }
            }
        } else {
            // Initial target: confidence first, box area second. This avoids
            // locking a tiny distant false positive when a clear vehicle exists.
            float best_score = -1.0F;
            const float frame_area = static_cast<float>(frame_width * frame_height);
            for (int i = 0; i < detections.count; ++i) {
                const object_detect_result& detection = detections.results[i];
                if (!is_vehicle_class(detection.cls_id) ||
                    detection.prop < kVehicleMinConfidence) {
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
                  const SingleVehicleTracker& tracker,
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

void gimbal_control_loop(GimbalUdpController* gimbal,
                         LatestDetection* latest_detection,
                         GimbalControlStatus* control_status,
                         std::atomic<bool>* enabled,
                         std::atomic<bool>* running,
                         int frame_width,
                         int frame_height)
{
    SingleVehicleTracker tracker;
    uint64_t processed_inference_count = 0;
    bool hold_sent = false;

    while (running->load(std::memory_order_acquire)) {
        const auto loop_begin = std::chrono::steady_clock::now();

        object_detect_result_list detections{};
        uint64_t inference_count = 0;
        {
            std::lock_guard<std::mutex> lock(latest_detection->mutex);
            detections = latest_detection->results;
            inference_count = latest_detection->completed_count;
        }

        // The display tracker may extrapolate for smooth boxes, but gimbal
        // control uses detector-corrected positions only. Extrapolating image
        // velocity while the camera itself is moving causes late braking and
        // overshoot.
        if (inference_count != processed_inference_count) {
            tracker.update(detections, frame_width, frame_height, loop_begin);
            processed_inference_count = inference_count;
        }

        const GimbalTelemetrySnapshot telemetry = gimbal->telemetry();
        const bool available = telemetry.valid &&
                               telemetry.age_ms <= kGimbalTelemetryTimeoutMs;
        bool control_enabled = enabled->load(std::memory_order_acquire);
        double m1_command = telemetry.m1_target_deg;
        double m2_command = telemetry.m2_target_deg;
        bool target_fresh = false;

        if (control_enabled && !available) {
            enabled->store(false, std::memory_order_release);
            control_enabled = false;
            if (!hold_sent) {
                gimbal->hold();
                hold_sent = true;
            }
        }

        if (control_enabled && available) {
            target_fresh = tracker.active() &&
                tracker.seconds_since_match(loop_begin) <= kGimbalTrackFreshSeconds;
            if (target_fresh) {
                const double normalized_x = gimbal_control_error(
                    box_center_x(tracker.box()) / (0.5 * frame_width) - 1.0);
                const double normalized_y = gimbal_control_error(
                    box_center_y(tracker.box()) / (0.5 * frame_height) - 1.0);

                // Image down -> M1 positive (pitch down).
                // Image right -> M2 negative (yaw right).
                const double m1_rate =
                    normalized_y * kM1MaximumRateDegPerSecond;
                const double m2_rate =
                    -normalized_x * kM2MaximumRateDegPerSecond;

                // Use a short position lead from the measured motor angle.
                // The old implementation integrated rate into the previous
                // target, allowing a large target lead to build up. When the
                // image reached center the motor still chased that stale lead,
                // producing visible overshoot. Here the lead collapses to zero
                // immediately when the visual error becomes zero.
                m1_command = std::max(
                    kM1MinimumDeg,
                    std::min(kM1MaximumDeg,
                             telemetry.m1_position_deg +
                                 m1_rate * kGimbalCommandLeadSeconds));
                m2_command = std::max(
                    kM2MinimumDeg,
                    std::min(kM2MaximumDeg,
                             telemetry.m2_position_deg +
                                 m2_rate * kGimbalCommandLeadSeconds));
                gimbal->set_targets(m1_command, m2_command);
                hold_sent = false;
            } else if (!hold_sent) {
                gimbal->hold();
                hold_sent = true;
            }
        }

        {
            std::lock_guard<std::mutex> lock(control_status->mutex);
            control_status->available = available;
            control_status->enabled = control_enabled;
            control_status->target_fresh = target_fresh;
            control_status->m1_command_deg = m1_command;
            control_status->m2_command_deg = m2_command;
            control_status->telemetry_age_ms = telemetry.age_ms;
        }

        const auto elapsed = std::chrono::steady_clock::now() - loop_begin;
        const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(kGimbalControlPeriodSeconds));
        if (elapsed < period) std::this_thread::sleep_for(period - elapsed);
    }

    gimbal->hold();
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
    std::atomic<uint64_t> capture_count{0};
    std::atomic<bool> gimbal_enabled{false};
    std::thread capture_thread;
    std::thread inference_thread;
    std::thread gimbal_control_thread;
    LatestFrame latest_frame;
    LatestDetection latest_detection;
    GimbalControlStatus gimbal_status;
    V4L2Camera camera;
    GimbalUdpController gimbal;
    bool gimbal_started = false;

    if (init_post_process() != 0) {
        std::fprintf(stderr, "init_post_process failed\n");
        goto cleanup;
    }
    postprocess_initialized = true;

    if (init_yolov8_model(model_path, &app_ctx) != 0) {
        std::fprintf(stderr, "init_yolov8_model failed: %s\n", model_path);
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

    window = SDL_CreateWindow("YOLOv8n INT8 Vehicle Tracker",
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

    gimbal_started = gimbal.start();
    if (!gimbal_started) {
        std::fprintf(stderr,
                     "gimbal UDP unavailable: %s\n"
                     "Stop gimbal_udp_client_v3 and verify 192.168.10.2 routes via eth1.\n",
                     gimbal.last_error().c_str());
    }

    running.store(true, std::memory_order_release);
    capture_thread = std::thread(capture_loop, &camera, &latest_frame,
                                 &capture_count, &running);
    inference_thread = std::thread(inference_loop, &app_ctx,
                                   &latest_frame, &latest_detection, &running,
                                   camera.width(), camera.height(), camera.stride());
    if (gimbal_started) {
        gimbal_control_thread = std::thread(
            gimbal_control_loop, &gimbal, &latest_detection, &gimbal_status,
            &gimbal_enabled, &running, camera.width(), camera.height());
    }

    std::printf("controls: G=enable/disable auto gimbal, H=hold, ESC/Q=quit\n");
    std::printf("pipeline: independent capture + inference + display threads\n");
    std::printf("gimbal starts DISABLED; M1 pitch [-70,+55], M2 yaw [-110,+110]\n");
    std::printf("vehicle classes: car(2), motorcycle(3), bus(5), truck(7), min_conf=%.2f\n",
                kVehicleMinConfidence);

    {
        SingleVehicleTracker tracker;
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
                    const bool requested =
                        !gimbal_enabled.load(std::memory_order_acquire);
                    if (requested) {
                        const GimbalTelemetrySnapshot telemetry = gimbal.telemetry();
                        if (!telemetry.valid || telemetry.age_ms > 300) {
                            std::fprintf(stderr,
                                         "cannot enable gimbal: telemetry unavailable/stale\n");
                        } else {
                            gimbal_enabled.store(true, std::memory_order_release);
                            std::printf("[GIMBAL] automatic tracking ENABLED\n");
                        }
                    } else {
                        gimbal_enabled.store(false, std::memory_order_release);
                        gimbal.hold();
                        std::printf("[GIMBAL] automatic tracking DISABLED, HOLD\n");
                    }
                } else if (event.type == SDL_KEYDOWN &&
                           event.key.keysym.sym == SDLK_h && gimbal_started) {
                    gimbal_enabled.store(false, std::memory_order_release);
                    gimbal.hold();
                    std::printf("[GIMBAL] HOLD\n");
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
                double gimbal_m1 = 0.0;
                double gimbal_m2 = 0.0;
                {
                    std::lock_guard<std::mutex> lock(gimbal_status.mutex);
                    gimbal_available = gimbal_status.available;
                    gimbal_auto = gimbal_status.enabled;
                    gimbal_target_fresh = gimbal_status.target_fresh;
                    gimbal_m1 = gimbal_status.m1_command_deg;
                    gimbal_m2 = gimbal_status.m2_command_deg;
                }
                const char* gimbal_mode = !gimbal_started ? "OFFLINE" :
                    (!gimbal_available ? "NO-TEL" :
                     (!gimbal_auto ? "HOLD" :
                      (gimbal_target_fresh ? "AUTO" : "WAIT")));

                char title[320];
                if (tracker.active()) {
                    const int error_x = static_cast<int>(box_center_x(tracker.box()) -
                                                         camera.width() * 0.5F);
                    const int error_y = static_cast<int>(box_center_y(tracker.box()) -
                                                         camera.height() * 0.5F);
                    std::snprintf(title, sizeof(title),
                                  "Vehicle Track | cap %.1f display %.1f det %.1f / %.1fms | ID%d cls%d %.2f ex%+d ey%+d | G:%s M1%.1f M2%.1f",
                                  capture_fps, display_fps, inference_fps, inference_ms,
                                  tracker.track_id(), tracker.class_id(), tracker.confidence(),
                                  error_x, error_y, gimbal_mode, gimbal_m1, gimbal_m2);
                    std::printf("TRACK id=%d class=%d conf=%.3f center=(%.1f,%.1f) error=(%+d,%+d) age=%.2fs\n",
                                tracker.track_id(), tracker.class_id(), tracker.confidence(),
                                box_center_x(tracker.box()), box_center_y(tracker.box()),
                                error_x, error_y, tracker.seconds_since_match(now));
                } else {
                    std::snprintf(title, sizeof(title),
                                  "Vehicle Track | cap %.1f display %.1f det %.1f / %.1fms | SEARCHING | G:%s M1%.1f M2%.1f",
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
    gimbal_enabled.store(false, std::memory_order_release);
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
