#pragma once

#include <cctype>
#include <cstdlib>
#include <string>

inline constexpr int kCocoClassCount = 80;
inline constexpr const char* kCocoClassNames[kCocoClassCount] = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed",
    "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
    "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
};

inline const char* coco_class_name(int class_id)
{
    return class_id >= 0 && class_id < kCocoClassCount ? kCocoClassNames[class_id] : "unknown";
}

inline std::string normalize_coco_class(std::string text)
{
    size_t begin = 0;
    while (begin < text.size() && std::isspace(static_cast<unsigned char>(text[begin]))) ++begin;
    size_t end = text.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1]))) --end;
    text = text.substr(begin, end - begin);

    std::string normalized;
    bool previous_space = false;
    for (char ch : text) {
        unsigned char value = static_cast<unsigned char>(ch);
        if (ch == '_' || ch == '-' || std::isspace(value)) {
            if (!normalized.empty() && !previous_space) normalized.push_back(' ');
            previous_space = true;
        } else {
            normalized.push_back(static_cast<char>(std::tolower(value)));
            previous_space = false;
        }
    }
    if (!normalized.empty() && normalized.back() == ' ') normalized.pop_back();
    return normalized;
}

inline int coco_class_id_from_text(const std::string& text)
{
    std::string name = normalize_coco_class(text);
    if (name.empty()) return -1;

    char* end = nullptr;
    const long numeric_id = std::strtol(name.c_str(), &end, 10);
    if (end && *end == '\0' && numeric_id >= 0 && numeric_id < kCocoClassCount) return static_cast<int>(numeric_id);

    if (name == "human") name = "person";
    else if (name == "motorbike") name = "motorcycle";
    else if (name == "aeroplane") name = "airplane";
    else if (name == "sofa") name = "couch";
    else if (name == "tvmonitor" || name == "television") name = "tv";
    else if (name == "phone" || name == "mobile phone") name = "cell phone";

    for (int i = 0; i < kCocoClassCount; ++i) {
        if (name == kCocoClassNames[i]) return i;
    }
    return -1;
}

