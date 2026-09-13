

https://github.com/user-attachments/assets/bfa60f73-d565-4e5e-9dd8-d5f60e244cc4

在RK3588上完成车辆检测和双轴云台跟踪。摄像头图像通过V4L2采集，检测模型运行在RKNPU上，Linux端根据目标位置生成云台角度指令，并通过UDP与STM32G4控制板通信。
只检测COCO数据集中的车辆类别：
摄像头：GC4653，1280×720 NV12，30 FPS
推理：RKNPU2，YOLOv8n INT8
显示：SDL2
跟踪：IoU和中心距离关联，卡尔曼滤波预测
通信：UDP非阻塞socket和poll
云台

https://github.com/user-attachments/assets/1ee3c13f-abe4-4fd1-a326-c74fd1417255

控制：STM32G431CBT6双轴FOC



数据流程：GC4653 → V4L2采集 → RKNN推理 → 目标关联/卡尔曼预测 → 云台控制量 → UDP → STM32G4 → 双轴电机

摄像头输出为30 FPS，YOLOv8n INT8单次推理约20 ms。云台控制周期为30 Hz，STM32遥测频率为100 Hz。通过SSH X11转发显示时，帧率会受到软件渲染和网络传输影响，性能测试应以板卡本地显示结果为准。

目录：
G431_DualAxis_CurrentFOC/    STM32G4 FOC工程
apps/                        实时检测与跟踪程序
gimbal/                      UDP云台通信模块
rknn/                        RT-DETR和YOLOv8 RKNN接口
tools/                       ONNX到RKNN转换脚本
configs/                     RKAiq 3A服务配置
docs/                        部署、调试和算法笔记
models/                      模型放置说明

板卡环境
安装编译和显示依赖：
sudo apt update
sudo apt install -y build-essential cmake pkg-config libsdl2-dev v4l-utils

确认摄像头节点：

v4l2-ctl -d /dev/video11 --all

v4l2-ctl -d /dev/video11 \
  --set-fmt-video=width=1280,height=720,pixelformat=NV12 \
  --stream-mmap=4 \
  --stream-poll \
  --stream-count=300 \
  --stream-to=/dev/null

启动ISP自动曝光服务：
sudo systemctl enable --now rkaiq-3a.service
systemctl --no-pager status rkaiq-3a.service

注意rkaiq_3A_server只能运行一个实例。多个实例会同时访问ISP并造成曝光或设备占用异常

云台网络配置，默认网络参数：
RK地址：  192.168.10.1
STM32地址：   192.168.10.2
指令端口：    5000
遥测端口：    5001

配置板卡网口：
sudo ip link set eth1 up
sudo ip addr replace 192.168.10.1/24 dev eth1
sudo ip route replace 192.168.10.0/24 dev eth1 src 192.168.10.1

检查地址和路由：
ip -br -4 addr show eth1
ip route get 192.168.10.2
ping -c 3 192.168.10.2

运行视觉程序前，应退出独立的gimbal_udp_client_v3，避免两个程序同时绑定192.168.10.1:5001。

编译，在YOLOv8 C++目录执行：
cd ~/lubancat_ai_manual_code/example/yolov8/cpp
./build-linux.sh -t rk3588

生成文件位于：install/rk3588_linux/yolov8_vehicle_tracker_demo

模型文件放到：install/rk3588_linux/model/yolov8n_rk3588_int8.rknn

运行，在板卡本地显示器运行：
cd ~/lubancat_ai_manual_code/example/yolov8/cpp/install/rk3588_linux

unset LD_LIBRARY_PATH
export DISPLAY=:0

./yolov8_vehicle_tracker_demo \
  ./model/yolov8n_rk3588_int8.rknn \
  /dev/video11

程序启动后默认关闭云台自动控制，确认画面和遥测正常后再手动开启。


G       开启或关闭云台自动跟踪
H       保持云台当前位置
Q/ESC   退出程序


安全角度
M1俯仰：-70°～+55°

M2航向：-110°～+110°

STM32G4工程主要包括：
TIM1/TIM8中心对齐三相PWM
ADC同步电流采样
Clarke/Park变换
电流、速度和位置控制
AS5600双轴角度反馈
UDP指令接收与100 Hz状态遥测



说明
实时跟踪使用YOLOv8n INT8，以降低检测延迟。模型文件未直接提交到仓库，转换方法和部署过程比较简单

部署RT-DETR时比较复杂，可能会遇到不支持的算子：Unsupport CPU op: GridSample in this librknnrt.so
导出部署版本，尽量从网络图中消除或替换不支持算子，后处理尽量交给c++实现，自定义算子适合TopK这类数据量有限的操作，不适合把大规模`GridSample`全部丢给CPU，否则即使能运行，实时性也可能完全失去。

完整的RT-DETRv2-S经过混合量化通过完整NPU单元延迟优化到了90ms左右

