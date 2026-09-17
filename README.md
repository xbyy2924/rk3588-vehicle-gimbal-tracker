

G431_DualAxis_CurrentFOC/          STM32G431云台控制程序，云台控制运行在RTOS
linux/rk3588_gimbal_linux_client/  RK3588目标检测与云台客户端


 Linux端依赖

sudo apt update
sudo apt install -y build-essential cmake libsdl2-dev


RKNN运行库已经放在工程的 `third_party` 目录。

模型文件不上传到Git，需要手动放到：
linux/rk3588_gimbal_linux_client/model/yolov8n_rk3588_int8.rknn


 编译
cd linux/rk3588_gimbal_linux_client
chmod +x build-linux.sh
./build-linux.sh -t rk3588 -b Release


也可以直接指定已有模型：

GIMBAL_MODEL_PATH=~/work/yuntai/model/yolov8n_rk3588_int8.rknn \
./build-linux.sh -t rk3588 -b Release


运行
cd install/rk3588_linux
./app


默认配置：
Camera:        /dev/video11
Linux IP:      192.168.10.1
STM32 IP:      192.168.10.2
Command Port:  5000
Telemetry Port: 5001


客户端命令
mode track car       跟踪汽车
mode track person    跟踪人员
mode track bottle    跟踪瓶子
classes              查看模型支持的类别
track off            关闭跟踪并保持当前位置
speed 70             设置两轴跟踪速度
speed m1 60          设置M1跟踪速度
speed m2 90          设置M2跟踪速度
range m1 -70 55      设置M1角度范围
range m2 -110 110    设置M2角度范围
manual 10 -20        手动设置两轴角度
hold                  保持当前位置
status                查看云台和UDP状态
disarm                退出远程控制
exit                  退出程序


默认以 car作为跟踪类别。目标类别支持COCO 80类，可以使用名称或类别ID切换。

历史版本

最初版本保存在Git标签：
legacy-v1-before-board-client
