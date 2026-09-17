# 模型文件

历史 GitHub 备份没有包含 RKNN 模型。把下面的文件放在本目录：

`yolov8n_rk3588_int8.rknn`

也可以在编译时指定已有模型：

```bash
GIMBAL_MODEL_PATH=/你的路径/yolov8n_rk3588_int8.rknn ./build-linux.sh -t rk3588 -b Release
```

构建脚本只会在目标文件不存在时复制模型，绝不会删除或覆盖已有模型。

