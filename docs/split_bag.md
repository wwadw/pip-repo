将 ROS bag （包含图片和点云）拆分为单帧图片与点云输出。



[split_bag-0.1.0-py3-none-any.zip](https://www.yuque.com/attachments/yuque/0/2026/zip/38905876/1769509612435-35fb9d53-a8c0-4fb0-8b78-d96d023ecf51.zip)

## 安装
```cpp
pip install split_bag-0.1.0-py3-none-any.whl
```

## 使用
```cpp
split_bag --input /path/to/bags \
    --img-output ./images \
    --pcd-output ./pointclouds \
    --image-topic /usb_cam/image_raw \
    --pointcloud-topic /livox/lidar_192_168_123_100    
```

参数：

+ `--input` bag包路径
+ `--img-output` 输出图片路径
+ `--pcd-output`输出点云路径
+ `--image-topic`（默认：`/usb_cam/image_raw`）
+ `--pointcloud-topic`（默认：`/livox/lidar_192_168_123_100`）

