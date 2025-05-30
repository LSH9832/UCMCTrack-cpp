# UCMCTrack的c++部署

![demo](https://github.com/LSH9832/UCMCTrack-cpp/assets/69880398/f732ec98-8449-4683-8f1a-a57cc9233416)

## 在使用之前

用之前请先阅读[原论文](https://arxiv.org/abs/2312.08952)！

用之前请先阅读[原论文](https://arxiv.org/abs/2312.08952)！

用之前请先阅读[原论文](https://arxiv.org/abs/2312.08952)！

**不要在完全不了解算法原理的情况下使用本项目，效果不好概不负责，谢谢！**

## 致谢

首先感谢大佬[corfyi](https://github.com/corfyi)将此多目标跟踪算法开源，但在[issues](https://github.com/corfyi/UCMCTrack/issues/18)中看出大佬是有c++版本的代码的，却并没有开源。本人水平有限，工作之余花费4天时间将原项目（[UCMCTrack](https://github.com/corfyi/UCMCTrack)）由python转为c++代码，难免出错，如发现bug或算法逻辑不对的地方欢迎在issues中提出。


## C++真是太快啦！

实测python版本的代码FPS不到100Hz, 作者在原文中提到纯跟踪情况下他代码的C++版FPS能达到1000+FPS，如果我的代码没写错的话，在demo.mp4中貌似可以来到3000FPS左右哦，见下图（CPU型号为I5-11320H）,和YOLO神经网络一起运行的时候速度更快（由于GPU推理延时使CPU负载反而变低了）

![ucmcspeed](https://github.com/LSH9832/UCMCTrack-cpp/assets/69880398/0594d9c0-ef21-492e-ae78-ac267ba9be19)

即使在arm平台（香橙派，RK3588芯片）速度也基本没差多少（CPU利用率10%以下），但和神经网络一起跑的时候速度有明显减慢（RKNN不仅让NPU有负载，CPU也有），CPU平均利用率45%时延迟约8毫秒，

![ucmcspeedrk3588](https://github.com/LSH9832/UCMCTrack-cpp/assets/69880398/7943d9ec-0f1a-447b-80d7-a8d8ba2b8820)


## 依赖库

- libopencv-dev
- libeigen3-dev
- libyaml-cpp0.6


## 注意

- 本项目不提供目标检测部分的代码，示例代码将原作者提供的demo.mp4中的检测结果写在demo/bboxes.yaml文件中了，参数写在demo/config.yaml中，可根据自己的需求改参数，并加入检测部分的代码以替换文件中的检测结果。文件中的检测结果使用的是本人在VisDrone-DET-2019上训练的模型，详见我的另一个项目[edgeyolo](https://github.com/LSH9832/edgeyolo)。其中检测结果的框架也是采用该项目提供的cpp源码。

- include/image_utils/lap中的代码来自[gatagat/lap](https://github.com/gatagat/lap)，也就是python中lap库源码。


## Setup
根据自己的需求改CMakeLists.txt，然后
```shell
mkdir build && cd build
cmake ..
make
```
## 使用Demo
进入build文件夹，然后
```shell
./ucmc_track ../demo/bboxes.yaml -f ../demo/config.yaml -p    # -p 可以使画面暂停，按空格键切换为自动播放，按其他键逐帧播放

# 看具体用法
./ucmc_track -?
```

## 调整外参矩阵
```
python scripts/estimate_cam_param.py --img /path/to/your/img.jpg \    # 使用图像文件
                                     --vid /path/to/your/video.mp4 \  # 或者使用视频文件，上面选项就不要写了
                                     --id number_of_frame_id \        # 使用视频文件的第number_of_frame_id帧
                                     --cam_para cfg/track.yaml \      # 参数文件
                                     --test-only                      # 仅测试，不修改参数文件内容    
```
