# UCMCTrack的c++部署

![demo](https://github.com/LSH9832/UCMCTrack-cpp/assets/69880398/f732ec98-8449-4683-8f1a-a57cc9233416)


## 致谢

首先感谢大佬[corfyi](https://github.com/corfyi)将此多目标跟踪算法开源，但在[issues](https://github.com/corfyi/UCMCTrack/issues/18)中看出大佬是有c++版本的代码的，却并没有开源。本人水平有限，工作之余花费4天时间将原项目（[UCMCTrack](https://github.com/corfyi/UCMCTrack)）由python转为c++代码，难免出错，如发现bug或算法逻辑不对的地方欢迎在issues中提出。


## C++真是太快啦！

实测python版本的代码FPS不到100Hz, 作者在原文中提到纯跟踪情况下他代码的C++版FPS能达到1000+FPS，如果我的代码没写错的话，在demo.mp4中貌似可以来到3000FPS左右哦，见下图（CPU型号为I5-11320H）

![ucmcspeed](https://github.com/LSH9832/UCMCTrack-cpp/assets/69880398/0594d9c0-ef21-492e-ae78-ac267ba9be19)

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
