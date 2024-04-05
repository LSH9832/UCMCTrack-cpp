# UCMCTrack C++ Deployment

![demo](https://github.com/LSH9832/UCMCTrack-cpp/assets/69880398/f732ec98-8449-4683-8f1a-a57cc9233416)


## Acknowledgements

First and foremost, I would like to thank [corfyi](https://github.com/corfyi) for open-sourcing this multi-object tracking algorithm. However, as mentioned in the issues, while the author does have a C++ version of the code, it has not been made public. Despite my limited expertise, I spent four days converting the original project ([UCMCTrack](https://github.com/corfyi/UCMCTrack)) from Python to C++ code. And if you find any bugs or inconsistencies in the algorithm logic, please feel free to raise them in the issues section.


## C++ Is Blazing Fast!

The measured FPS of the Python version of the code is less than 100Hz. In the original paper, the author mentioned that the C++ version of their code can achieve over 1000+ FPS in pure tracking scenarios. If my implementation is correct, it seems to reach around 3000FPS in the demo.mp4 (using an I5-11320H CPU), as shown in the image below. When combined with YOLO, the speed is even faster (due to the GPU inference latency reducing the CPU load).

![ucmcspeed](https://github.com/LSH9832/UCMCTrack-cpp/assets/69880398/0594d9c0-ef21-492e-ae78-ac267ba9be19)

Even on an ARM platform (Orange Pi, RK3588 chip), the speed is almost unchanged (with CPU utilization below 10%). However, when combined with a neural network, there is a significant slowdown (as RKNN not only loads the NPU but also the CPU). With an average CPU utilization of 45%, the latency is approximately 8 ms.

![ucmcspeedrk3588](https://github.com/LSH9832/UCMCTrack-cpp/assets/69880398/7943d9ec-0f1a-447b-80d7-a8d8ba2b8820)


## Dependencies

- libopencv-dev
- libeigen3-dev
- libyaml-cpp0.6


## Notes

- This project does not provide code for the object detection part. The example code uses the detection results from the demo.mp4 provided by the original author, which are written to the demo/bboxes.yaml file. The parameters are written to the demo/config.yaml file. You can modify the parameters according to your needs and add detection code to replace the results in the file. The detection results in the file are from a model I trained on VisDrone-DET-2019, see my other project [edgeyolo](https://github.com/LSH9832/edgeyolo). The detection framework also uses the cpp source code provided in that project.

- The code in include/image_utils/lap is from [gatagat/lap](https://github.com/gatagat/lap), which is the source code for the lap library in Python.


## Setup
Modify the CMakeLists.txt file according to your needs, then
```shell
mkdir build && cd build
cmake ..
make
```
## Using the Demo
Navigate to the build folder and then
```shell
./ucmc_track ../demo/bboxes.yaml -f ../demo/config.yaml -p    # The '-p' flag pauses the video, press the spacebar to switch to autoplay, or any other key to play frame by frame  
  
# For detailed usage instructions  
./ucmc_track -?
```

## Adjusting Extrinsic Parameters
```
python scripts/estimate_cam_param.py --img /path/to/your/img.jpg \    # Use an image file
                                     --vid /path/to/your/video.mp4 \  # Or use a video file, do not use the above option if this one is selected  
                                     --id number_of_frame_id \        # Use the number_of_frame_id frame from the video file
                                     --cam_para cfg/track.yaml \      # parameter file
                                     --test-only                      # Test mode only, without modifying the content of the parameter file
```
