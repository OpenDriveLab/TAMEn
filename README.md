# TAMEn: Tactile-Aware Manipulation Engine for Closed-Loop Data Collection in Contact-Rich Tasks

<img src="https://raw.githubusercontent.com/OpenDriveLab/opendrivelab.github.io/master/TAMEn/landing/teaser.jpg" alt="TAMEn: dual-mode acquisition, data recipe, and data flywheel (teaser)" width="100%" />

🚀 [Website](https://opendrivelab.com/TAMEn) | 📄 [Paper](https://arxiv.org/abs/2604.07335) | 🛠️ [Hardware (Coming Soon)](https://github.com/OpenDriveLab/TAMEn)

✒️ **Longyan Wu**<sup>1,2,3</sup>, **Jieji Ren**<sup>4</sup>, **Chenghang Jiang**<sup>5</sup>, **Junxi Zhou**<sup>5</sup>, **Shijia Peng**<sup>3</sup>, **Ran Huang**<sup>1</sup>, **Guoying Gu**<sup>4</sup>, **Li Chen**<sup>3</sup>, **Hongyang Li**<sup>2,3</sup>

💼 <sup>1</sup> Fudan University; <sup>2</sup> Shanghai Innovation Institute; <sup>3</sup> OpenDriveLab at The University of Hong Kong; <sup>4</sup> Shanghai Jiao Tong University; <sup>5</sup> East China University of Science and Technology

## 🦾 Highlight
- **(i)** A visuo-tactile data **engine** for bimanual contact-rich manipulation, which integrates hardware, acquisition strategy, and policy learning into a closed-loop framework.
- **(ii)** A human-machine **interface** that supports a dual-mode pipeline with sub-millimeter MoCap and VR-based in-the-wild acquisition, and can rapidly adapt to heterogeneous grippers.
- **(iii)** A data collection **recipe** that incorporates real-time validation during collection and organizes heterogeneous multi-modal data into a pyramid-structured regime for staged learning.
- **(iv)** A closed-loop data **flywheel** that leverages AR-based teleoperation with tactile feedback (tAmeR) to refine policies using corrective data from realistic failures.

## 🎯 Table of Contents
- [🦾 Highlight](#-highlight)
- [✅ TODO List](#-todo-list)
- [🎮 tAmeR Teleopration](#-tamer-teleopration)
- [📝 Citation](#-citation)
- [🙏 Acknowledgements](#-acknowledgements)
- [📄 License](#-license)

## ✅ TODO List

> Note: TAMEn follows a staged release plan for hardware, data collection, and policy learning.
- [x] Release tAmeR app for Pico 4 Ultra / Pico 4 and the corresponding teleoperation programs
- [ ] Release CAD models for multimodal data collection devices (compatible with GelSight, Xense, DW-Tac, PaXini, and our own sensor)
- [ ] Release data collection workflow and dataset
- [ ] Release training scripts, model configs, and inference pipeline

## 🎮 tAmeR Teleopration

### tAmeR Quick Start

**tAmeR** is an AR app for robot teleoperation, providing operators with real-time visual and tactile feedback.

- `tAmeR` runs on **PICO 4 / 4 Ultra** and sends controller poses + button states to a PC via TCP.
- It can be directly applied to teleoperation for arbitrary robot arms. This repository now includes the teleoperation workflow for **JAKA K1**, and support for **AgileX Piper** will be open-sourced in the future.
- A ROS2 WebSocket-JPEG backend can be used to stream multi-camera visual/tactile images.

#### 1) Prerequisites
- Headset side:
  - `tAmeR.apk` installed on PICO
  - PICO and PC are in the same LAN
  - Controllers are paired and tracked
- PC side:
  - A TCP receiver is running on your PC
  - TCP port (default `8018`) is open and matches the app config
  - Wrist cameras and visuo-tactile cameras are connected to the PC

#### 2) Video Backend (ROS2 -> WebSocket JPEG)

**Dependencies**

```bash
sudo apt update
sudo apt install -y python3-opencv python3-aiohttp python3-numpy
source /opt/ros/<your_ros_distro>/setup.bash
python3 -c "import rclpy; from sensor_msgs.msg import Image; print('ros2 ok')"
```

**Start backend service**

```bash
# optional venv
# python3 -m venv tamen && source tamen/bin/activate && pip install --upgrade pip && pip install numpy opencv-python aiohttp

source /opt/ros/<your_ros_distro>/setup.bash
python3 tAmeR/tAmeR_ws.py \
  --host 0.0.0.0 \
  --port 8765 \
  --left-topic /left_camera/color/image_raw \
  --right-topic /right_camera/color/image_raw \
  --right-tactile-topic /right_tactile_camera/color/image_raw \
  --left-tactile-topic /left_tactile_camera/color/image_raw \
  --tile-width 320 \
  --tile-height 240 \
  --fps 10 \
  --jpeg-quality 70
```

#### 3) Teleoperation Workflow for JAKA K1

**Power on and enable the robot**

```bash
ros2 launch vr_data_pub teleoperation_system_30degree_xy.launch.py
```

**Start the cameras**

```bash
ros2 launch orbbec_camera tac_multi_usbcamera_xy.launch.py
```

**Connect the headset**

```text
Launch tAmeR.apk on PICO.

[Input]
PC IP : LAN IP of your PC
Port  : 8018 (default) or your custom TCP port
Period: 0.01 (recommended)

[Action]
Click LOGIN.

[Status]
App starts sending data automatically.
Input panel is hidden.
LOGOUT button is shown.
```

**Teleoperate the robot arm**

```bash
ros2 run vr_data_pub teleoperation_control_30degree_xy
```

**Record robot poses and camera data**

```bash
ros2 run vr_data_pub collect_data_xy
```

#### 4) Data Format (TCP)
Each line is semicolon-separated:
`timestamp;LG=T/F;RG=T/F;LT=T/F;RT=T/F;left_pose;right_pose;X=T/F;A=T/F;Y=T/F;B=T/F`
`left_pose/right_pose`: `x y z rx ry rz`

- `LT/RT`: control gripper closing/opening
- `RG`: start teleoperation for the robot arm only, and start data recording
- `LG`: start teleoperation for both the robot arm and gripper, and start data recording
- `B`: reset the robot
- `Y`: fully exit teleoperation
- `A`: finish recording the current data segment


## 📝 Citation

If you find this project useful in your research, please consider citing:

```bibtex
@misc{wu2026tamentactileawaremanipulationengine,
      title={TAMEn: Tactile-Aware Manipulation Engine for Closed-Loop Data Collection in Contact-Rich Tasks}, 
      author={Longyan Wu and Jieji Ren and Chenghang Jiang and Junxi Zhou and Shijia Peng and Ran Huang and Guoying Gu and Li Chen and Hongyang Li},
      year={2026},
      eprint={2604.07335},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2604.07335}, 
}
```

## 🙏 Acknowledgements

We gratefully acknowledge Qianyu Guo, Checheng Yu, Chonghao Sima, Jingmin Zhang, and Chenyu Lin for their valuable insights and constructive discussions. We also extend our sincere gratitude to JAKA for their generous hardware and technical support.

## 📄 License

This project is licensed under the [Apache License 2.0](./LICENSE.txt).
