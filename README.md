# TAMEn: Tactile-Aware Manipulation Engine for Closed-Loop Data Collection in Contact-Rich Tasks

<video src="https://raw.githubusercontent.com/OpenDriveLab/opendrivelab.github.io/master/TAMEn/landing/teaser.mp4" autoplay muted loop playsinline width="100%"></video>

🚀 [Website](https://opendrivelab.com/TAMEn) | 📄 [Paper](https://arxiv.org/abs/2604.07335) | 🛠️ [Hardware (Coming Soon)](#hardware-assembly)

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
- [🎮 Getting Started](#-getting-started)
- [📝 Citation](#-citation)
- [🙏 Acknowledgements](#-acknowledgements)
- [📄 License](#-license)

## ✅ TODO List

> Note: TAMEn follows a staged release plan for hardware, data, and policy modules.
- [x] APK installation on Pico 4 Ultra (available and verified)
- [x] Release hardware assembly package (4 vision-tactile sensor adaptor types)
- [ ] Release data collection workflow and synchronization strategy
- [ ] Release training scripts, model configs, and checkpoints
- [ ] Release inference/deployment pipeline and benchmark examples

## 🎮 Getting Started

### @tAmeR Quick Start

`tAmeR` runs on **PICO 4 / 4 Ultra** and sends controller poses + button states to a PC via TCP.  
An optional ROS2 WebSocket-JPEG backend can be used to stream multi-camera visual/tactile images.

#### 1) Prerequisites
- Headset side:
  - `tAmeR.apk` installed on PICO
  - PICO and PC are in the same LAN
  - Controllers are paired and tracked
- PC side:
  - A TCP receiver is running on your PC (custom script or your own ROS bridge)
  - TCP port (default `8018`) is open and matches the app config

#### 2) Optional Video Backend (ROS2 -> WebSocket JPEG)
System recommendation: Ubuntu 20.04/22.04, ROS2 (Foxy/Humble/Iron), Python 3.8+.

Install dependencies:
   ```bash
   sudo apt update
   sudo apt install -y python3-pip python3-venv libopencv-dev
   python3 -m venv venv
   source venv/bin/activate
   pip install --upgrade pip
   pip install numpy opencv-python aiohttp
   ```

Verify ROS2 Python packages:
   ```bash
   python3 -c "import rclpy; print('rclpy ok')"
   python3 -c "from sensor_msgs.msg import Image; print('sensor_msgs ok')"
   ```

Start backend:
   ```bash
   source /opt/ros/<your_ros_distro>/setup.bash
   python tAmeR/tAmeR_ws.py \
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

Health checks:
- `http://<your-ip>:8765/health`
- `http://<your-ip>:8765/config`
- `ws://<your-ip>:8765/ws/video`

#### 3) Run VR Collection on Headset
1. Launch `tAmeR.apk` on PICO.
2. Input:
   - **PC IP** (LAN IP of your receiver machine)
   - **Port** (`8018` by default, or your custom TCP port)
   - **Period** (`0.01` recommended)
3. Click **Connect**.
4. After successful connection:
   - App starts sending data automatically
   - Input panel is hidden
   - **Disconnect** button is shown

Minimal TCP receiver example (Python):
```python
import socket

HOST = "0.0.0.0"
PORT = 8018

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen(1)
    print(f"Listening on {HOST}:{PORT}")
    conn, addr = s.accept()
    print("Connected by", addr)
    with conn:
        while True:
            data = conn.recv(4096)
            if not data:
                break
            print(data.decode("utf-8", errors="ignore").strip())
```

#### 4) Data Format (TCP)
Each line is semicolon-separated:
`timestamp;LG=T/F;RG=T/F;LT=T/F;RT=T/F;left_pose;right_pose;X=T/F;A=T/F;Y=T/F;B=T/F`

- `LG/RG`: grip buttons
- `LT/RT`: triggers
- `X/Y/A/B`: face buttons
- `left_pose/right_pose`: `x y z rx ry rz`

#### 5) Troubleshooting
- **Connect failed**: check receiver process, IP/port match, and firewall.
- **valid=False / controller not tracked**: re-pair controllers and verify headset tracking.
- **Connected but no data**: confirm same LAN and verify receiver parsing logic.

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

This project is licensed under the [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/).
