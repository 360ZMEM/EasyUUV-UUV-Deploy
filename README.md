# EasyUUV-UUV-Deploy

[![arXiv](https://img.shields.io/badge/arXiv-2503.00527-b31b1b.svg)](https://arxiv.org/abs/2503.00527)  [![WebSite](https://img.shields.io/badge/Github_Page-Supp_Material-77DDFF.svg)](https://360zmem.github.io/AUV-RSControl/) [![WebSite](https://img.shields.io/github/last-commit/360ZMEM/AUV-RSControl-Code?color=green)](https://360zmem.github.io/AUV-RSControl-Code)

This repository contains code implementation for hardware deployment of the paper "EasyUUV: An LLM-Enhanced Universal and Lightweight Sim-to-Real Reinforcement Learning Framework for UUV Attitude Control".

The RL policy training and simulation code repository refers to [**HERE**](https://github.com/360ZMEM/EasyUUV-Isaac-Simulation).

<这里也放图片>

## Introduction

In this repository, we provide essential code for the UUV hardware deployment described in our paper, along with potentially useful guidance. Specifically, our EasyUUV is driven using ESP32-WROOM with Arduino API. In the original paper, we ran reinforcement learning policies and called LLM models on a computer, which transmitted the reinforcement learning output ($e(t)$ from Eq. (3) of the paper) and controller parameter adjustments via RS485 serial communication. Upon receiving this data, the ESP32 drives the A-S-Surface controller and handles sensor reading logic.

Additional notes:

- We use the Arduino API (C++) to drive the ESP32 control board.
- Python scripts are executed on the computer side for serial communication. With proper serial configuration, this Python code can also be used on single-board computers (SBCs) such as Raspberry Pi or Jetson Nano. Additionally, if you need to call the LLM, ensure that the SBC has internet access.
- The prompt of our LLM workflow refers to `python/prompt.py`.

## Get Started - ESP32

### Arduino Configuration

First, ensure that you have installed the [Arduino IDE](https://www.arduino.cc/en/software/). Then, install the necessary hardware support packages and the PID control library. In this study, we used the ESP32-WROOM.

After this, you can open the file `esp32_arduino/esp32_arduino.ino` in the IDE for further operations, and compile and upload it.

### Deployment Recommendations

- Our UUV has 8 thrusters, whose corresponding pin definitions are in `thrusterPins`. You need to adjust these according to your hardware deployment. Additionally, you should modify the variable `thrustMatrix`, which determines which thrusters are used for controlling various attitude angles and their corresponding thrust directions.
- Some parameters in this code were manually tuned after hardware deployment and provide good initial performance. However, we cannot guarantee that they will work well on your hardware. You may need to make appropriate adjustments.
- This code contains some hardware-specific sections, such as IMU reading based on the JY60 from Wit Motion. You can adjust these according to your actual hardware.

## Get Started - PC/SBC

To configure the system on a PC/SBC, you should modify the file `python/configs.py`, which contains the following important variables:

- `serial_id`: The serial port identifier obtained from Arduino IDE or other serial communication software (e.g., "COM7" for Windows).
- `enable_LLM`: Whether to enable the LLM for adjusting controller parameters.
- `api_key`: The API key used for calling the LLM.
- Modify `api_base_url` and `model_name` as needed.

After this, ensure that you have set up a Python virtual environment and installed the necessary libraries:

```bash
pip install -r python/requirements.txt
```

Once the configuration is complete, you can run the main program to start control (if using a SBC, consider necessary peripheral configurations such as auto-start):

```bash
python python/main.py
```

Additionally, we provide a Python program that only sends target values through the serial port, serving as a baseline for our paper (denoted as w/o RL):

```bash
python python/main_woRL.py
```

After executing the above programs, the results will be generated in the `hist_log` folder, including:
- Data recording files containing the actual/target attitude angles of the UUV and corresponding timestamps (see the `Record` class definition in `python/main.py` for details).
- Pose recording images named with date and time (also used for multimodal LLM interaction).
- All intermediate files from interactions with the LLM.

## Citation

If you find the code repository useful for your work, please cite:

```bibtex
@article{xie2025AUVRScontrol,
      title={Never too Prim to Swim: An LLM-Enhanced RL-based Adaptive S-Surface Controller for AUVs under Extre Sea Conditions},
      author={Xie, Guanwen and Xu, Jingzehua and Ding, Yimian and Zhang, Zhi and Zhang, Shuai and Li, Yi},
      journal={arXiv preprint arXiv:2503.00527},
      year={2025}}
```