
# ROS2 Vision & Gimbal Control

本專案使用 ROS2 進行視覺處理與雲台控制，透過 Serial Bridge 建立 ROS2 與控制器之間的通訊。

---

## 1. 環境需求

- Ubuntu 22.04
- ROS2 Humble
- Python 3

## 2. 安裝相依套件

安裝 Python Serial 通訊套件：

```bash
python3 -m pip install pyserial
```

## 3. 啟動程式

### 3.1 視覺辨識節點

```bash
ros2 run vision vision
```

### 3.2 Serial Bridge 通訊節點

```bash
ros2 run vision bridge
```

## 4. 專案檔案結構

```text
vision/
├── package.xml
├── setup.py
├── resource/
├── vision/
│   ├── vision.py
│   ├── bridge.py
│   └── arduino.cpp
└── test/
```
