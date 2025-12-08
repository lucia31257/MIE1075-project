#  Empty Slot Detector (ROS2 + ONNX + Camera)

This ROS2 package performs **empty shelf slot detection** using a camera feed and a YOLOv11 ONNX model exported from Roboflow.  
It is designed for supermarket robots to check for out-of-stock conditions.

The detector activates **only when it receives a signal** from the navigation system or from the test script.

---

## 1. Requirements

### OS & Framework
- Ubuntu 22.04
- ROS2 Humble
- Python 3.10

### Python dependencies
Install required packages:

```bash
sudo apt install python3-pip
pip3 install onnxruntime opencv-python numpy
sudo apt install ros-humble-cv-bridge
```

If NumPy errors appear:

```bash
pip3 install "numpy<2"
```

---

##  2. Project Structure

Place this folder inside your ROS2 workspace:

```
~/ros2_ws/src/empty_slot_detector/
```

Directory contents:

```
empty_slot_detector/
├── empty_slot_detector
│   ├── detector_node.py          # Main detection node
│   ├── test_start_detection.py   # Test script to simulate navigation triggering
│   ├── __init__.py
├── package.xml
├── setup.py
├── setup.cfg
└── resource/empty_slot_detector
```

---

##  3. ONNX Model

Place the `best.onnx` model anywhere you like.  
Then update this line in `detector_node.py`:

```python
self.model_path = "/home/YOUR_USERNAME/best.onnx"
```

Ensure the path is correct.

---

## 4. Build the Package

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

##  5. Camera Requirements

The detector listens to:

```
/camera/image_raw
```

You must have:

- a Gazebo camera plugin **or**  
- a real camera node  

publishing this topic.

Check with:

```bash
ros2 topic list | grep image
```

---

##  6. Run the Empty Slot Detector

```bash
ros2 run empty_slot_detector detector
```

You should see:

```
🟢 Detector ready. Waiting for /start_detection = True...
```

---

##  7. Trigger Detection (Manual Test)

If you do NOT have the navigation system yet, use the test script:

```bash
ros2 run empty_slot_detector test_detection
```

Enter:

```
1
```

This triggers one detection cycle.

Expected output:

```
✔️ No empty slot detected.
```
or

```
⚠️ EMPTY slot detected.
```

---

## 8. What the Detector Does

- Waits for a Boolean signal on `/start_detection`
- When True, processes a single camera frame
- Runs ONNX inference using YOLOv11
- Checks confidence scores
- Prints:

```
✔️ No empty slot detected.
⚠️ EMPTY slot detected.
```

---

##  Troubleshooting

### NumPy Errors
```
ValueError: can only convert an array...
```

Fix:

```bash
pip3 install "numpy<2"
```

### cv_bridge Errors
Install:

```bash
sudo apt install ros-humble-cv-bridge
```

### Camera Not Publishing
Check:

```bash
ros2 topic list | grep image
```

---

## ✔️ The System is Ready

The package can now operate independently or be integrated into your robot’s navigation system.

For enhancements (visualization, saving images, publishing custom topics), more features can be added easily.

---
