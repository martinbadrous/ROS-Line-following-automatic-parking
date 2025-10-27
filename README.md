# 🐢 ROS Line Following & Automatic Parking (Melodic)

ROS Melodic (Ubuntu 18.04) implementation of **line following** and **automatic parking** for **TurtleBot3** in Gazebo or on a real robot.

## ✨ Features
- Visual **line detection** (HSV thresholding + morphology) and **PID control**
- **Automatic parking** using LaserScan front distance
- Clean ROS node separation + launch files
- Works in **simulation** or **real** TurtleBot3 (Burger)

## 🧱 Repository Layout
```
ROS-Line-following-automatic-parking/
├── my_following_line_package_1/        # Line detection + PID controller
│   ├── scripts/
│   │   ├── line_detector.py
│   │   └── controller.py
│   ├── launch/
│   │   └── line_following.launch
│   ├── package.xml
│   └── CMakeLists.txt
├── Parking/                            # Automatic parking node
│   ├── scripts/
│   │   └── auto_parking.py
│   ├── launch/
│   │   └── parking.launch
│   ├── package.xml
│   └── CMakeLists.txt
└── README.md
```

## 🛠️ Setup (ROS Melodic)
```bash
# 1) Install ROS Melodic + TurtleBot3 sims
sudo apt update
sudo apt install ros-melodic-desktop-full
sudo apt install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
sudo apt install ros-melodic-turtlebot3-gazebo ros-melodic-turtlebot3-simulations

# 2) Create catkin workspace (if you don't have one)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 3) Clone the repository
git clone https://github.com/martinbadrous/ROS-Line-following-automatic-parking.git
cd .. && catkin_make
source ~/catkin_ws/devel/setup.bash

# 4) Set TurtleBot3 model
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
export TURTLEBOT3_MODEL=burger
```

## 🚀 Run (Simulation)
In one terminal:
```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
In another terminal:
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch my_following_line_package_1 line_following.launch display:=true camera_topic:=/camera/rgb/image_raw
```
For automatic parking:
```bash
roslaunch Parking parking.launch
```

## 🧩 Nodes & Topics

### `line_detector.py`
- **Subscribes**: `/camera/rgb/image_raw` (`sensor_msgs/Image`)
- **Publishes**: `/line_error` (`std_msgs/Float32`), `/line_visible` (`std_msgs/Bool`)
- **Params**: `hsv_lower`, `hsv_upper`, `blur`, `min_area`, `display`

### `controller.py`
- **Subscribes**: `/line_error` (`Float32`), `/line_visible` (`Bool`)
- **Publishes**: `/cmd_vel` (`geometry_msgs/Twist`)
- **Params**: `kp`, `ki`, `kd`, `lin_speed`, `max_ang`

### `auto_parking.py`
- **Subscribes**: `/scan` (`sensor_msgs/LaserScan`)
- **Publishes**: `/cmd_vel` (`geometry_msgs/Twist`)
- **Params**: `target_distance`, `approach_speed`, `scan_angle_deg`

## 🔧 Tuning Tips
- Adjust `hsv_lower` / `hsv_upper` to match your line color.
- Start with small `kp` and gradually increase; add `kd` to damp oscillations.
- For real TurtleBot3, reduce `lin_speed` to avoid overshoot.

## 🧪 Real Robot
```bash
# On robot:
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_bringup turtlebot3_robot.launch

# On desktop (networking setup with ROS_MASTER_URI/ROS_HOSTNAME)
roslaunch my_following_line_package_1 line_following.launch camera_topic:=/raspicam_node/image
```

## 👤 Author
**Martin Badrous** — Computer Vision & Robotics Engineer  
📧 martin.badrous@gmail.com | 🔗 https://github.com/martinbadrous

License: MIT
