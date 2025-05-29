# Bluerobotics Stonefish simulator
## Description
This project aimed to connect stonefish from [Patryk Cieślak](https://github.com/patrykcieslak/stonefish) to the [Bluerobotics](https://github.com/bluerobotics) architecture directly from the [ArduSub](https://github.com/ArduPilot/ardupilot) firmware.
## Clone 
```bash
git clone https://github.com/Aymccar/Simulator.git --recurse-submodules
```
## Installation 
### Building Stonefish
#### Dependencies
- OpenGL (libglm-dev)
- SDL2 libsdl2-dev
- Freetype (libfreetype6-dev)

```bash
cd stonefish
mkdir build && cd build
cmake ..
make -jX && sudo make install
```
---

### Building BlueOS core docker image 
#### Dependencies 
- docker

```bash
cd blueos-docker-base
docker build . -t ros-blueos-base
```
---
### Building BlueOS docker image
#### Dependencies
-	docker

```bash
cd BlueOS/core
docker build . -t ros-blueos
```
This will automatically clone and build the [Waf_ROS2_bridge](https://github.com/Aymccar/Waf_Ros2_Bridge).

---
### Building ROS architecture
#### Dependencies
- ROS2 (tested with ROS2 Jazzy)

```bash
cd colcon_ws
colcon build
```
Do not forget to source the ROS2 workspace 
```bash
source install/setup.bash|zsh
```
----
### Building Ardupilot firmware
#### Dependencies
- Classic ardupilot dep
Run ```Tools/environment_install/install-prereqs-ubuntu.sh -y``` on the root
- ROS2 (tested with ROS2 Jazzy)
- Waf_Ros2_bridge (built previously)
```bash
cd ardupilot 
./waf configure --board ros --toolchain native --wafrosbridge_include_path  
"{PATH to this repo}/colcon_ws/src/wafros2bridge/include" --wafrosbridge_lib_path "{PATH to this repo}/colcon_ws/install/wafros2bridge/lib"
./waf sub
```

## Running the simulator
First launch BlueOS:
```bash
cd BlueOS
docker compose -f core/compose/compose.yml up
``` 
Then access to **localhost** in a browser.
Click on the robot face on left top and enable **Pirate Mode**.
Go to **Autopilot Firmware**.
Click on **Change Board** and select ROS.
Click on **UPLOAD FIRMWARE FILE** and upload the freshly compiled firmware located in ``{PATH to this repo}/ardupilot/ros/bin/ardusub``.

Then run: 
```bash 
ros2 launch ArdupilotStonefish parser_launch.yaml
```
and finally run Stonefish with:
```bash
ros2 launch stonefish_ros2 launchros.launch.py #Add stonefish args if needed
```

