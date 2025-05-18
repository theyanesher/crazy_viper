# ViPER Integration with CrazySwarm2

## ENV SETUP

- Create and activate conda env with ```crazy.yml``` 

- clone the repo inside a workspace and build using 
    ```bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

- clone the CrazySim repo outside the workspace 
    ```bash
    git clone https://github.com/theyanesher/CrazySim.git
    ```

- cd into CrazySim/crazyflie-lib-python
    ```bash
    pip install e .
    ```

- install Gazebo Garden from https://gazebosim.org/docs/garden/install_ubuntu

- cd into CrazySim/crazyflie-firmware
    ```bash
        cd crazyflie-firmware
        mkdir -p sitl_make/build && cd $_
        cmake ..
        make all
    ```

## RUN COMMANDS

### viper_demo with Crazyswarm
Remember to source WS
> /$WS/src/crazyswarm2/crazyflie_py
```bash
    python3 viper_demo.py
```

### Firmware and GZ 
>  CrazySim/crazyflie-firmware
```bash
    bash tools/crazyflie-simulation/simulator_files/gazebo/launch sitl_multiagent_text.sh -m crazyflie
```

### Crazyflie
Remember to source WS
> /$WS
```bash
    ros2 launch crazyflie launch.py backend:=cflib
```

**Check the file paths once inside viper_demo.py**


