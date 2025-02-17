## Run through guide for setting up Roomba Assignemnt
- Start Docker desktop
- Starting docker virtual environment
``` bash
docker build -f Dockerfile_roomba -t ros2_roomba .
```
```bash
docker run -it --rm --name ros2_roomba_container -e DISPLAY=host.docker.internal:0.0 -v .\lab_one\:/home/ros2_ws ros2_roomba
````

- Launching robot in virtual world
 ``` bash
    cd /home/ros2_ws
        colcon build --symlink-install ##Build when docker first started up
       source install/setup.bash
       ros2 launch roomba_one launch_sim.launch.py world:=src/world roomba_world.world
``` 
Make sure xserver is running in the background
- Opening docker terminal
```` bash 
docker exec -it ros2_roomba_container /bin/bash
````
- Running keyboard controls
````bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
````

- Running SLAM 
````bash
ros2 launch slam_toolbox online_async_launch.py async:=true use_sim_time:=true
````

ros2 controller node needs to be running

