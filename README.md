# 544_lab3

## Prerequistes

Please install these prerequistes:
```
```

## How to build

- clone the repo such that the folder `interfaces/` and `lab3/` are located inside your `src/` directory of your ros2 workspace
    - e.g your directory might look like:
    ```
    ros2_ws/
      build/
      install/
      launch/
      src/
        interfaces/
        lab3/
    ```
- cd into your workspace root (e.g. `ros2_ws/`) and then the run the commands in order:
    - `colcon build --packages-select interfaces`
    - `colcon build --packages-select lab3`
    - `. install/setup.bash`

## How to run

`ros2 run lab3 controller` will create the controller node. This can be activated all time since it only continuously updates the current pose of the robot until a service request is put it

`ros2 run lab3 client <start_x> <start_y> <end_x> <end_y>` will start the client node, which will send the start and end position as a Point request to the controller. The controller will perform A* on these start and end points on a pre-generated OGM, and run a controller on each node to reach the target destination. Client ends when the target has been reached.