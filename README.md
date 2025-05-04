# F1Tenth-ROS2-Workspace

In this GitHub Repo the `ros2_ws` is outlined package by package to give an understand of what each package does and why it was created.

> **Note:** Don't forget to add this ros2_ws to the bottom of your `~/.bashrc` file. Also if an autonomuous package is being used you must hold down the **right bumper (RB)** to unable autonomus driving. If RB is not held down the car will not run properly.

### Package list
- drive2
- test_5m
- safety_pkg
- wall_follower
- gap_follow


## `drive2`

`drive2` is a simple package that was used to check if the autonomuous control was working properly. This can be helpful when the controller is first setup.

### Steps to run `drive2` along with an explanation

>**Note:** When running this ros2 package place the car on a box where the wheels aren't touching the ground or the box. Putting th car in this position will allow the wheels to freely spin.

### Step: 1 - Connect Controller and Start Launch File

To connect controller go to settings and pair the controller to the car. Then use the follolowing command to launch all nessecary files to drive the F1Tenth car. This bringup launch will intializes the VESC drivers, the LiDAR drivers, the joystick drivers, and all necessary packages for running the car. 

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

Ensure that the ros2_ws is properly sourced.

### Step: 2 - Run the `drive2` Package

When properly setting up a ROS2 package you can use the `ros2 run` command to execute the package. Open a new terminal window and use the following command to run the `drive2` package. This will have the car spin its wheels at 2 m/s until the package is killed.

```bash
ros2 run drive2 drive2
```

## `test_5m`

`test_5m` is used to tune the odometry of the car. This package controls the car to drive at 1 m/s for 5 seconds for a total of 5 meters. This can be measured on for accuracy and will give more consistant results than manual control.

### Steps to run `test_5m` along with an explanation

### Step: 1 - Connect Controller and Start Launch File

To connect controller go to settings and pair the controller to the car. Then use the follolowing command to launch all nessecary files to drive the F1Tenth car. This bringup launch will intializes the VESC drivers, the LiDAR drivers, the joystick drivers, and all necessary packages for running the car. 

```bash
ros2 launch f1tenth_stack bringup_launch.py
```
### Step: 2 - Monitor Position and Run the `test_5m` Package

To monitor the position on the car use the following command in a new bash session.

```bash
ros2 topic echo --no-arr /odom
```

Next when using a properly setting up ROS2 package you can use the `ros2 run` command to execute the package. Open a new terminal window and use the following command to run the `test_5m` package. This will have the car spin its wheels at 1 m/s for 5 seconds.

>**Note:** While running the command the **right bumper (RB)** must be pressed or the car will not move.

```bash
ros2 run test_5m test_5m
```

If the distance in the `ros2 topic echo --no-arr /odom` window is not 5 meters than the `speed_to_erpm_offset` value needs to be changed this can be done by opening the `$HOME/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml` file and editing the `speed_to_erpm_offset` value. Compare the measured value to the value shown in the echoed message. If the distance reported by echo is larger, decrease the `speed_to_erpm_gain` value. Otherwise increase the gain. The change is usually on the order of thousands. Note that changing this value also changes the forward speed of teleop. Please drive carefully once the velocity is calibrated. If the forward speed when teleoping is too high, change the scale in `human_control` for `drive-speed` in `joy_teleop.yaml`.


## `safety_pkg`

The `safety_pkg` is a ros2 package that incorperates automatic emergency braking. This means that the car can stop when an obsacle is detected but the operator is unable to stop in time. The governing equation is `d = v² / (2 * |a|)`. This equation had to be modified due to the cars deceleration rate. To account for the slower deceleration in real life verses in the simulator the velocity was doubled to increase the amount of stopping distance.

> **Note:** The most important part of this package is that it can over ride the manual and autonomuous controls. To do this you will need to change the `mux.yaml` file. This file can be found `$HOME/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/mux.yaml`. The following is an example of what needs to be changed. The `safety_node` portion was added to the `mux` file with a **higher priority** than any other **topic**.

    ```yaml
    ackermann_mux:
    ros__parameters:
        topics:
        navigation:
            topic   : /drive
            timeout : 0.2
            priority: 10
        joystick:
            topic   : /teleop
            timeout : 0.2
            priority: 100
        safety_node:
            topic   : /safety_node
            timeout : 1.0
            priority: 200
    ```

### Step: 1 - Connect Controller and Start Launch File

To connect controller go to settings and pair the controller to the car. Then use the follolowing command to launch all nessecary files to drive the F1Tenth car. This bringup launch will intializes the VESC drivers, the LiDAR drivers, the joystick drivers, and all necessary packages for running the car. 

```bash
ros2 launch f1tenth_stack bringup_launch.py
```
### Step: 2 - Run the `safety_pkg` Package

Next when using a properly setting up ROS2 package you can use the `ros2 run` command to execute the package. Open a new terminal window and use the following command to run the `safety_pkg` package. This will have the car stop with a reasonable stopping distance.

```bash
ros2 run safety_node safety_node
```

Now you will be able to drive freely in manual control without the worry of running into an object.

## `wall_follower`



## `gap_follower`

This package was downloaded from Maliks SURE program has been implemented but not testing or modifcations have been made.