## Crover Localization challenge

## Observations:

The following observations can be made initially with respect to the given data (data.bag file).

- GNSS sensor messages:
    - Type: nav_msgs/Odometry
    - Frequency: ~10Hz
    - Does not provide orientation data
- Wheel Odometry messages:
    - Type: nav_msgs/Odometry
    - Frequency: ~50Hz
    - Likely inaccurate Pose data but accurate Twist (velocity) data 

- cannot visualize GNSS and wheel odom at the same time( Messages are too old error)


    
