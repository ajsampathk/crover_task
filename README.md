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

- cannot visualize GNSS and wheel odom at the same time( Messages are too old error) FIXED: set use_sim_time=true

## Approach:

The problem requires combining two different odometry information to output a predicted position and orientation of the car. 
This can be done mostly by a filter, a kalman-filter or an EKF is most likely to give the best estimates. However, this will take quite 
a bit of research as it is not something that I am familar with at this moment. 
 

    
