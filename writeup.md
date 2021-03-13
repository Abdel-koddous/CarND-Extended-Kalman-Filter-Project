# **CarND Extended Kalman Filter Project**
### Abdelkoddous Khamsi

[image1]: ./writeup_resources/radarAndLidar.jpg "Lidar and Radar data handled"
[gif1]: ./writeup_resources/lidar.gif "Lidar Only"
[gif2]: ./writeup_resources/radar.gif "Radar Only"
[gif3]: ./writeup_resources/radar&lidar.gif "Lidar and Radar"

* The goal of this project is to track a vehicle position and velocity using the Extended Kalman filter given simulated radar and lidar measurements.

The main steps I went through in this project are orgamised as follows:
1. Initialized Extended Kalman Filter state and covariance matrices from raw measurements in `FusionEKF.cpp`. 
2. Implemented the prediction step in `kalman_filter.cpp`.
3. Implemented the update state for both the lidar and the non-linear radar measurements in `kalman_filter.cpp`.
4. In the `main.cpp` I added the option to take into account only the radar data, the lidar data or both for maximum accuracy of the tracking. 


The Following table shows the results I was able to get in the various scenarios:

|     RMSE      |    X    |   Y    |   Vx   |   Vy   | 
| -----------   | ------- | ------ |------- |------- |
| Radar Only    |  0.2415 | 0.3373 | 0.6025 | 0.7565 |
| Lidar Only    |  0.1838 | 0.1542 | 0.6051 | 0.4858 |
| Lidar & Radar |  0.0973 | 0.0855 | 0.4513 | 0.4399 |

* End result with Lidar and Radar measurements handled:

Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles.

![alt text][image1]

### **Extended Kalman Filter in Action!**

* Using lidar measurments only:

![alt text][gif1]

* Using radar measurements only:

![alt text][gif2]

* Using both lidar and radar measurements:

![alt text][gif3]
