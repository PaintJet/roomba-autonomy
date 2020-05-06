# Sensor Suite

We selected 4 sensors for autonomous localization and navigation.

## [Decawave MDEK1001](decawave.com/product/mdek1001-deployment-kit/)
This sensor is a time of flight (ToF) sensor that measures distance to anchors placed in the environment. The ulta-wide band spectrum used helps for penetration through obstacles.

With 4+ anchors placed in the environment, we are capable of meausuring the position of the robot in 2D space. The sensors are not very accurate, but provide us a global estimate of the robot position.

## [Kauai Labs NavX-Micro IMU](https://pdocs.kauailabs.com/navx-micro/)
This IMU provides us acceleration and orientation measurements. The sensor has filtering algorithms onboard that significantly reduce noise and drift in the sensor measurements. In our testing, we saw < 1 degree of drift over 30 minutes of use in the orientation estimate. This provides us a very stable estimate of the robot's orientation. The accelerometer still has a lot of noise despite the filtering. Therefore, we plan to use the accelerometer to flag high acceleration events that may have moved the robot without the other sensors detecting the position. The plan was to simply add noise to the state distribution proportional to the size of the acceleration event.

## [SLAMTEC RPLidar A3](https://www.slamtec.com/en/Lidar/A3)
This 2D lidar provides us distance measurements in a single plane with very high accuracy. These measurements can be used in 2 ways.

1. Detect dynamic obstacles that had not been included in the pre-generated map
2. Compare measurements with pre-generated map to localize robot

## Wheel Encoders
The wheel encoders measure the position of the robot by measuring how much the wheels have rotated. This measurement is subject to drift over time, but provides a continuously envolving measurement of state (meaning there are no discontinuities in the position estimate).

# Why these sensors?
The sensors were selected to compliment each others strenghts and weaknesses. The LIDAR gives us the most accurate data out of all the sensors, but only works in a feature-rich environment (areas of the room where there are lots of walls or other obstacles). When in feature rich areas, we will get very good position and orientation (pose) estimates. However, when we are in the middle of a warehouse floor, we need other sensors for position estimates.

This is where the Decawave modules are useful. With a high enough density of anchors in the environment, we will have an always online estimate of the pose regardless of where we are in the warehouse. The estimate is rough, but not subject to drift. Similarly, the wheel encoders can also be used for localization in the middle of the warehouse. The pose estimate from the wheel encoders is subject to drift. These two sensors together can provide localization well enough until the robot approaches a wall again, which will allow the LIDAR to correct any errors in the estimate.

Finally, the IMU provides a very good estimate of the robot orientation, but it is a relative measurement (meaning we don't know the actual orientation, just how it has changed relative to the starting orientation of the robot) and subject to slow drift. The LIDAR can solve both of these issues by providing a good estimate of the orientation relative to the warehouse when the robot is turned on and can correct drift in the IMU during the run.

All of these sensors together should be capable of stable localization.