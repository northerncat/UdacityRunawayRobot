# UdacityRunawayRobot
The Runaway Robot final project for Udacity's online course, [Artificial Intelligence for Robotics](https://classroom.udacity.com/courses/cs373).

## The Project
The project describes a scenario where a robot is lost in the desert, and we are going to use localization, search, path
planning and PID controls to track the robot and take it back. We assume that the robot moves in a circular motion on a (x, y)
coordinate plane, where it moves for a certain distance, localizes itself, turns a certain angle, and then moves again.
Whenever the robot localizes itself, we would get measurements of its position.

## Code Summary
The robot is simulated with the program provided by Udacity in `robot.py`. Udacity also provides a `matrix.py` to perform basic
matrix computations.

My implementation parts of the project are divided into five guided parts, which will be described next.

## Part 1: Noiseless Prediction
In this first part, we assume that the robot's motion is exact, and its measurements of its positions are also accurate. Based
on these assumptions, we would be able to deduce its motion patterns, including the distance between each measurements and the
turning angle, after three measurements.

## Part 2: Adding Noise
In this part, we assume that the robot's measurements of its positions have a zero-mean Gaussian noise, while its movements are
still exact. I decide to use a Kalman filter to estimate the heading direction, the turning angle and distance traveled between
each measurements.
