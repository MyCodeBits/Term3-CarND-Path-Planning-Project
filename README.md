# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

The project STARTER details from UDACITY's are belwo from the Section named : "Goals".

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

## Implementation/Submission Details:

With reference to [Rubric](https://review.udacity.com/#!/rubrics/1020/view):

- The code compiles correctly.  - **<span style="color:green; font-size:1em;"> YES </span>**
- The car is able to drive at least 4.32 miles without incident - **<span style="color:green; font-size:1em;"> YES, tested for above 20 miles. </span>**
- The car drives according to the speed limit - **<span style="color:green; font-size:1em;"> Always stayed within 49.5 MPH. </span>**
- Max Acceleration and Jerk are not Exceeded - **<span style="color:green; font-size:1em;"> Never exceeded a acceleration >= 10 m/s^2 & a jerk >= 10 m/s^3. </span>**
- Car does not have collisions - **<span style="color:green; font-size:1em;"> No collision detected in the span of 20+ miles run. </span>**
- The car stays in its lane, except for the time between changing lanes - **<span style="color:green; font-size:1em;"> YES, criteria met. </span>**
- The car is able to change lanes - **<span style="color:green; font-size:1em;"> Wherever feasible and required, ECO did the lane change. </span>**
- **<span style="color:green; font-size:1em;"> YES, tested for above 20 miles. </span>**


### Reflection - Model Documentation

Started implementation by taking the STARTER code from [Udacity's CarND-Path-Planning-Project](https://github.com/udacity/CarND-Path-Planning-Project) and lead given in the Project walk through lecture.

The Path Planning code is in src/main.cpp from Line 242 to 432.

**Prediction Part:** This code uses the following inputs : **(1).** Telemetry and **(2).** Sensor fusion data. It makes predictions about the environment we are in, for the following scenarios:

- Whether there a car if in **FRONT** of us blocking the traffic ?
- Whether there a car to the **LEFT** of us making a lane change not safe for car ?
- Whether there a car to the **RIGHT** of us making a lane change not safe for car ?

They are determined by calculating the lane the other car(s) is in and the location it will be at based on the last planned trajectory. A move won't be done if a given car's distance in that lane is less than 30 meters.

The code is from line 250 to 295.

**Behavior Part:** This code part does the following:

- Should we speed UP or speed DOWN ?
- Should we change lanes, if we have a car AHEAD of us ?

Based on the prediction output it will :

- Either increase the speed, or
- Decrease the speed, or
- Make a lane change when it is safe/feasible.

We don't increase the speed in Behavior code area. We maintain a speed difference in **diff_speed** variable, which is used for speed changes when generating the following **Trajectory code part**. This approach makes the car more responsive acting faster to changing situations.

The code is from line 298 to 320.

**Trajectory Part:** Does the trajectory calculation based on :
- Past path points,
- Speed,
- Lane output from the Behavior module,
- Car coordinates

Initially, last 2 points of the previous trajectory or the car's position, if there is no previous trajectory found (Line 334) are used in conjunction with 3 points at a far distance (lines 363 to 365) for spline calculation. For spline calculation, the coordinates are transformed (shifted and rotated) to local car coordinates (Line 375).

To get a continuous trajectory, the earlier trajectory points are copied to the new trajectory (Line 396). The remaining points are calculated by evaluating the spline & then transforming the output coordinates to local coordinates.

For change in velocity of the car (Line 409), the change is decided in Behavior part (298-320), but is used here to increase/decrease speed for every trajectory point.

The code is from line 322 to 436.

----------------------------------------


### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

# Term3-CarND-Path-Planning-Project
# Term3-CarND-Path-Planning-Project
