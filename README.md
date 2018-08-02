# Autonomous Parking Planning
This work is about planning the trajectory for autonoumous vehicles in the parking sceniro. The vehicle movement is based the reeds-shepp curve and the planning algorithm is the hybrid A* algorithm. And we show you two parking scenes, parallel parking and backward parking.
## The reeds-shepp curve
The reeds-shepp curve is the basic movement for bicycle model vehicles in the obstacle free environment. It's usually used in the movement unit in the motion planning problem.
## The A* and hybrid A* algorithm
I think the basic theory of the algorithm is simple for every one. What I want to focus on is the kd-tree and the priority queue in the algorithm implement. The kd-tree code is down load from leet code and something wrong were corrected and the basic *inrange* function is attached.
## The effect
The algorithm is not efficient and the construction of the obstacle map both with the policy map consume most of the time. For the convience of debug, I saved this data as .pkl file.

The following is the animination

![backwards parking](https://github.com/arkria/ParkingPlann/blob/master/figure/backwards.gif)

