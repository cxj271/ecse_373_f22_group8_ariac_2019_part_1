# ecse_373_ariac_2019_part_1

**How to run:**

1. Run the launch file for the simulation environment

```
roslaunch ecse_373_ariac ecse_373_ariac.launch  python:=false&
```
Note: Either use the patch or add the arguement python:=false when launching

This directory and launch file can be cloned from [here](https://github.com/cwru-eecs-373/ecse_373_ariac).

2. Start the competition

```
rosservice info /ariac/start_competition
```

3. Run the node created in starter.cpp

```
rosrun cwru_ecse_373_submission subscriber_node
```
