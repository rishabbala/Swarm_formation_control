# Swarm_formation_control

This repo contains the code for Formation Control for multiple omni-directional robots. I've used PID for the go-to-goal algorithm and Optimal Reciprocal Control Algorithm (ORCA) for collision avoidance. The results are shown below. 

![](6_bot.gif)

![](4_bot.gif)

![](2_bot.gif)

Due to the large computational requirement, the I made changes to the algorithm to take a sub-optimal path in the 6 robot case. The optimization and computational requirements are the major drawback
