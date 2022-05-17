# Single UAV
**autoflight.launch**
This contains two main parts:
1) random_forest (**random_forest_sensing.cpp**) defines the map properties
2) **run_in_sim.xml**
    - Planner: **advanced_param.xml**
    - trajectory server: **traj_server**
    - simulator: **simulator.xml**
        - sensing_horizon (5 m) set here

# Multi-UAV