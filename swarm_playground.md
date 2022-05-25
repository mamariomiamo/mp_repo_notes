# Single UAV
**autoflight.launch**
This contains two main parts:
1) random_forest (**random_forest_sensing.cpp**) defines the map properties
2) **run_in_sim.xml**
    - Planner: **advanced_param.xml**
        - Starts ego_planner_node that calls EGOReplanFSM class (`rebo_replan`) that starts:
            - planner_manager
                - PolyTrajOptimizer class (`ploy_traj_opt_`)
                - GridMap class (`grid_map_`)
            - visualization
    - trajectory server: **traj_server**
    - simulator: **simulator.xml**
        - sensing_horizon (5 m) set here

# Multi-UAV