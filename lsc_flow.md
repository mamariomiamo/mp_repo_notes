# LSC planner implementation flow
## Classes
- param.cpp
    - Loads various parameters
- mission.cpp **Mission**
    - Does mission-related setup: 
        - Setup world octomap point3d; 
        - Get kinodynamic profiles of platform; 
        - Initilize all **agents** container
- multi_sync_simulator.cpp (**MultiSyncSimulator**)

## Flow
launch file will define parameters used for simulation.
launch file will start several nodes"
- rviz for visualization
- octomap server
- multi_sync_simulator_node

### multi_sync_simulator_node
Determine whether runs **Replay** or **Planner** mode.

**Planner mode**:
Instantiate [Param](#Classes) class
Instantiate [Mission](#Classes) class
Instantiate [MultiSyncSimulator](#Classes) class

### Parameters
```multisim/patrol``` --> ```multisim_patrol```: Agents will be in partrol mode to go back and forth between the start and goal points
```experiment``` --> ```multisim_experiment```: Physical implemtation
