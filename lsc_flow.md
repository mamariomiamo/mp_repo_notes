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

### multi_sync_simulator.cpp
**Related to planning:** Initialize an instance of **TrajPlanner** class for each agent.
```
        agents.resize(mission.qn);
        for(int qi = 0; qi < mission.qn; qi++){
            agents[qi] = std::make_unique<TrajPlanner>(qi, nh, param, mission);
            dynamic_msgs::State initial_state;
            initial_state.pose.position = point3DToPointMsg(mission.agents[qi].start_position);
            agents[qi]->setCurrentState(initial_state);
            agents[qi]->initializeROS();
        }
```

**```update()```** update ideal, current states and obstacles' state for each agent.

**Related to obstacle:** 

**ToDo:** Only update obstacles' state that is in the agent's sensing range in . 

Since LSC requires obstacles/ other agents' trajectories. Following snippet is used to get obstacles' id, type, pose (position and orientation), velocity goal_point, radius, max_acc, downwash trajectories
```
            // Other agents
            for(int qj = 0; qj < mission.qn; qj++){
                if(qi == qj){
                    continue;
                }

                dynamic_msgs::Obstacle obstacle;
                obstacle.id = qj;
                obstacle.type = ObstacleType::AGENT;

                // If diff between ideal and observed position is small, then use ideal position
                octomap::point3d ideal_agent_curr_position = pointMsgToPoint3d(ideal_agent_curr_states[qj].pose.position);
                if((ideal_agent_curr_position - real_agent_curr_positions[qj]).norm() > param.multisim_reset_threshold)
                {
                    obstacle.pose.position = point3DToPointMsg(real_agent_curr_positions[qj]);
                    obstacle.pose.orientation = defaultQuaternion();
                    obstacle.velocity.linear = defaultVector();
                }
                else {
                    obstacle.pose = ideal_agent_next_states[qj].pose;
                    obstacle.velocity = ideal_agent_next_states[qj].velocity;
                }

                std::cout << "agent index" << obstacle.id << "at position" << obstacle.pose.position << "observed by agent" << qi << std::endl;

                obstacle.goal_point = point3DToPointMsg(agents[qj]->getDesiredGoalPosition());
                obstacle.radius = mission.agents[qj].radius;
                obstacle.max_acc = mission.agents[qj].max_acc[0];
                obstacle.downwash = mission.agents[qj].downwash;
                msg_obstacles.obstacles.emplace_back(obstacle);

                traj_t obs_traj = agents[qj]->getTraj();
                obs_prev_trajs.emplace_back(obs_traj);
            }
```
**```setObstacles(msg_obstacles)```** is then called to create ***dynamic_msgs::ObstacleArray*** object which will be used for priority planning (**```goalPlanningWithPriority()```** and so on).

**```setObsPrevTrajs(obs_prev_trajs)```** is then called to create a vector of traj_t object which will be used to set **```obs_pred_trajs```** and for LSC generation.

**NOTE**
You may wonder why when number of agents increase, the simulation slows down significantly. This is because the implementation of the currrent algorithm uses this node to update states for all agents using a **FOOR LOOP**.

```
        // Compute ideal state of agents
        std::vector<dynamic_msgs::State> ideal_agent_curr_states;
        std::vector<dynamic_msgs::State> ideal_agent_next_states;
        ideal_agent_curr_states.resize(mission.qn);
        ideal_agent_next_states.resize(mission.qn);
        for(int qi = 0; qi < mission.qn; qi++) {
```

```
        // Measure current state of objects
        std::vector<octomap::point3d> real_agent_curr_positions;
        real_agent_curr_positions.resize(mission.qn);
        for(int qi = 0; qi < mission.qn; qi++) {
```

```
        // Update agent's states
        // If difference between ideal and observed position is small, then use ideal position
        for(int qi = 0; qi < mission.qn; qi++) {
```
```
        // Update obstacle's states to each agent
        for(int qi = 0; qi < mission.qn; qi++){
```


### traj_planner.cpp
#### LSC (```generateLSC()```)
LSC is generated for each segment of the whole trajectory using the initial trajectory **```initial_traj_trans```** and **```obs_pred_traj_trans```** by calling **```normalVectorBetweenPolys```** to compute the normal vector and Eqn 29 in paper to compute the safety margin.

**```obs_pred_traj_trans```** is transformed from **```obs_pred_trajs```** which is set from **```obs_prev_trajs```** (i.e. previous trajectories).

Again **```obs_prev_trajs```** (previous trajectories) was set from the [multi_sync_simulator.cpp](#multisyncsimulatorcpp)

For loop iterate over all obstacles which is 


#### SFC
Calls **`corridor_constructor.expandBoxFromPoint(...)`** to generate SFCs. 
Original implementation (in **`corridor_constructor.hpp`**) expands a box along x,y,z directions from current position and stops when there is obstacle (**`isObstacleInBox(...)`**) or out of map boundary (**`isBoxInBoundary(...)`**). It is computationally expensive to evaluate until the boundary of the map. It is faster when we only look for SFC within the sensor range and thus **`isBoxInSensing(...)`** is implemented (last argument *sensor_range* gives the side length of sensing cube).