#ifndef WBCTASK_HH_
#define WBCTASK_HH_

#include <vector>

struct Task{
    // Default constructor
    Task(){
        active_tasks = std::vector<bool>(joint_count + 6, false);
        tasks = std::vector<float>(joint_count + 6, 0);
    }
    
    // A list of all targets for the robot state vector in the form
    // V = {xCOM_pow, xCOM_rot, alphaCOM, betaCOM, gammaCOM, q0, q1, q2, ... , qn}
    std::vector<float> tasks;
    
    // Which of the setpoints are active for this task definition
    std::vector<bool> active_tasks;
    
    // Number of joints in the robot
    static int joint_count;
};

#endif