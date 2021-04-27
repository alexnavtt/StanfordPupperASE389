#ifndef WBCTASK_HH_
#define WBCTASK_HH_

#include <vector>
#include <string>
#include <array>

struct Task{    
    // Which body this task is for (use JOINT for a joint position task)
    std::string body_id;

    // What type of task is this (body_pos, body_ori, or joint_pos)
    std::string type;

    // The desired task goal and which to consider for this task
    std::vector<bool> active_targets;
    std::vector<float> targets;
};

#endif