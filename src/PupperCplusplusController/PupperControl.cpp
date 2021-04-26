#include <iostream>
#include "ase389/PupperWBC.hpp"


int Task::joint_count = 12;

static std::string urdf_relative_path = "pupper_description/pupper.urdf";

int main(int argc, char** argv){
    PupperWBC Pup;
    Pup.Load(urdf_relative_path);
    return 0;
}