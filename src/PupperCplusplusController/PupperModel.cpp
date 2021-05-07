#include "ase389/PupperModel.h"

std::shared_ptr<RigidBodyDynamics::Model> createPupperModel(){
    auto model = std::make_shared<RigidBodyDynamics::Model>();

    return model;
}
