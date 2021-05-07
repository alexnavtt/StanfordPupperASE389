#include <iostream>
#include "ase389/PupperModel.h"

using std::cout;
using std::endl;

using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics;

namespace {
    const double inch2meter = 0.0254;
    const double mm2meter   = 0.001;
    const double density    = 0.3 * 1240; // kg/m^3 with infill of 30%

    Matrix3d scaleInteria(Matrix3d M){
        // Meshlab interia is in mm^2 * mm^3
        // To get the actual inertia in kg*m^2 we have to 
        // multiply by 0.001 ^ 5 * density
        return M * density * pow(mm2meter, 5);
    }

    Matrix3d scaleInteriaInches(Matrix3d M){
        // Meshlab interia is in in^2 * in^3
        // To get the actual inertia in kg*m^2 we have to 
        // multiply by (0.0254) ^ 5 * density
        return M * density * pow(inch2meter, 5);
    }

    Matrix3d getRotation(double x, double y, double z){
        Quaternion q;
        q.fromXYZAngles(Vector3d(x, y, z));
        return q.toMatrix();
    }

}

std::shared_ptr<Model> createPupperModel(){

    auto model = std::make_shared<Model>();

    // Z direction is up
    model->gravity = Vector3d(0, 0, -9.81);

    /* ------ BOTTOM PCB (inches) ------ */

    // Create the bottom PCB
    double pcb_mass = 0.0246155987687;
    Vector3d pcb_com = inch2meter * Vector3d(-0.134500, -0.000013, -0.000000);
    Matrix3d pcb_inertia;
    pcb_inertia << 4.405234, 0,         0, 
                   0,        41.052544, 0,
                   0,        0,         45.448391;
    pcb_inertia = scaleInteriaInches(pcb_inertia);
    
    // Joint between PCB and ROOT
    Joint floating_joint(JointTypeFloatingBase);

    // Add PCB to body
    Body bottom_PCB = Body(pcb_mass, pcb_com, pcb_inertia);
    uint bottom_PCB_id = model->AddBody(0, Xtrans(Vector3d(0, 0, 0)), floating_joint, bottom_PCB, "bottom_PCB");

    /* ------ FRONT BULKHEAD (mm) ------ */

    double front_bulkhead_mass = 28243.179688 * pow(mm2meter,3) * density;
    Vector3d front_bulkhead_com = mm2meter * Vector3d(0.000006, 0.000004, 245.537643);
    Matrix3d front_bulkhead_inertia;
    front_bulkhead_inertia << 27491730.000000,  -18.677107,        -1.684238, 
                             -18.677107,         74592784.000000,  -1.507474,
                              -1.684238,        -1.507474,          83928296.000000;
    front_bulkhead_inertia = scaleInteria(front_bulkhead_inertia);

    // Fixed joint connecting front bulkhead to PCB
    Joint fixed_joint_floating_bulkhead(JointTypeFixed);
    SpatialTransform front_bulkhead_T;
    front_bulkhead_T.E = getRotation(-M_PI_2, 0, 0);
    front_bulkhead_T.r = Vector3d(-0.128, 0.00, 0.044);

    Body front_bulkhead = Body(front_bulkhead_mass, front_bulkhead_com, front_bulkhead_inertia);

    cout << "Before add: " << model->I.back() << endl;
    uint front_bulkhead_id = model->AddBody(bottom_PCB_id, front_bulkhead_T, fixed_joint_floating_bulkhead, front_bulkhead, "front_bulkhead");
    cout << "After add: " << model->I.back() << endl;

    // Test
    cout << "---------- MODEL CREATION TESTING ------------" << endl;
    for (int i = 0; i < model->mBodies.size(); i++){
        cout << "Body " << i << ": " << model->GetBodyName(i) << endl;
    }

    for (int i = 0; i < model->mFixedBodies.size(); i++){
        cout << "Body " << i << ": " << model->GetBodyName(i) << endl;
    }

    for (int i = 0; i < model->I.size(); i++){
        cout << "Inertia " << i << ": " << model->I[i] << endl;
    }

    VectorNd q(6); q.setZero();
    MatrixNd M(6, 6); M.setZero();
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(*model, q, M);
    cout << M << endl;

    return model;
}
