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
        q = q.fromXYZAngles(Vector3d(x, y, z));
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

    // Add front_bulkhead to body
    Body front_bulkhead = Body(front_bulkhead_mass, front_bulkhead_com, front_bulkhead_inertia);
    uint front_bulkhead_id = model->AddBody(bottom_PCB_id, front_bulkhead_T, fixed_joint_floating_bulkhead, front_bulkhead, "front_bulkhead");

    /* ------ MIDDLE BULKHEAD 1 (mm) ------ */
    double middle_bh_1_mass = 33243.015625 * pow(mm2meter,3) * density;
    Vector3d middle_bh_1_com = mm2meter * Vector3d(0.050467, -0.533781, 202.751328);
    Matrix3d middle_bh_1_inertia;
    middle_bh_1_inertia << 22996456.000000,  -192.887802,      -639.832642,
                          -192.887802,        51008240.000000, -22193.009766,
                          -639.832642,       -22193.009766,    73639128.000000;
    middle_bh_1_inertia = scaleInteria(middle_bh_1_inertia);

    // Fixed joint connecting middle bulkhead to PCB
    Joint middle_bh_1_joint(JointTypeFixed);
    SpatialTransform middle_bh_1_T;
    middle_bh_1_T.E = getRotation(M_PI_2, 0, M_PI_2);
    middle_bh_1_T.r = Vector3d(-0.128, 0.00, 0.044);

    // Add middle bulkhead to body
    Body middle_bh_1 = Body(middle_bh_1_mass, middle_bh_1_com, middle_bh_1_inertia);
    uint middle_bh_1_id = model->AddBody(bottom_PCB_id, middle_bh_1_T, middle_bh_1_joint, middle_bh_1, "middle_bulkhead_1");

    
    /* ------ FRONT LEFT HUB ------ */
    double left_hub_mass = 7095.798340 * pow(mm2meter,3) * density;
    Vector3d left_hub_com = mm2meter * Vector3d(5.921877, 0.000081, -12.887457);
    Matrix3d left_hub_inertia;
    left_hub_inertia << 1456708.875000,  12.194215,      -446692.343750,
                        12.194215,       1472451.125000,  4.978187,
                       -446692.343750,   4.978187,        1155731.875000;
    left_hub_inertia  = scaleInteria(left_hub_inertia);

    // Revolute joint connecting hub to PCB
    Joint front_left_hub_joint(JointTypeRevolute, Vector3d(-1, 0, 0));
    SpatialTransform front_left_hub_T;
    front_left_hub_T.E = getRotation(-M_PI_2, 0, 0);
    front_left_hub_T.r = Vector3d(0.054, 0.045, 0.041);

    // Add hub to PCB with revolute joint
    Body left_hub = Body(left_hub_mass, left_hub_com, left_hub_inertia);
    uint front_left_hub_id = model->AddBody(bottom_PCB_id, front_left_hub_T, front_left_hub_joint, left_hub, "front_left_hub");             

    /* ------ FRONT LEFT UPPER LINK ------ */
    double upper_link_mass = 13804.297852 * pow(mm2meter,3) * density;
    Vector3d upper_link_com = mm2meter * Vector3d(4.592267, 43.607891, 0.000017);
    Matrix3d upper_link_inertia;
    upper_link_inertia << 14050550.000000, 156734.750000,  0.007331,
                          156734.750000,   957646.812500, -6.240734,
                          0.007331,       -6.240734,       13227597.000000;
    upper_link_inertia = scaleInteria(upper_link_inertia);

    // Revolute joint connecting upper link to hub
    Joint front_left_shoulder_joint(JointTypeRevolute, Vector3d(1, 0, 0));
    SpatialTransform front_left_shoulder_T;
    front_left_shoulder_T.E = getRotation(0, -M_PI_2, 0);
    front_left_shoulder_T.r = Vector3d(0, 0, 0.022);

    // Add upper link to hub with revolute joint
    Body front_left_upper_link = Body(upper_link_mass, upper_link_com, upper_link_inertia);
    uint front_left_upper_link_id = model->AddBody(front_left_hub_id, front_left_shoulder_T, front_left_shoulder_joint, front_left_upper_link, "front_left_upper_link");


    /* ------ FRONT LEFT LOWER LINK ------ */
    double left_lower_link_mass = 13170.648438 * pow(mm2meter,3) * density;
    Vector3d left_lower_link_com = mm2meter * Vector3d(-0.310779, -51.736179, -4.568526);
    Matrix3d left_lower_link_inertia;
    left_lower_link_inertia << 18151954.000000, -238616.937500, -19946.636719,
                              -238616.937500,    437603.937500, -672611.562500,
                              -19946.636719,    -672611.562500,  18307054.000000;
    left_lower_link_inertia = scaleInteria(left_lower_link_inertia);

    // Revolute joint connecting upper link and lower link
    Joint front_left_elbow_joint = Joint(JointTypeRevolute, Vector3d(0, 1, 0));
    SpatialTransform front_left_elbow_T;
    front_left_elbow_T.E = getRotation(0, M_PI_2, 0);
    front_left_elbow_T.r = Vector3d(0.023, 0.082, 0);

    // Add lower link to upper link with a revolute joint
    Body front_left_lower_link = Body(left_lower_link_mass, left_lower_link_com, left_lower_link_inertia);
    uint front_left_lowr_link_id = model->AddBody(front_left_upper_link_id, front_left_elbow_T, front_left_elbow_joint, front_left_lower_link, "front_left_lower_link");


    // Test
    cout << "---------- MODEL CREATION TESTING ------------\n" << endl;
    cout << "Model has " << model->mJoints.size() << " joints\n";

    cout << "All bodies:\n";
    for (int i = 0; i < model->mBodies.size(); i++){
        cout << "Body " << i << ": " << model->GetBodyName(i) << endl;
    }
    cout << "\nFixed bodies:\n";
    for (int i = 0; i < model->mFixedBodies.size(); i++){
        cout << "Body " << i << ": " << model->GetBodyName(i) << endl;
    }
    cout << "\nAll masses:\n";
    double total_mass = 0;
    for (int i = 0; i < model->mBodies.size(); i++){
        cout << "Mass of " << model->GetBodyName(i) << ": " << model->mBodies[i].mMass << "kg\n";
        total_mass += model->mBodies[i].mMass;
    }
    cout << "Total mass: " << total_mass << " kg" << endl;

    cout << "\n---------- KINEMATICS TESTING ------------\n" << endl;
    VectorNd q(model->q_size); q.setZero();
    Vector3d zero_offset(0, 0, 0);
    // q(7) = M_PI_2;

    for (int i = 2; i < model->mBodies.size(); i++){
        MatrixNd J(6, model->qdot_size); J.setZero();
        CalcPointJacobian6D(*model, q, i, zero_offset, J);
        for (int j = 0; j < J.size(); j++){
            if (abs(J.data()[j]) < 1e-5) J.data()[j] = 0;
        }
        cout << "\nJacobian for " << model->GetBodyName(i) << ": \n" << J << endl;
    }

    std::vector<const char*> points_to_test = {"front_left_hub", "front_left_upper_link", "front_left_lower_link"};
    for (auto body : points_to_test){
        auto pos = CalcBodyToBaseCoordinates(*model, q, model->GetBodyId(body), zero_offset);
        pos += model->mBodies[model->GetBodyId(body)].mCenterOfMass;
        cout << "Location of " << body << " in base coordinates: (" << pos[0] << ", " << pos[1] << ", "  << pos[2] << ")\n";
    }


    cout << "\n---------- DYNAMICS TESTING ------------\n" << endl;

    MatrixNd M(model->qdot_size, model->qdot_size); M.setZero();
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(*model, q, M);
    cout << "\nMass Matrix: \n" << M << endl;

    return model;
}
