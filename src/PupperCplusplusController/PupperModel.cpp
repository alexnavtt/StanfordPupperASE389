#include <iostream>
#include "ase389/PupperModel.h"

using std::cout;
using std::endl;

using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics;

#define FIXED Joint(JointTypeFixed)

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


// TODO:    1. Leg shrouds
//          2. Back left leg
//          3. Back right leg
//          4. Front right leg motors
std::shared_ptr<Model> createPupperModel(){

    auto model = std::make_shared<Model>();

    // Z direction is up
    model->gravity = Vector3d(0, 0, -9.81);

    // ============================================================================= //
    //                                   BASE LINK                                   //
    // ============================================================================= //


    //////////////////////////////////
    //            BODIES            //
    //////////////////////////////////

    /* ------ BOTTOM PCB (inches) ------ */

    // Create the bottom PCB
    double pcb_mass = 0.0246155987687;
    Vector3d pcb_com = inch2meter * Vector3d(-0.134500, -0.000013, -0.000000);
    Matrix3d pcb_inertia;
    pcb_inertia << 4.405234, 0,         0, 
                   0,        41.052544, 0,
                   0,        0,         45.448391;
    pcb_inertia = scaleInteriaInches(pcb_inertia);
    Body bottom_PCB = Body(pcb_mass, pcb_com, pcb_inertia);

    /* ------ FRONT BULKHEAD (mm) ------ */

    double front_bulkhead_mass = 28243.179688 * pow(mm2meter,3) * density;
    Vector3d front_bulkhead_com = mm2meter * Vector3d(0.000006, 0.000004, 245.537643);
    Matrix3d front_bulkhead_inertia;
    front_bulkhead_inertia << 27491730.000000,  -18.677107,        -1.684238, 
                             -18.677107,         74592784.000000,  -1.507474,
                              -1.684238,        -1.507474,          83928296.000000;
    front_bulkhead_inertia = scaleInteria(front_bulkhead_inertia);
    Body front_bulkhead(front_bulkhead_mass, front_bulkhead_com, front_bulkhead_inertia);

    /* ------ MIDDLE BULKHEAD 1 (mm) ------ */

    double middle_bh_1_mass = 33243.015625 * pow(mm2meter,3) * density;
    Vector3d middle_bh_1_com = mm2meter * Vector3d(0.050467, -0.533781, 202.751328);
    Matrix3d middle_bh_1_inertia;
    middle_bh_1_inertia << 22996456.000000,  -192.887802,      -639.832642,
                          -192.887802,        51008240.000000, -22193.009766,
                          -639.832642,       -22193.009766,    73639128.000000;
    middle_bh_1_inertia = scaleInteria(middle_bh_1_inertia);
    Body middle_bh_1(middle_bh_1_mass, middle_bh_1_com, middle_bh_1_inertia);

    /* ------ MIDDLE BULKHEAD 2 (mm) ------ */

    double middle_bh_2_mass = 33262.445312 * pow(mm2meter,3) * density;
    Vector3d middle_bh_2_com = mm2meter * Vector3d(-0.147800, 0.000000, 127.256111);
    Matrix3d middle_bh_2_inertia;
    middle_bh_2_inertia << 21912060.000000, -49.972462,       -10377.245117,
                          -49.972462,        40583024.000000, -0.003996,
                          -10377.245117,    -0.003996,         62043808.000000;
    middle_bh_2_inertia = scaleInteria(middle_bh_2_inertia);
    Body middle_bh_2(middle_bh_2_mass, middle_bh_2_com, middle_bh_2_inertia);

    /* ------ REAR BULKHEAD (mm) ------ */

    double rear_bh_mass = 33735.734375 * pow(mm2meter,3) * density;
    Vector3d rear_bh_com = mm2meter * Vector3d(-0.004437, -0.593278, 2.799114);
    Matrix3d rear_bh_inertia;
    rear_bh_inertia << 23357426.000000, -4462.781738,      427.257507,
                      -4462.781738,      52151916.000000, -17892.154297,
                       427.257507,      -17892.154297,     75135296.000000;
    rear_bh_inertia = scaleInteria(rear_bh_inertia);
    Body rear_bh(rear_bh_mass, rear_bh_com, rear_bh_inertia);

    /* ------ LEFT SHROUD (mm) ------ */

    double left_shroud_mass = 15699.697266 * pow(mm2meter,3) * density;
    Vector3d left_shroud_com = mm2meter * Vector3d(56.354546, -0.000083, 67.307045);
    Matrix3d left_shroud_inertia;
    left_shroud_inertia <<  30641502.000000, -101.878265,       1374721.250000,
                           -101.878265,       21280568.000000,  78.329567,
                            1374721.250000,   78.329567,        10692940.000000;
    left_shroud_inertia = scaleInteria(left_shroud_inertia);
    Body left_shroud(left_shroud_mass, left_shroud_com, left_shroud_inertia);

    /* ------ RIGHT SHROUD (mm) ------ */

    double right_shroud_mass = 15699.833984 * pow(mm2meter,3) * density;
    Vector3d right_shroud_com = Vector3d(-56.354591, 0.000044, 67.307060);
    Matrix3d right_shroud_inertia;
    right_shroud_inertia << 30641556.000000, -117.941475,      -1374718.500000,
                           -117.941475,       21280556.000000, -71.618393,
                           -1374718.500000,  -71.618393,        10692993.000000;
    right_shroud_inertia = scaleInteria(right_shroud_inertia);
    Body right_shroud(right_shroud_mass, right_shroud_com, right_shroud_inertia);

    /* ------ BATTERY (m) ------ */

    double batt_mass = 0.228;
    Vector3d batt_com(0, 0, 0);
    Matrix3d batt_inertia;
    batt_inertia << 6.796775e-05, 0,             0,
                    0,            0.00013779275, 0,
                    0,            0,             0.000116375;
    Body battery(batt_mass, batt_com, batt_inertia);
    

    //////////////////////////////////
    //            JOINTS            //
    //////////////////////////////////

    // Joint between PCB and ROOT
    Joint floating_joint(JointTypeFloatingBase);
    uint bottom_PCB_id = model->AddBody(0, Xtrans(Vector3d(0, 0, 0)), floating_joint, bottom_PCB, "bottom_PCB");

    // Fixed joint connecting front bulkhead to PCB
    SpatialTransform front_bulkhead_T;
    front_bulkhead_T.E = getRotation(-M_PI_2, 0, 0);
    front_bulkhead_T.r = Vector3d(-0.128, 0.00, 0.044);
    uint front_bulkhead_id = model->AddBody(bottom_PCB_id, front_bulkhead_T, FIXED, front_bulkhead, "front_bulkhead");

    // Fixed joint connecting middle bulkhead to PCB
    SpatialTransform middle_bh_1_T;
    middle_bh_1_T.E = getRotation(M_PI_2, 0, M_PI_2);
    middle_bh_1_T.r = Vector3d(-0.128, 0.00, 0.044);
    uint middle_bh_1_id = model->AddBody(bottom_PCB_id, middle_bh_1_T, FIXED, middle_bh_1, "middle_bulkhead_front");

    // Fixed joint connecting middle bulkhead 2 to PCB
    SpatialTransform middle_bh_2_T;
    middle_bh_2_T.E = getRotation(M_PI_2, 0, M_PI_2);
    middle_bh_2_T.r = Vector3d(-0.128, 0.00, 0.044);
    uint middle_bh_2_id = model->AddBody(bottom_PCB_id, middle_bh_2_T, FIXED, middle_bh_2, "middle_bulkhead_rear");

    // Fixed joint connecting rear bulkhed to PCB
    SpatialTransform rear_bh_T;
    rear_bh_T.E = getRotation(M_PI_2, 0, M_PI_2);
    rear_bh_T.r = Vector3d(-0.128, 0.00, 0.044);
    uint rear_bh_id = model->AddBody(bottom_PCB_id, rear_bh_T, FIXED, rear_bh, "rear_bulkhead");

    // Fixed joint connecting left shroud to PCB
    SpatialTransform left_shroud_T;
    left_shroud_T.E = getRotation(M_PI_2, 0, M_PI_2);
    left_shroud_T.r = Vector3d(-0.128, 0.00, 0.044);
    uint left_shroud_id = model->AddBody(bottom_PCB_id, left_shroud_T, FIXED, left_shroud, "left_shroud");

    // Fixed joint connecting right shroud to PCB
    SpatialTransform right_shroud_T;
    right_shroud_T.E = getRotation(M_PI, 0, 0);
    right_shroud_T.r = Vector3d(0.007, 0.085, 0);
    uint right_shroud_id = model->AddBody(bottom_PCB_id, right_shroud_T, FIXED, right_shroud, "right_shroud");
    
    // Fixed joint connecting battery to PCB
    SpatialTransform battery_T;
    battery_T.E = getRotation(0, 0, M_PI_2);
    battery_T.r = Vector3d(-0.028, 0.0, 0.02425);
    uint battery_id = model->AddBody(bottom_PCB_id, battery_T, FIXED, battery, "battery");
    




    
    

    // ============================================================================= //
    //                                   LEGS                                        //
    // ============================================================================= //
    

    //////////////////////////////////
    //            BODIES            //
    //////////////////////////////////


    /* ------  MOTOR (m) ------ */

    double motor_mass = 0.12;
    Vector3d motor_com = Vector3d(0, 0, 0);
    Matrix3d motor_inertia;
    motor_inertia << 7.2e-6, 0,       0,
                     0,      1.11e-5, 0,
                     0,      0,       1.11e-5;
    Body motor(motor_mass, motor_com, motor_inertia);
    
    /* ------ LEFT HUB (mm) ------ */

    double left_hub_mass = 7095.798340 * pow(mm2meter,3) * density;
    Vector3d left_hub_com = mm2meter * Vector3d(5.921877, 0.000081, -12.887457);
    Matrix3d left_hub_inertia;
    left_hub_inertia << 1456708.875000,  12.194215,      -446692.343750,
                        12.194215,       1472451.125000,  4.978187,
                       -446692.343750,   4.978187,        1155731.875000;
    left_hub_inertia  = scaleInteria(left_hub_inertia);
    Body left_hub(left_hub_mass, left_hub_com, left_hub_inertia);

    /* ------ RIGHT HUB (mm) ------ */

    double right_hub_mass = 7095.798340 * pow(mm2meter,3) * density;
    Vector3d right_hub_com = mm2meter * Vector3d(5.921877, 0.000081, 12.887457);
    Matrix3d right_hub_inertia;
    right_hub_inertia << 1456708.875000,  12.194215,        446692.343750,
                         12.194215,       1472451.125000,  -4.978187,
                         446692.343750,   -4.978187,        1155731.875000;
    right_hub_inertia  = scaleInteria(right_hub_inertia);
    Body right_hub(right_hub_mass, right_hub_com, right_hub_inertia);

    /* ------  UPPER LINK (mm) ------ */ 
    double upper_link_mass = 13804.297852 * pow(mm2meter,3) * density;
    Vector3d upper_link_com = mm2meter * Vector3d(4.592267, 43.607891, 0.000017);
    Matrix3d upper_link_inertia;
    upper_link_inertia << 14050550.000000, 156734.750000,  0.007331,
                          156734.750000,   957646.812500, -6.240734,
                          0.007331,       -6.240734,       13227597.000000;
    upper_link_inertia = scaleInteria(upper_link_inertia);
    Body upper_link = Body(upper_link_mass, upper_link_com, upper_link_inertia);

    /* ------ LEFT LOWER LINK (mm) ------ */

    double left_lower_link_mass = 13170.648438 * pow(mm2meter,3) * density;
    Vector3d left_lower_link_com = mm2meter * Vector3d(-0.310779, -51.736179, -4.568526);
    Matrix3d left_lower_link_inertia;
    left_lower_link_inertia << 18151954.000000, -238616.937500, -19946.636719,
                              -238616.937500,    437603.937500, -672611.562500,
                              -19946.636719,    -672611.562500,  18307054.000000;
    left_lower_link_inertia = scaleInteria(left_lower_link_inertia);
    Body left_lower_link(left_lower_link_mass, left_lower_link_com, left_lower_link_inertia);
    
    /* ------ FRONT RIGHT LOWER LINK (mm) ------ */

    double right_lower_link_mass = 13170.648438 * pow(mm2meter,3) * density;
    Vector3d right_lower_link_com = mm2meter * Vector3d(-0.310779, -51.736179, 4.568526);
    Matrix3d right_lower_link_inertia;
    right_lower_link_inertia << 18151954.000000, -238616.937500, 19946.636719,
                              -238616.937500,    437603.937500, 672611.562500,
                              19946.636719,    672611.562500,  18307054.000000;
    right_lower_link_inertia = scaleInteria(right_lower_link_inertia);
    Body right_lower_link(right_lower_link_mass, right_lower_link_com, right_lower_link_inertia);

    /* ------ UPPER LINK SHROUD (mm) ------ */
    double leg_shroud_mass = 6413.942871 * pow(mm2meter,3) * density;
    Vector3d leg_shroud_com = mm2meter * Vector3d(12.682721, 54.673603, 0.000087);
    Matrix3d leg_shroud_inertia;
    leg_shroud_inertia << 4268927.000000, 171794.656250,  0.706674,
                          171794.656250,  991662.187500, -24.252905,
                          0.706674,      -24.252905,      3827615.750000;
    leg_shroud_inertia = scaleInteria(leg_shroud_inertia);
    Body leg_shroud(leg_shroud_mass, leg_shroud_com, leg_shroud_inertia);

    /* ------ FOOT (mm) ------ */
    double foot_mass = 0.01;
    Vector3d foot_com = mm2meter * Vector3d(0.000083, 0.000001, 14.012547);
    Matrix3d foot_inertia;
    foot_inertia << 3.61e-07, 0,        0,
                    0,        3.61e-07, 0,
                    0,        0,        3.61e-07;
    Body foot = Body(foot_mass, foot_com, foot_inertia);

    
    //////////////////////////////////
    //            JOINTS            //
    //////////////////////////////////


    /* ---------- BACK LEFT LEG ---------- */

    // Fixed joint connecting back left motor to PCB
    SpatialTransform back_left_motor_T;
    back_left_motor_T.E = getRotation(0, M_PI_2, 0);
    back_left_motor_T.r = Vector3d(-0.110, 0.045, 0.041);
    model->AddBody(bottom_PCB_id, back_left_motor_T, FIXED, motor, "back_left_hip_motor");

    // Revolute joint connecting back left hub to PCB
    Joint back_left_hub_joint(JointTypeRevolute, Vector3d(-1, 0, 0));
    SpatialTransform back_left_hub_T;
    back_left_hub_T.E = getRotation(-M_PI_2, 0, 0);
    back_left_hub_T.r = Vector3d(-0.147, 0.045, 0.041);
    uint back_left_hub_id = model->AddBody(bottom_PCB_id, back_left_hub_T, back_left_hub_joint, left_hub, "back_left_hub");

    // Fixed joint connecting back left shoulder motor to back left hub
    SpatialTransform back_left_shoulder_motor_T;
    back_left_shoulder_motor_T.E = getRotation(0, 0, 0);
    back_left_shoulder_motor_T.r = Vector3d(0, 0, 0.005);
    model->AddBody(back_left_hub_id, back_left_shoulder_motor_T, FIXED, motor, "back_left_shoulder_motor");

    // Revolute joint connecting back left upper link to back left hub
    Joint back_left_shoulder_joint(JointTypeRevolute, Vector3d(1, 0, 0));
    SpatialTransform back_left_shoulder_T;
    back_left_shoulder_T.E = getRotation(0, -M_PI_2, 0);
    back_left_shoulder_T.r = Vector3d(0, 0, 0.022);
    uint back_left_upper_link_id = model->AddBody(back_left_hub_id, back_left_shoulder_T, back_left_shoulder_joint, upper_link, "back_left_upper_link");

    // Fixed joint connecting leg shroud to back left lower link
    SpatialTransform shroud_T;
    shroud_T.E = getRotation(M_PI, 0, 0);
    shroud_T.r = Vector3d(0.007, 0.085, 0);
    model->AddBody(back_left_upper_link_id, shroud_T, FIXED, leg_shroud, "back_left_leg_shroud");

    // Revolute joint connecting back left lower link to back left upper link
    Joint back_left_elbow_joint(JointTypeRevolute, Vector3d(0, 0, -1));
    SpatialTransform back_left_elbow_T;
    back_left_elbow_T.E = getRotation(0, -M_PI_2, M_PI);
    back_left_elbow_T.r = Vector3d(-0.006, 0.08, 0);
    uint back_left_lower_link_id = model->AddBody(back_left_upper_link_id, back_left_elbow_T, back_left_elbow_joint, left_lower_link, "back_left_lower_link");

    // Fixed joint connecting motor to back left upper joint
    SpatialTransform back_left_elbow_motor_T;
    back_left_elbow_motor_T.E = getRotation(0, M_PI_2, 0);
    back_left_elbow_motor_T.r = Vector3d(0.023, 0.082, 0);
    model->AddBody(back_left_upper_link_id, back_left_elbow_motor_T, FIXED, motor, "back_left_elbow_motor");

    // Fixed joint connecting foot to back left lower link
    SpatialTransform back_left_foot_T;
    back_left_foot_T.E = getRotation(0, 0, 0);
    back_left_foot_T.r = Vector3d(0, -0.11, 0.009);
    model->AddBody(back_left_lower_link_id, back_left_foot_T, FIXED, foot, "back_left_foot");


    /* ---------- BACK RIGHT LEG ---------- */

    // Fixed joint connecting back right motor to PCB
    SpatialTransform back_right_motor_T;
    back_right_motor_T.E = getRotation(3*M_PI_2, 0, M_PI_2);
    back_right_motor_T.r = Vector3d(-0.110, -0.045, 0.041);
    model->AddBody(bottom_PCB_id, back_right_motor_T, FIXED, motor, "back_right_hip_motor");

    // Revolute joint connecting back right hub to PCB
    Joint back_right_hip_joint(JointTypeRevolute, Vector3d(-1, 0, 0));
    SpatialTransform back_right_hip_T;
    back_right_hip_T.E = getRotation(-M_PI_2, 0, 0);
    back_right_hip_T.r = Vector3d(-0.147, -0.045, 0.041);
    uint back_right_hub_id = model->AddBody(bottom_PCB_id, back_right_hip_T, back_right_hip_joint, right_hub, "back_right_hub");

    // Fixed joint connecting back right shoulder motor to back right hub
    SpatialTransform back_right_shoulder_motor_T;
    back_right_shoulder_motor_T.E = getRotation(0, 0, 0);
    back_right_shoulder_motor_T.r = Vector3d(0, 0, -0.005);
    model->AddBody(back_right_hub_id, back_right_shoulder_motor_T, FIXED, motor, "back_right_shoulder_motor");

    // Revolute joint connecting back right top link to back right hub
    Joint back_right_shoulder_joint(JointTypeRevolute, Vector3d(1, 0, 0));
    SpatialTransform back_right_shoulder_T;
    back_right_shoulder_T.E = getRotation(0, M_PI_2, 0);
    back_right_shoulder_T.r = Vector3d(0, 0, -0.02);
    uint back_right_upper_link_id = model->AddBody(back_right_hub_id, back_right_shoulder_T, back_right_shoulder_joint, upper_link, "back_right_upper_link");

    // Fixed joint connecting back right elbow motor to back right upper link
    SpatialTransform back_right_elbow_motor_T;
    back_right_elbow_motor_T.E = getRotation(0, M_PI_2, 0);
    back_right_elbow_motor_T.r = Vector3d(0.023, 0.082, 0);
    model->AddBody(back_right_upper_link_id, back_right_elbow_motor_T, FIXED, motor, "back_right_elbow_motor");

    // Fixed joint connecting leg shroud to back right lower link
    model->AddBody(back_right_upper_link_id, shroud_T, FIXED, leg_shroud, "back_right_leg_shroud");

    // Revolute joint connecting back right lower link to back right upper link
    Joint back_right_elbow_joint(JointTypeRevolute, Vector3d(0, 0, 1));
    SpatialTransform back_right_elbow_T;
    back_right_elbow_T.E = getRotation(0, M_PI_2, M_PI);
    back_right_elbow_T.r = Vector3d(-0.006, 0.08, 0);
    uint back_right_lower_link_id = model->AddBody(back_right_upper_link_id, back_right_elbow_T, back_right_elbow_joint, right_lower_link, "back_right_lower_link");

    // Fixed joint connecting back right foot to back right lower link
    SpatialTransform back_right_foot_T;
    back_right_foot_T.E = getRotation(0, 0, 0);
    back_right_foot_T.r = Vector3d(0, -0.11, -0.009);
    model->AddBody(back_right_lower_link_id, back_right_foot_T, FIXED, foot, "back_right_foot");



    /* ---------- FRONT LEFT LEG ---------- */

    // Fixed joint connecting front left motor to PCB
    SpatialTransform front_left_motor_T;
    front_left_motor_T.E = getRotation(0, M_PI_2, 0);
    front_left_motor_T.r = Vector3d(0.091, 0.045, 0.041);
    uint front_left_motor_id = model->AddBody(bottom_PCB_id, front_left_motor_T, FIXED, motor, "front_left_motor");

    // Revolute joint connecting front left hub to PCB (FL HIP)
    Joint front_left_hub_joint(JointTypeRevolute, Vector3d(-1, 0, 0));
    SpatialTransform front_left_hub_T;
    front_left_hub_T.E = getRotation(-M_PI_2, 0, 0);
    front_left_hub_T.r = Vector3d(0.054, 0.045, 0.041);
    uint front_left_hub_id = model->AddBody(bottom_PCB_id, front_left_hub_T, front_left_hub_joint, left_hub, "front_left_hub");             

    // Fixed joint connecting front left shoulder motor to front left hub
    SpatialTransform front_left_shoulder_motor_T;
    front_left_shoulder_motor_T.E = getRotation(0, 0, 0);
    front_left_shoulder_motor_T.r = Vector3d(0, 0, 0.005);
    model->AddBody(front_left_hub_id, front_left_shoulder_motor_T, FIXED, motor, "front_left_shoulder_motor");

    // Revolute joint connecting front left upper link to front left hub (FL SHOULDER)
    Joint front_left_shoulder_joint(JointTypeRevolute, Vector3d(1, 0, 0));
    SpatialTransform front_left_shoulder_T;
    front_left_shoulder_T.E = getRotation(0, -M_PI_2, 0);   
    front_left_shoulder_T.r = Vector3d(0, 0, 0.022);       
    uint front_left_upper_link_id = model->AddBody(front_left_hub_id, front_left_shoulder_T, front_left_shoulder_joint, upper_link, "front_left_upper_link");

    // Fixed joint connecting elbow motor to front left upper link
    SpatialTransform front_left_elbow_motor_T;
    front_left_elbow_motor_T.E = getRotation(0, M_PI_2, 0);
    front_left_elbow_motor_T.r = Vector3d(0.023, 0.082, 0);
    model->AddBody(front_left_upper_link_id, front_left_elbow_motor_T, FIXED, motor, "front_left_elbow_motor");

    // Fixed joint connecting leg shroud to front left lower link
    model->AddBody(front_left_upper_link_id, shroud_T, FIXED, leg_shroud, "front_left_leg_shroud");

    // Revolute joint connecting front left upper link and front left lower link (FL ELBOW)
    Joint front_left_elbow_joint = Joint(JointTypeRevolute, Vector3d(0, 0, -1));
    SpatialTransform front_left_elbow_T;
    front_left_elbow_T.E = getRotation(0, -M_PI_2, M_PI);
    front_left_elbow_T.r = Vector3d(-0.006, 0.08, 0);
    uint front_left_lower_link_id = model->AddBody(front_left_upper_link_id, front_left_elbow_T, front_left_elbow_joint, left_lower_link, "front_left_lower_link");
    
    // Fixed joint connecting foot to lower link
    SpatialTransform left_foot_T;
    left_foot_T.E = getRotation(0, 0, 0);
    left_foot_T.r = Vector3d(0, -0.11, 0.009);
    model->AddBody(front_left_lower_link_id, left_foot_T, FIXED, foot, "front_left_foot");



    /* ---------- FRONT RIGHT LEG ---------- */

    // Fixed joint connecting front right motor to PCB
    SpatialTransform front_right_motor_T;
    front_right_motor_T.E = getRotation(0, M_PI_2, 0);
    front_right_motor_T.r = Vector3d(0.091, -0.045, 0.041);
    uint front_right_motor_id = model->AddBody(bottom_PCB_id, front_right_motor_T, FIXED, motor, "front_right_motor");

    // Revolute joint connecting front right hub to PCB (FR HIP)
    Joint front_right_hip_joint(JointTypeRevolute, Vector3d(-1, 0, 0));
    SpatialTransform front_right_hub_T;
    front_right_hub_T.E = getRotation(-M_PI_2, 0, 0);
    front_right_hub_T.r = Vector3d(0.054, -0.045, 0.041);
    uint front_right_hub_id = model->AddBody(bottom_PCB_id, front_right_hub_T, front_right_hip_joint, right_hub, "front_right_hub");

    // Fixed joint connecting motor to front right hub
    SpatialTransform front_right_shoulder_motor_T;
    front_right_shoulder_motor_T.E = getRotation(0, 0, 0);
    front_right_shoulder_motor_T.r = Vector3d(0, 0, -0.005);
    model->AddBody(front_right_hub_id, front_right_shoulder_motor_T, FIXED, motor, "front_right_shoulder_motor");

    // Revolute joint connecting front right upper link to front right hub (FR SHOULDER)
    Joint front_right_shoulder_joint(JointTypeRevolute, Vector3d(1, 0, 0));
    SpatialTransform front_right_upper_link_T;
    front_right_upper_link_T.E = getRotation(0, M_PI_2, 0);
    front_right_upper_link_T.r = Vector3d(0, 0, -0.02);
    uint front_right_upper_link_id = model->AddBody(front_right_hub_id, front_right_upper_link_T, front_right_shoulder_joint, upper_link, "front_right_upper_link");

    // Fixed joint connecting elbow motor with front rigth upper link
    SpatialTransform front_right_elbow_motor_T;
    front_right_elbow_motor_T.E = getRotation(0, M_PI_2, 0);
    front_right_elbow_motor_T.r = Vector3d(0.023, 0.082, 0);
    model->AddBody(front_right_upper_link_id, front_right_elbow_motor_T, FIXED, motor, "front_right_elbow_motor");

    // Fixed joint connecting leg shroud to front right lower link
    model->AddBody(front_right_upper_link_id, shroud_T, FIXED, leg_shroud, "front_right_leg_shroud");

    // Revolute joint connection front right lower link to front right upper link (FR ELBOW)
    Joint front_right_elbow_joint(JointTypeRevolute, Vector3d(0, 0, 1));
    SpatialTransform front_right_elbow_T;
    front_right_elbow_T.E = getRotation(0, M_PI_2, M_PI);
    front_right_elbow_T.r = Vector3d(-0.006, 0.08, 0);
    uint front_right_lower_link_id = model->AddBody(front_right_upper_link_id, front_right_elbow_T, front_right_elbow_joint, right_lower_link, "front_right_lower_link");

    // Fixed joint connecting front right foot to front right lower link
    SpatialTransform front_right_foot_T;
    front_right_foot_T.E = getRotation(0, 0, 0);
    front_right_foot_T.r = Vector3d(0, -0.11, -0.009);
    uint front_right_foot_id = model->AddBody(front_right_lower_link_id, front_right_foot_T, FIXED, foot, "front_right_foot");



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
    q(10) = M_PI_2;

    for (int i = 2; i < model->mBodies.size(); i++){
        MatrixNd J(6, model->qdot_size); J.setZero();
        CalcPointJacobian6D(*model, q, i, zero_offset, J);
        for (int j = 0; j < J.size(); j++){
            if (abs(J.data()[j]) < 1e-5) J.data()[j] = 0;
        }
        cout << "\nJacobian for " << model->GetBodyName(i) << ": \n" << J << endl;
    }

    std::vector<const char*> points_to_test = {"front_right_hub", "front_right_upper_link", "front_right_lower_link", "front_right_foot"};
    for (auto body : points_to_test){
        auto pos = CalcBodyToBaseCoordinates(*model, q, model->GetBodyId(body), zero_offset);
        cout << "Location of " << body << " in base coordinates: (" << pos[0] << ", " << pos[1] << ", "  << pos[2] << ")\n";
    }


    cout << "\n---------- DYNAMICS TESTING ------------\n" << endl;

    MatrixNd M(model->qdot_size, model->qdot_size); M.setZero();
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(*model, q, M);
    cout << "\nMass Matrix: \n" << M << endl;

    return model;
}



// Old tests saved for posterity:
// std::shared_ptr<Model> createPupperModel(){

//     auto model = std::make_shared<Model>();

//     // Z direction is up
//     model->gravity = Vector3d(0, 0, -9.81);

// //* -----------------------------TESTING---------------------------------*//
// // Answering questions:
// // 1. For fixed bodies, do the intertias have to be in the same frame or no?
// //     a. Join two bodies with the same frame -> print mass matrix
// //     b. Join two bodies with different frames -> print mass matrix
// //     c. mass matrix different? Then inertia's DONT have to be in the same frame. (GOOD)
// //        Answer: Yes, inertia's don't have to be in the same frame (the inertia of the child is described in the child's frame and can differ from the parent frame) 
// // 2. COM described in parent or child??
// //     a. Add body at 0,0,0 with COM at 0,0,0 -> print mass matrix
// //     b. Add body at 1000,1000,1000 with COM at 0,0,0 -> print mass matrix
// //     c. Mass matrix different? Then COM is described in child frame. (GOOD)
// //        Answer: Yes, COM of added body is described in child frame.
//  /* ------ BOTTOM PCB (inches) ------ */
//     // TEST 1 /////////////////////////

//     // Create the bottom PCB
//     double pcb_mass = 0.0246155987687;
//     Vector3d pcb_com = inch2meter * Vector3d(-0.134500, -0.000013, -0.000000);
//     Matrix3d pcb_inertia;
//     pcb_inertia << 10,       0,          0, 
//                    0,        10,         0,
//                    0,        0,         10;
    
//     // Joint between PCB and ROOT
//     Joint floating_joint(JointTypeFloatingBase);

//     // Add PCB to body
//     Body bottom_PCB = Body(pcb_mass, pcb_com, pcb_inertia);
//     uint bottom_PCB_id = model->AddBody(0, Xtrans(Vector3d(0, 0, 0)), floating_joint, bottom_PCB, "bottom_PCB");

//     /* ------ TEST BODY 2 (mm) ------ */

//     double front_bulkhead_mass = 28243.179688 * pow(mm2meter,3) * density;
//     Vector3d front_bulkhead_com = mm2meter * Vector3d(0, 0, 0);
//     Matrix3d front_bulkhead_inertia;
//     front_bulkhead_inertia << 10,       0,          0, 
//                               0,        0,          0,
//                                0,        0,         0;

//     // Fixed joint connecting TEST BODY 2 to PCB
//     Joint fixed_joint_floating_bulkhead(JointTypeFixed);
//     SpatialTransform front_bulkhead_T;
//     front_bulkhead_T.E << 1, 0, 0, // 1.a  2.a,b
//                           0, 1, 0,
//                           0, 0, 1; 
//     // front_bulkhead_T.E << 0,-1, 0, // 1.b
//     //                       1, 0, 0,
//     //                       0, 0, 1; 
//     cout << "TestBody2 T.E = \n" << front_bulkhead_T.E <<endl;
//     //front_bulkhead_T.r = Vector3d(0, 0, 0); // 1.a,b ,  2.a
//     front_bulkhead_T.r = Vector3d(1000, 1000, 1000); // 2.b

//     // Add TEST BODY 2 to body
//     Body front_bulkhead = Body(front_bulkhead_mass, front_bulkhead_com, front_bulkhead_inertia);
//     uint front_bulkhead_id = model->AddBody(bottom_PCB_id, front_bulkhead_T, fixed_joint_floating_bulkhead, front_bulkhead, "front_bulkhead");

//     ///////////////////////////////////////////////////////////////
//     cout << "\n---------- DYNAMICS TESTING ------------\n" << endl;
//     VectorNd q(model->q_size); q.setZero();
//     MatrixNd M(model->qdot_size, model->qdot_size); M.setZero();
//     RigidBodyDynamics::CompositeRigidBodyAlgorithm(*model, q, M);
//     cout << "\nMass Matrix: \n" << M << endl;


// ////////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////RESULTS////////////////////////////////////////////////////
// //
// ////////////////////////////////////////////////////////////////////////////////////////////////