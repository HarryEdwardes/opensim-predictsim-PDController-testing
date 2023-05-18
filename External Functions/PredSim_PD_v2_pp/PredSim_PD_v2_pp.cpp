/*  This code describes the OpenSim model and the skeleton dynamics
    Author: Antoine Falisse
    Contributor: Joris Gillis, Gil Serrancoli, Chris Dembia
*/
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/MultiplierFunction.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Simulation/Model/SmoothSphereHalfSpaceForce.h>
#include "SimTKcommon/internal/recorder.h"

#include <iostream>
#include <iterator>
#include <random>
#include <cassert>
#include <algorithm>
#include <vector>
#include <fstream>

using namespace SimTK;
using namespace OpenSim;

/*  The function F describes the OpenSim model and, implicitly, the skeleton
    dynamics. F takes as inputs joint positions and velocities (states x),
    joint accelerations (controls u), and returns the joint torques as well as
    several variables for use in the optimal control problems. F is templatized
    using type T. F(x,u)->(r).
*/

// Inputs/outputs of function F
/// number of vectors in inputs/outputs of function F
constexpr int n_in = 2; // reference pos reference vel previous gait
constexpr int n_out = 1; // actuator torque
/// number of elements in input/output vectors of function F
constexpr int ndof = 19;        // # degrees of freedom (including locked)
constexpr int NX = ndof * 2;      // # states plus two for references
constexpr int NU = ndof;        // # controls
constexpr int NR = ndof + 4 * 3;    // # residual torques + # joint origins

// Helper function value
template<typename T>
T value(const Recorder& e) { return e; }
template<>
double value(const Recorder& e) { return e.getValue(); }

// OpenSim and Simbody use different indices for the states/controls when the
// kinematic chain has joints up and down the origin (e.g., lumbar joint/arms
// and legs with pelvis as origin).
// The two following functions allow getting the indices from one reference
// system to the other. These functions are inspired from
// createSystemYIndexMap() in Moco.
// getIndicesOSInSimbody() returns the indices of the OpenSim Qs in the Simbody
// reference system. Note that we only care about the order here so we divide
// by 2 because the states include both Qs and Qdots.
SimTK::Array_<int> getIndicesOSInSimbody(const Model& model) {
    auto s = model.getWorkingState();
    const auto svNames = model.getStateVariableNames();
    SimTK::Array_<int> idxOSInSimbody(s.getNQ());
    s.updQ() = 0;
    for (int iy = 0; iy < s.getNQ(); ++iy) {
        s.updQ()[iy] = SimTK::NaN;
        const auto svValues = model.getStateVariableValues(s);
        for (int isv = 0; isv < svNames.size(); ++isv) {
            if (SimTK::isNaN(svValues[isv])) {
                s.updQ()[iy] = 0;
                idxOSInSimbody[iy] = isv/2;
                break;
            }
        }
    }
    return idxOSInSimbody;
}
// getIndicesSimbodyInOS() returns the indices of the Simbody Qs in the OpenSim
// reference system.
SimTK::Array_<int> getIndicesSimbodyInOS(const Model& model) {
    auto idxOSInSimbody = getIndicesOSInSimbody(model);
    auto s = model.getWorkingState();
    SimTK::Array_<int> idxSimbodyInOS(s.getNQ());
	for (int iy = 0; iy < s.getNQ(); ++iy) {
		for (int iyy = 0; iyy < s.getNQ(); ++iyy) {
			if (idxOSInSimbody[iyy] == iy) {
				idxSimbodyInOS[iy] = iyy;
				break;
			}
		}
	}
    return idxSimbodyInOS;
}

// Function F
template<typename T>
int F_generic(const T** arg, T** res) {

    ///////////////////////////////////////////////////////////////////////////
    // This part was automatically generated using buildModelCpp.m with
    // subject1_v2.osim as input.
    // OpenSim: create components
    /// Model
    OpenSim::Model* model;
    /// Bodies
    OpenSim::Body* pelvis;
    OpenSim::Body* femur_r;
    OpenSim::Body* tibia_r;
    OpenSim::Body* talus_r;
    OpenSim::Body* calcn_r;
    OpenSim::Body* toes_r;
    OpenSim::Body* femur_l;
    OpenSim::Body* tibia_l;
    OpenSim::Body* talus_l;
    OpenSim::Body* calcn_l;
    OpenSim::Body* toes_l;
    

    /// Joints
    OpenSim::CustomJoint* ground_pelvis;
    OpenSim::CustomJoint* hip_r;
    OpenSim::CustomJoint* knee_r;
    OpenSim::CustomJoint* ankle_r;
    OpenSim::CustomJoint* subtalar_r;
    OpenSim::WeldJoint* mtp_r;
    OpenSim::CustomJoint* hip_l;
    OpenSim::CustomJoint* knee_l;
    OpenSim::CustomJoint* ankle_l;
    OpenSim::CustomJoint* subtalar_l;
    OpenSim::WeldJoint* mtp_l;
	OpenSim::CustomJoint* knee_r_act;



    /// Model
    model = new OpenSim::Model();
    /// Bodies
	pelvis = new OpenSim::Body("pelvis", 40.1124, Vec3(-0.068278, 0, 0), Inertia(0.3365, 0.3365, 0.1839, 0, 0, 0));
	model->addBody(pelvis);
	femur_r = new OpenSim::Body("femur_r", 6.9838, Vec3(0, -0.17047, 0), Inertia(0.10109, 0.026499, 0.1066, 0, 0, 0));
	model->addBody(femur_r);
	tibia_r = new OpenSim::Body("tibia_r", 2.7837, Vec3(0, -0.18049, 0), Inertia(0.035366, 0.0035787, 0.035857, 0, 0, 0));
	model->addBody(tibia_r);
	talus_r = new OpenSim::Body("talus_r", 0.075084, Vec3(0, 0, 0), Inertia(0.00062714, 0.00062714, 0.00062714, 0, 0, 0));
	model->addBody(talus_r);
	calcn_r = new OpenSim::Body("calcn_r", 0.93854, Vec3(0.091392, 0.027418, 0), Inertia(0.000878, 0.0024459, 0.0025713, 0, 0, 0));
	model->addBody(calcn_r);
	toes_r = new OpenSim::Body("toes_r", 0.16263, Vec3(0.031622, 0.0054836, -0.015994), Inertia(6.2714e-05, 0.00012543, 6.2714e-05, 0, 0, 0));
	model->addBody(toes_r);
	femur_l = new OpenSim::Body("femur_l", 6.9838, Vec3(0, -0.17047, 0), Inertia(0.10109, 0.026499, 0.1066, 0, 0, 0));
	model->addBody(femur_l);
	tibia_l = new OpenSim::Body("tibia_l", 2.7837, Vec3(0, -0.18049, 0), Inertia(0.035366, 0.0035787, 0.035857, 0, 0, 0));
	model->addBody(tibia_l);
	talus_l = new OpenSim::Body("talus_l", 0.075084, Vec3(0, 0, 0), Inertia(0.00062714, 0.00062714, 0.00062714, 0, 0, 0));
	model->addBody(talus_l);
	calcn_l = new OpenSim::Body("calcn_l", 0.93854, Vec3(0.091392, 0.027418, 0), Inertia(0.000878, 0.0024459, 0.0025713, 0, 0, 0));
	model->addBody(calcn_l);
	toes_l = new OpenSim::Body("toes_l", 0.16263, Vec3(0.031622, 0.0054836, 0.015994), Inertia(6.2714e-05, 0.00012543, 6.2714e-05, 0, 0, 0));
	model->addBody(toes_l);
    /// Joints
   //ground_pelvis
	SpatialTransform st_ground_pelvis;
	st_ground_pelvis[0].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tilt", 1, 1));
	st_ground_pelvis[0].setFunction(new LinearFunction());
	st_ground_pelvis[0].setAxis(Vec3(0, 0, 1));
	st_ground_pelvis[1].setCoordinateNames(OpenSim::Array<std::string>("pelvis_list", 1, 1));
	st_ground_pelvis[1].setFunction(new LinearFunction());
	st_ground_pelvis[1].setAxis(Vec3(1, 0, 0));
	st_ground_pelvis[2].setCoordinateNames(OpenSim::Array<std::string>("pelvis_rotation", 1, 1));
	st_ground_pelvis[2].setFunction(new LinearFunction());
	st_ground_pelvis[2].setAxis(Vec3(0, 1, 0));
	st_ground_pelvis[3].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tx", 1, 1));
	st_ground_pelvis[3].setFunction(new LinearFunction());
	st_ground_pelvis[3].setAxis(Vec3(1, 0, 0));
	st_ground_pelvis[4].setCoordinateNames(OpenSim::Array<std::string>("pelvis_ty", 1, 1));
	st_ground_pelvis[4].setFunction(new LinearFunction());
	st_ground_pelvis[4].setAxis(Vec3(0, 1, 0));
	st_ground_pelvis[5].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tz", 1, 1));
	st_ground_pelvis[5].setFunction(new LinearFunction());
	st_ground_pelvis[5].setAxis(Vec3(0, 0, 1));
	ground_pelvis = new CustomJoint("ground_pelvis", model->getGround(), Vec3(0, 0, 0), Vec3(0, 0, 0), *pelvis, Vec3(0, 0, 0), Vec3(0, 0, 0), st_ground_pelvis);
	//hip_r
	SpatialTransform st_hip_r;
	st_hip_r[0].setCoordinateNames(OpenSim::Array<std::string>("hip_flexion_r", 1, 1));
	st_hip_r[0].setFunction(new LinearFunction());
	st_hip_r[0].setAxis(Vec3(0, 0, 1));
	st_hip_r[1].setCoordinateNames(OpenSim::Array<std::string>("hip_adduction_r", 1, 1));
	st_hip_r[1].setFunction(new LinearFunction());
	st_hip_r[1].setAxis(Vec3(1, 0, 0));
	st_hip_r[2].setCoordinateNames(OpenSim::Array<std::string>("hip_rotation_r", 1, 1));
	st_hip_r[2].setFunction(new LinearFunction());
	st_hip_r[2].setAxis(Vec3(0, 1, 0));
	st_hip_r[3].setFunction(new MultiplierFunction(new Constant(0), 0.96574));
	st_hip_r[3].setAxis(Vec3(1, 0, 0));
	st_hip_r[4].setFunction(new MultiplierFunction(new Constant(0), 0.96574));
	st_hip_r[4].setAxis(Vec3(0, 1, 0));
	st_hip_r[5].setFunction(new MultiplierFunction(new Constant(0), 0.986));
	st_hip_r[5].setAxis(Vec3(0, 0, 1));
	hip_r = new CustomJoint("hip_r", *pelvis, Vec3(-0.068278, -0.063835, 0.082331), Vec3(0, 0, 0), *femur_r, Vec3(0, 0, 0), Vec3(0, 0, 0), st_hip_r);
	//knee_r
	SpatialTransform st_knee_r;
	st_knee_r[0].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_r", 1, 1));
	st_knee_r[0].setFunction(new LinearFunction());
	st_knee_r[0].setAxis(Vec3(0, 0, 1));
	st_knee_r[1].setFunction(new Constant(0));
	st_knee_r[1].setAxis(Vec3(0, 1, 0));
	st_knee_r[2].setFunction(new Constant(0));
	st_knee_r[2].setAxis(Vec3(1, 0, 0));
	st_knee_r[3].setFunction(new MultiplierFunction(new Constant(0), 1.0027));
	st_knee_r[3].setAxis(Vec3(1, 0, 0));
	st_knee_r[4].setFunction(new MultiplierFunction(new Constant(0), 1.0027));
	st_knee_r[4].setAxis(Vec3(0, 1, 0));
	st_knee_r[5].setFunction(new MultiplierFunction(new Constant(0), 1.0027));
	st_knee_r[5].setAxis(Vec3(0, 0, 1));
	knee_r = new CustomJoint("knee_r", *femur_r, Vec3(-0.0045122, -0.39691, 0), Vec3(0, 0, 0), *tibia_r, Vec3(0, 0, 0), Vec3(0, 0, 0), st_knee_r);
	//knee_r_actuator
	SpatialTransform st_knee_r_act;
	st_knee_r[0].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_r_act", 1, 1));
	st_knee_r[0].setFunction(new LinearFunction());
	st_knee_r[0].setAxis(Vec3(0, 0, 1));
	st_knee_r[1].setFunction(new Constant(0));
	st_knee_r[1].setAxis(Vec3(0, 1, 0));
	st_knee_r[2].setFunction(new Constant(0));
	st_knee_r[2].setAxis(Vec3(1, 0, 0));
	st_knee_r[3].setFunction(new MultiplierFunction(new Constant(0), 1.0027));
	st_knee_r[3].setAxis(Vec3(1, 0, 0));
	st_knee_r[4].setFunction(new MultiplierFunction(new Constant(0), 1.0027));
	st_knee_r[4].setAxis(Vec3(0, 1, 0));
	st_knee_r[5].setFunction(new MultiplierFunction(new Constant(0), 1.0027));
	st_knee_r[5].setAxis(Vec3(0, 0, 1));
	knee_r_act = new CustomJoint("knee_r_act", *femur_r, Vec3(-0.0045122, -0.39691, 0), Vec3(0, 0, 0), *tibia_r, Vec3(0, 0, 0), Vec3(0, 0, 0), st_knee_r_act);
	//ankle_r
	SpatialTransform st_ankle_r;
	st_ankle_r[0].setCoordinateNames(OpenSim::Array<std::string>("ankle_angle_r", 1, 1));
	st_ankle_r[0].setFunction(new LinearFunction());
	st_ankle_r[0].setAxis(Vec3(-0.10501, -0.17402, 0.97913));
	st_ankle_r[1].setFunction(new Constant(0));
	st_ankle_r[1].setAxis(Vec3(0, 1, 0));
	st_ankle_r[2].setFunction(new Constant(0));
	st_ankle_r[2].setAxis(Vec3(0.97913, 0, 0.10501));
	st_ankle_r[3].setFunction(new MultiplierFunction(new Constant(0), 0.96673));
	st_ankle_r[3].setAxis(Vec3(1, 0, 0));
	st_ankle_r[4].setFunction(new MultiplierFunction(new Constant(0), 0.96673));
	st_ankle_r[4].setAxis(Vec3(0, 1, 0));
	st_ankle_r[5].setFunction(new MultiplierFunction(new Constant(0), 0.96673));
	st_ankle_r[5].setAxis(Vec3(0, 0, 1));
	ankle_r = new CustomJoint("ankle_r", *tibia_r, Vec3(0, -0.41569, 0), Vec3(0, 0, 0), *talus_r, Vec3(0, 0, 0), Vec3(0, 0, 0), st_ankle_r);
	//subtalar_r
	SpatialTransform st_subtalar_r;
	st_subtalar_r[0].setCoordinateNames(OpenSim::Array<std::string>("subtalar_angle_r", 1, 1));
	st_subtalar_r[0].setFunction(new LinearFunction());
	st_subtalar_r[0].setAxis(Vec3(0.78718, 0.60475, -0.12095));
	st_subtalar_r[1].setFunction(new Constant(0));
	st_subtalar_r[1].setAxis(Vec3(0, 1, 0));
	st_subtalar_r[2].setFunction(new Constant(0));
	st_subtalar_r[2].setAxis(Vec3(-0.12095, 0, -0.78718));
	st_subtalar_r[3].setFunction(new MultiplierFunction(new Constant(0), 0.91392));
	st_subtalar_r[3].setAxis(Vec3(1, 0, 0));
	st_subtalar_r[4].setFunction(new MultiplierFunction(new Constant(0), 0.91392));
	st_subtalar_r[4].setAxis(Vec3(0, 1, 0));
	st_subtalar_r[5].setFunction(new MultiplierFunction(new Constant(0), 0.91392));
	st_subtalar_r[5].setAxis(Vec3(0, 0, 1));
	subtalar_r = new CustomJoint("subtalar_r", *talus_r, Vec3(-0.044572, -0.038339, 0.0072383), Vec3(0, 0, 0), *calcn_r, Vec3(0, 0, 0), Vec3(0, 0, 0), st_subtalar_r);
	//mtp_r
	mtp_r = new WeldJoint("mtp_r", *calcn_r, Vec3(0.16341, -0.0018278, 0.00098704), Vec3(0, 0, 0), *toes_r, Vec3(0, 0, 0), Vec3(0, 0, 0));
	//hip_l
	SpatialTransform st_hip_l;
	st_hip_l[0].setCoordinateNames(OpenSim::Array<std::string>("hip_flexion_l", 1, 1));
	st_hip_l[0].setFunction(new LinearFunction());
	st_hip_l[0].setAxis(Vec3(0, 0, 1));
	st_hip_l[1].setCoordinateNames(OpenSim::Array<std::string>("hip_adduction_l", 1, 1));
	st_hip_l[1].setFunction(new LinearFunction());
	st_hip_l[1].setAxis(Vec3(-1, 0, 0));
	st_hip_l[2].setCoordinateNames(OpenSim::Array<std::string>("hip_rotation_l", 1, 1));
	st_hip_l[2].setFunction(new LinearFunction());
	st_hip_l[2].setAxis(Vec3(0, -1, 0));
	st_hip_l[3].setFunction(new MultiplierFunction(new Constant(0), 0.96574));
	st_hip_l[3].setAxis(Vec3(1, 0, 0));
	st_hip_l[4].setFunction(new MultiplierFunction(new Constant(0), 0.96574));
	st_hip_l[4].setAxis(Vec3(0, 1, 0));
	st_hip_l[5].setFunction(new MultiplierFunction(new Constant(0), 0.986));
	st_hip_l[5].setAxis(Vec3(0, 0, 1));
	hip_l = new CustomJoint("hip_l", *pelvis, Vec3(-0.068278, -0.063835, -0.082331), Vec3(0, 0, 0), *femur_l, Vec3(0, 0, 0), Vec3(0, 0, 0), st_hip_l);
	//knee_l
	SpatialTransform st_knee_l;
	st_knee_l[0].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_l", 1, 1));
	st_knee_l[0].setFunction(new LinearFunction());
	st_knee_l[0].setAxis(Vec3(0, 0, 1));
	st_knee_l[1].setFunction(new Constant(0));
	st_knee_l[1].setAxis(Vec3(0, 1, 0));
	st_knee_l[2].setFunction(new Constant(0));
	st_knee_l[2].setAxis(Vec3(1, 0, 0));
	st_knee_l[3].setFunction(new MultiplierFunction(new Constant(0), 1.0027));
	st_knee_l[3].setAxis(Vec3(1, 0, 0));
	st_knee_l[4].setFunction(new MultiplierFunction(new Constant(0), 1.0027));
	st_knee_l[4].setAxis(Vec3(0, 1, 0));
	st_knee_l[5].setFunction(new MultiplierFunction(new Constant(0), 1.0027));
	st_knee_l[5].setAxis(Vec3(0, 0, 1));
	knee_l = new CustomJoint("knee_l", *femur_l, Vec3(-0.0045122, -0.39691, 0), Vec3(0, 0, 0), *tibia_l, Vec3(0, 0, 0), Vec3(0, 0, 0), st_knee_l);
	//ankle_l
	SpatialTransform st_ankle_l;
	st_ankle_l[0].setCoordinateNames(OpenSim::Array<std::string>("ankle_angle_l", 1, 1));
	st_ankle_l[0].setFunction(new LinearFunction());
	st_ankle_l[0].setAxis(Vec3(0.10501, 0.17402, 0.97913));
	st_ankle_l[1].setFunction(new Constant(0));
	st_ankle_l[1].setAxis(Vec3(0, 1, 0));
	st_ankle_l[2].setFunction(new Constant(0));
	st_ankle_l[2].setAxis(Vec3(0.97913, 0, -0.10501));
	st_ankle_l[3].setFunction(new MultiplierFunction(new Constant(0), 0.96673));
	st_ankle_l[3].setAxis(Vec3(1, 0, 0));
	st_ankle_l[4].setFunction(new MultiplierFunction(new Constant(0), 0.96673));
	st_ankle_l[4].setAxis(Vec3(0, 1, 0));
	st_ankle_l[5].setFunction(new MultiplierFunction(new Constant(0), 0.96673));
	st_ankle_l[5].setAxis(Vec3(0, 0, 1));
	ankle_l = new CustomJoint("ankle_l", *tibia_l, Vec3(0, -0.41569, 0), Vec3(0, 0, 0), *talus_l, Vec3(0, 0, 0), Vec3(0, 0, 0), st_ankle_l);
	//subtalar_l
	SpatialTransform st_subtalar_l;
	st_subtalar_l[0].setCoordinateNames(OpenSim::Array<std::string>("subtalar_angle_l", 1, 1));
	st_subtalar_l[0].setFunction(new LinearFunction());
	st_subtalar_l[0].setAxis(Vec3(-0.78718, -0.60475, -0.12095));
	st_subtalar_l[1].setFunction(new Constant(0));
	st_subtalar_l[1].setAxis(Vec3(0, 1, 0));
	st_subtalar_l[2].setFunction(new Constant(0));
	st_subtalar_l[2].setAxis(Vec3(-0.12095, 0, 0.78718));
	st_subtalar_l[3].setFunction(new MultiplierFunction(new Constant(0), 0.91392));
	st_subtalar_l[3].setAxis(Vec3(1, 0, 0));
	st_subtalar_l[4].setFunction(new MultiplierFunction(new Constant(0), 0.91392));
	st_subtalar_l[4].setAxis(Vec3(0, 1, 0));
	st_subtalar_l[5].setFunction(new MultiplierFunction(new Constant(0), 0.91392));
	st_subtalar_l[5].setAxis(Vec3(0, 0, 1));
	subtalar_l = new CustomJoint("subtalar_l", *talus_l, Vec3(-0.044572, -0.038339, -0.0072383), Vec3(0, 0, 0), *calcn_l, Vec3(0, 0, 0), Vec3(0, 0, 0), st_subtalar_l);
	//mtp_l
	mtp_l = new WeldJoint("mtp_l", *calcn_l, Vec3(0.16341, -0.0018278, -0.00098704), Vec3(0, 0, 0), *toes_l, Vec3(0, 0, 0), Vec3(0, 0, 0));

	/// Add joints
	model->addJoint(ground_pelvis);
	model->addJoint(hip_l);
	model->addJoint(hip_r);
	model->addJoint(knee_l);
	model->addJoint(knee_r);
	model->addJoint(ankle_l);
	model->addJoint(ankle_r);
	model->addJoint(subtalar_l);
	model->addJoint(subtalar_r);
	model->addJoint(mtp_l);
	model->addJoint(mtp_r);
	model->addJoint(knee_r_actuator);
	///////////////////////////////////////////////////////////////////////////

	  /// Contact elements
	OpenSim::SmoothSphereHalfSpaceForce* HC_1_r;
	OpenSim::SmoothSphereHalfSpaceForce* HC_2_r;
	OpenSim::SmoothSphereHalfSpaceForce* HC_3_r;
	OpenSim::SmoothSphereHalfSpaceForce* HC_4_r;
	OpenSim::SmoothSphereHalfSpaceForce* HC_5_r;
	OpenSim::SmoothSphereHalfSpaceForce* HC_6_r;
	OpenSim::SmoothSphereHalfSpaceForce* HC_1_l;
	OpenSim::SmoothSphereHalfSpaceForce* HC_2_l;
	OpenSim::SmoothSphereHalfSpaceForce* HC_3_l;
	OpenSim::SmoothSphereHalfSpaceForce* HC_4_l;
	OpenSim::SmoothSphereHalfSpaceForce* HC_5_l;
	OpenSim::SmoothSphereHalfSpaceForce* HC_6_l;
	/// Parameters
	osim_double_adouble radiusSphere = 0.032;
	osim_double_adouble stiffness = 1000000;
	osim_double_adouble dissipation = 2.0;
	osim_double_adouble staticFriction = 0.8;
	osim_double_adouble dynamicFriction = 0.8;
	osim_double_adouble viscousFriction = 0.5;
	osim_double_adouble transitionVelocity = 0.2;
	Vec3 halfSpaceLocation(0);
	Vec3 halfSpaceOrientation(0, 0, -0.5*SimTK::Pi);
	Vec3 locSphere_1_r(0.00190115788407966, -0.021859, -0.00382630379623308);
	Vec3 locSphere_2_r(0.148386399942063, -0.021859, -0.028713422052654);
	Vec3 locSphere_3_r(0.133001170607051, -0.021859, 0.0516362473449566);
	Vec3 locSphere_4_r(0.06, -0.0214476, -0.0187603084619177);
	Vec3 locSphere_5_r(0.0662346661991635, -0.021859, 0.0263641606741698);
	Vec3 locSphere_6_r(0.045, -0.0214476, 0.0618569567549652);
	Vec3 locSphere_1_l(0.00190115788407966, -0.021859, 0.00382630379623308);
	Vec3 locSphere_2_l(0.148386399942063, -0.021859, 0.028713422052654);
	Vec3 locSphere_3_l(0.133001170607051, -0.021859, -0.0516362473449566);
	Vec3 locSphere_4_l(0.06, -0.0214476, 0.0187603084619177);
	Vec3 locSphere_5_l(0.0662346661991635, -0.021859, -0.0263641606741698);
	Vec3 locSphere_6_l(0.045, -0.0214476, -0.0618569567549652);
	/// Left foot contact shere specifications
	HC_1_l = new SmoothSphereHalfSpaceForce("sphere_1_l", *calcn_l, model->getGround());
	HC_1_l->set_contact_sphere_location(locSphere_1_l);
	HC_1_l->set_contact_sphere_radius(radiusSphere);
	HC_1_l->set_contact_half_space_location(halfSpaceLocation);
	HC_1_l->set_contact_half_space_orientation(halfSpaceOrientation);
	HC_1_l->set_stiffness(stiffness);
	HC_1_l->set_dissipation(dissipation);
	HC_1_l->set_static_friction(staticFriction);
	HC_1_l->set_dynamic_friction(dynamicFriction);
	HC_1_l->set_viscous_friction(viscousFriction);
	HC_1_l->set_transition_velocity(transitionVelocity);
	HC_1_l->connectSocket_sphere_frame(*calcn_l);
	HC_1_l->connectSocket_half_space_frame(model->getGround());

	HC_2_l = new SmoothSphereHalfSpaceForce("sphere_2_l", *calcn_l, model->getGround());
	HC_2_l->set_contact_sphere_location(locSphere_2_l);
	HC_2_l->set_contact_sphere_radius(radiusSphere);
	HC_2_l->set_contact_half_space_location(halfSpaceLocation);
	HC_2_l->set_contact_half_space_orientation(halfSpaceOrientation);
	HC_2_l->set_stiffness(stiffness);
	HC_2_l->set_dissipation(dissipation);
	HC_2_l->set_static_friction(staticFriction);
	HC_2_l->set_dynamic_friction(dynamicFriction);
	HC_2_l->set_viscous_friction(viscousFriction);
	HC_2_l->set_transition_velocity(transitionVelocity);
	HC_2_l->connectSocket_sphere_frame(*calcn_l);
	HC_2_l->connectSocket_half_space_frame(model->getGround());

	HC_3_l = new SmoothSphereHalfSpaceForce("sphere_3_l", *calcn_l, model->getGround());
	HC_3_l->set_contact_sphere_location(locSphere_3_l);
	HC_3_l->set_contact_sphere_radius(radiusSphere);
	HC_3_l->set_contact_half_space_location(halfSpaceLocation);
	HC_3_l->set_contact_half_space_orientation(halfSpaceOrientation);
	HC_3_l->set_stiffness(stiffness);
	HC_3_l->set_dissipation(dissipation);
	HC_3_l->set_static_friction(staticFriction);
	HC_3_l->set_dynamic_friction(dynamicFriction);
	HC_3_l->set_viscous_friction(viscousFriction);
	HC_3_l->set_transition_velocity(transitionVelocity);
	HC_3_l->connectSocket_sphere_frame(*calcn_l);
	HC_3_l->connectSocket_half_space_frame(model->getGround());

	HC_4_l = new SmoothSphereHalfSpaceForce("sphere_4_l", *toes_l, model->getGround());
	HC_4_l->set_contact_sphere_location(locSphere_4_l);
	HC_4_l->set_contact_sphere_radius(radiusSphere);
	HC_4_l->set_contact_half_space_location(halfSpaceLocation);
	HC_4_l->set_contact_half_space_orientation(halfSpaceOrientation);
	HC_4_l->set_stiffness(stiffness);
	HC_4_l->set_dissipation(dissipation);
	HC_4_l->set_static_friction(staticFriction);
	HC_4_l->set_dynamic_friction(dynamicFriction);
	HC_4_l->set_viscous_friction(viscousFriction);
	HC_4_l->set_transition_velocity(transitionVelocity);
	HC_4_l->connectSocket_sphere_frame(*toes_l);
	HC_4_l->connectSocket_half_space_frame(model->getGround());

	HC_5_l = new SmoothSphereHalfSpaceForce("sphere_5_l", *calcn_l, model->getGround());
	HC_5_l->set_contact_sphere_location(locSphere_5_l);
	HC_5_l->set_contact_sphere_radius(radiusSphere);
	HC_5_l->set_contact_half_space_location(halfSpaceLocation);
	HC_5_l->set_contact_half_space_orientation(halfSpaceOrientation);
	HC_5_l->set_stiffness(stiffness);
	HC_5_l->set_dissipation(dissipation);
	HC_5_l->set_static_friction(staticFriction);
	HC_5_l->set_dynamic_friction(dynamicFriction);
	HC_5_l->set_viscous_friction(viscousFriction);
	HC_5_l->set_transition_velocity(transitionVelocity);
	HC_5_l->connectSocket_sphere_frame(*calcn_l);
	HC_5_l->connectSocket_half_space_frame(model->getGround());

	HC_6_l = new SmoothSphereHalfSpaceForce("sphere_6_l", *toes_l, model->getGround());
	HC_6_l->set_contact_sphere_location(locSphere_6_l);
	HC_6_l->set_contact_sphere_radius(radiusSphere);
	HC_6_l->set_contact_half_space_location(halfSpaceLocation);
	HC_6_l->set_contact_half_space_orientation(halfSpaceOrientation);
	HC_6_l->set_stiffness(stiffness);
	HC_6_l->set_dissipation(dissipation);
	HC_6_l->set_static_friction(staticFriction);
	HC_6_l->set_dynamic_friction(dynamicFriction);
	HC_6_l->set_viscous_friction(viscousFriction);
	HC_6_l->set_transition_velocity(transitionVelocity);
	HC_6_l->connectSocket_sphere_frame(*toes_l);
	HC_6_l->connectSocket_half_space_frame(model->getGround());

	/// Add left foot contact spheres to model
	model->addComponent(HC_1_l);
	model->addComponent(HC_2_l);
	model->addComponent(HC_3_l);
	model->addComponent(HC_4_l);
	model->addComponent(HC_5_l);
	model->addComponent(HC_6_l);
	/// Right foot contact shere specifications
	HC_1_r = new SmoothSphereHalfSpaceForce("sphere_1_r", *calcn_r, model->getGround());
	HC_1_r->set_contact_sphere_location(locSphere_1_r);
	HC_1_r->set_contact_sphere_radius(radiusSphere);
	HC_1_r->set_contact_half_space_location(halfSpaceLocation);
	HC_1_r->set_contact_half_space_orientation(halfSpaceOrientation);
	HC_1_r->set_stiffness(stiffness);
	HC_1_r->set_dissipation(dissipation);
	HC_1_r->set_static_friction(staticFriction);
	HC_1_r->set_dynamic_friction(dynamicFriction);
	HC_1_r->set_viscous_friction(viscousFriction);
	HC_1_r->set_transition_velocity(transitionVelocity);
	HC_1_r->connectSocket_sphere_frame(*calcn_r);
	HC_1_r->connectSocket_half_space_frame(model->getGround());

	HC_2_r = new SmoothSphereHalfSpaceForce("sphere_2_r", *calcn_r, model->getGround());
	HC_2_r->set_contact_sphere_location(locSphere_2_r);
	HC_2_r->set_contact_sphere_radius(radiusSphere);
	HC_2_r->set_contact_half_space_location(halfSpaceLocation);
	HC_2_r->set_contact_half_space_orientation(halfSpaceOrientation);
	HC_2_r->set_stiffness(stiffness);
	HC_2_r->set_dissipation(dissipation);
	HC_2_r->set_static_friction(staticFriction);
	HC_2_r->set_dynamic_friction(dynamicFriction);
	HC_2_r->set_viscous_friction(viscousFriction);
	HC_2_r->set_transition_velocity(transitionVelocity);
	HC_2_r->connectSocket_sphere_frame(*calcn_r);
	HC_2_r->connectSocket_half_space_frame(model->getGround());

	HC_3_r = new SmoothSphereHalfSpaceForce("sphere_3_r", *calcn_r, model->getGround());
	HC_3_r->set_contact_sphere_location(locSphere_3_r);
	HC_3_r->set_contact_sphere_radius(radiusSphere);
	HC_3_r->set_contact_half_space_location(halfSpaceLocation);
	HC_3_r->set_contact_half_space_orientation(halfSpaceOrientation);
	HC_3_r->set_stiffness(stiffness);
	HC_3_r->set_dissipation(dissipation);
	HC_3_r->set_static_friction(staticFriction);
	HC_3_r->set_dynamic_friction(dynamicFriction);
	HC_3_r->set_viscous_friction(viscousFriction);
	HC_3_r->set_transition_velocity(transitionVelocity);
	HC_3_r->connectSocket_sphere_frame(*calcn_r);
	HC_3_r->connectSocket_half_space_frame(model->getGround());

	HC_4_r = new SmoothSphereHalfSpaceForce("sphere_4_r", *toes_r, model->getGround());
	HC_4_r->set_contact_sphere_location(locSphere_4_r);
	HC_4_r->set_contact_sphere_radius(radiusSphere);
	HC_4_r->set_contact_half_space_location(halfSpaceLocation);
	HC_4_r->set_contact_half_space_orientation(halfSpaceOrientation);
	HC_4_r->set_stiffness(stiffness);
	HC_4_r->set_dissipation(dissipation);
	HC_4_r->set_static_friction(staticFriction);
	HC_4_r->set_dynamic_friction(dynamicFriction);
	HC_4_r->set_viscous_friction(viscousFriction);
	HC_4_r->set_transition_velocity(transitionVelocity);
	HC_4_r->connectSocket_sphere_frame(*toes_r);
	HC_4_r->connectSocket_half_space_frame(model->getGround());

	HC_5_r = new SmoothSphereHalfSpaceForce("sphere_5_r", *calcn_r, model->getGround());
	HC_5_r->set_contact_sphere_location(locSphere_5_r);
	HC_5_r->set_contact_sphere_radius(radiusSphere);
	HC_5_r->set_contact_half_space_location(halfSpaceLocation);
	HC_5_r->set_contact_half_space_orientation(halfSpaceOrientation);
	HC_5_r->set_stiffness(stiffness);
	HC_5_r->set_dissipation(dissipation);
	HC_5_r->set_static_friction(staticFriction);
	HC_5_r->set_dynamic_friction(dynamicFriction);
	HC_5_r->set_viscous_friction(viscousFriction);
	HC_5_r->set_transition_velocity(transitionVelocity);
	HC_5_r->connectSocket_sphere_frame(*calcn_r);
	HC_5_r->connectSocket_half_space_frame(model->getGround());

	HC_6_r = new SmoothSphereHalfSpaceForce("sphere_6_r", *toes_r, model->getGround());
	HC_6_r->set_contact_sphere_location(locSphere_6_r);
	HC_6_r->set_contact_sphere_radius(radiusSphere);
	HC_6_r->set_contact_half_space_location(halfSpaceLocation);
	HC_6_r->set_contact_half_space_orientation(halfSpaceOrientation);
	HC_6_r->set_stiffness(stiffness);
	HC_6_r->set_dissipation(dissipation);
	HC_6_r->set_static_friction(staticFriction);
	HC_6_r->set_dynamic_friction(dynamicFriction);
	HC_6_r->set_viscous_friction(viscousFriction);
	HC_6_r->set_transition_velocity(transitionVelocity);
	HC_6_r->connectSocket_sphere_frame(*toes_r);
	HC_6_r->connectSocket_half_space_frame(model->getGround());

	/// Add right foot contact spheres to model
	model->addComponent(HC_1_r);
	model->addComponent(HC_2_r);
	model->addComponent(HC_3_r);
	model->addComponent(HC_4_r);
	model->addComponent(HC_5_r);
	model->addComponent(HC_6_r);

    // Initialize system and state
    SimTK::State* state;
    state = new State(model->initSystem());

    // Read inputs
    std::vector<T> x(arg[0], arg[0] + NX);
    std::vector<T> u(arg[1], arg[1] + NU);

    // States and controls
	T ua[NU]; /// joint accelerations (Qdotdots) - controls
	Vector QsUs(NX); /// joint positions (Qs) and velocities (Us) - states,

    // Assign inputs to model variables
    /// States
	for (int i = 0; i < NX; ++i) QsUs[i] = x[i];
	
	/// Controls
    /// OpenSim and Simbody have different state orders so we need to adjust
	auto indicesOSInSimbody = getIndicesOSInSimbody(*model);
	for (int i = 0; i < NU; ++i) ua[i] = u[indicesOSInSimbody[i]];

	// Set state variables and realize
	model->setStateVariableValues(*state, QsUs);
	model->realizeVelocity(*state);

	// Compute residual forces
	/// appliedMobilityForces (# mobilities)
	Vector appliedMobilityForces(ndof);
	appliedMobilityForces.setToZero();
	/// appliedBodyForces (# bodies + ground)
	Vector_<SpatialVec> appliedBodyForces;
	int nbodies = model->getBodySet().getSize() + 1;
	appliedBodyForces.resize(nbodies);
	appliedBodyForces.setToZero();
	/// Set gravity
	Vec3 gravity(0);
	gravity[1] = -9.81;
	/// Add weights to appliedBodyForces
	for (int i = 0; i < model->getBodySet().getSize(); ++i) {
		model->getMatterSubsystem().addInStationForce(*state,
			model->getBodySet().get(i).getMobilizedBodyIndex(),
			model->getBodySet().get(i).getMassCenter(),
			model->getBodySet().get(i).getMass()*gravity, appliedBodyForces);
	}

	/// Add contact forces to appliedBodyForces
	/// Right foot
	Array<osim_double_adouble> Force_values_1_r = HC_1_r->getRecordValues(*state);
	Array<osim_double_adouble> Force_values_2_r = HC_2_r->getRecordValues(*state);
	Array<osim_double_adouble> Force_values_3_r = HC_3_r->getRecordValues(*state);
	Array<osim_double_adouble> Force_values_4_r = HC_4_r->getRecordValues(*state);
	Array<osim_double_adouble> Force_values_5_r = HC_5_r->getRecordValues(*state);
	Array<osim_double_adouble> Force_values_6_r = HC_6_r->getRecordValues(*state);
	SpatialVec GRF_1_r;
	GRF_1_r[0] = Vec3(Force_values_1_r[3], Force_values_1_r[4], Force_values_1_r[5]);
	GRF_1_r[1] = Vec3(Force_values_1_r[0], Force_values_1_r[1], Force_values_1_r[2]);
	SpatialVec GRF_2_r;
	GRF_2_r[0] = Vec3(Force_values_2_r[3], Force_values_2_r[4], Force_values_2_r[5]);
	GRF_2_r[1] = Vec3(Force_values_2_r[0], Force_values_2_r[1], Force_values_2_r[2]);
	SpatialVec GRF_3_r;
	GRF_3_r[0] = Vec3(Force_values_3_r[3], Force_values_3_r[4], Force_values_3_r[5]);
	GRF_3_r[1] = Vec3(Force_values_3_r[0], Force_values_3_r[1], Force_values_3_r[2]);
	SpatialVec GRF_4_r;
	GRF_4_r[0] = Vec3(Force_values_4_r[3], Force_values_4_r[4], Force_values_4_r[5]);
	GRF_4_r[1] = Vec3(Force_values_4_r[0], Force_values_4_r[1], Force_values_4_r[2]);
	SpatialVec GRF_5_r;
	GRF_5_r[0] = Vec3(Force_values_5_r[3], Force_values_5_r[4], Force_values_5_r[5]);
	GRF_5_r[1] = Vec3(Force_values_5_r[0], Force_values_5_r[1], Force_values_5_r[2]);
	SpatialVec GRF_6_r;
	GRF_6_r[0] = Vec3(Force_values_6_r[3], Force_values_6_r[4], Force_values_6_r[5]);
	GRF_6_r[1] = Vec3(Force_values_6_r[0], Force_values_6_r[1], Force_values_6_r[2]);
	int ncalcn_r = model->getBodySet().get("calcn_r").getMobilizedBodyIndex();
	int ntoes_r = model->getBodySet().get("toes_r").getMobilizedBodyIndex();
	appliedBodyForces[ncalcn_r] = appliedBodyForces[ncalcn_r] + GRF_1_r + GRF_2_r + GRF_3_r + GRF_5_r;
	appliedBodyForces[ntoes_r] = appliedBodyForces[ntoes_r] + GRF_4_r + GRF_6_r;
	/// Left foot
	Array<osim_double_adouble> Force_values_1_l = HC_1_l->getRecordValues(*state);
	Array<osim_double_adouble> Force_values_2_l = HC_2_l->getRecordValues(*state);
	Array<osim_double_adouble> Force_values_3_l = HC_3_l->getRecordValues(*state);
	Array<osim_double_adouble> Force_values_4_l = HC_4_l->getRecordValues(*state);
	Array<osim_double_adouble> Force_values_5_l = HC_5_l->getRecordValues(*state);
	Array<osim_double_adouble> Force_values_6_l = HC_6_l->getRecordValues(*state);
	SpatialVec GRF_1_l;
	GRF_1_l[0] = Vec3(Force_values_1_l[3], Force_values_1_l[4], Force_values_1_l[5]);
	GRF_1_l[1] = Vec3(Force_values_1_l[0], Force_values_1_l[1], Force_values_1_l[2]);
	SpatialVec GRF_2_l;
	GRF_2_l[0] = Vec3(Force_values_2_l[3], Force_values_2_l[4], Force_values_2_l[5]);
	GRF_2_l[1] = Vec3(Force_values_2_l[0], Force_values_2_l[1], Force_values_2_l[2]);
	SpatialVec GRF_3_l;
	GRF_3_l[0] = Vec3(Force_values_3_l[3], Force_values_3_l[4], Force_values_3_l[5]);
	GRF_3_l[1] = Vec3(Force_values_3_l[0], Force_values_3_l[1], Force_values_3_l[2]);
	SpatialVec GRF_4_l;
	GRF_4_l[0] = Vec3(Force_values_4_l[3], Force_values_4_l[4], Force_values_4_l[5]);
	GRF_4_l[1] = Vec3(Force_values_4_l[0], Force_values_4_l[1], Force_values_4_l[2]);
	SpatialVec GRF_5_l;
	GRF_5_l[0] = Vec3(Force_values_5_l[3], Force_values_5_l[4], Force_values_5_l[5]);
	GRF_5_l[1] = Vec3(Force_values_5_l[0], Force_values_5_l[1], Force_values_5_l[2]);
	SpatialVec GRF_6_l;
	GRF_6_l[0] = Vec3(Force_values_6_l[3], Force_values_6_l[4], Force_values_6_l[5]);
	GRF_6_l[1] = Vec3(Force_values_6_l[0], Force_values_6_l[1], Force_values_6_l[2]);
	int ncalcn_l = model->getBodySet().get("calcn_l").getMobilizedBodyIndex();
	int ntoes_l = model->getBodySet().get("toes_l").getMobilizedBodyIndex();
	appliedBodyForces[ncalcn_l] = appliedBodyForces[ncalcn_l] + GRF_1_l + GRF_2_l + GRF_3_l + GRF_5_l;
	appliedBodyForces[ntoes_l] = appliedBodyForces[ntoes_l] + GRF_4_l + GRF_6_l;
	/// knownUdot
	Vector knownUdot(ndof);
	knownUdot.setToZero();
	for (int i = 0; i < ndof; ++i) knownUdot[i] = ua[i];
	/// Calculate residual forces
	Vector residualMobilityForces(ndof);
	residualMobilityForces.setToZero();
	model->getMatterSubsystem().calcResidualForceIgnoringConstraints(*state,
		appliedMobilityForces, appliedBodyForces, knownUdot,
		residualMobilityForces);

	// Extract several joint origins to set constraints in problem
	Vec3 calcn_or_l = calcn_l->getPositionInGround(*state);
	Vec3 calcn_or_r = calcn_r->getPositionInGround(*state);

	// Extract ground reaction forces
	SpatialVec GRF_r = GRF_1_r + GRF_2_r + GRF_3_r + GRF_4_r + GRF_5_r + GRF_6_r;
	SpatialVec GRF_l = GRF_1_l + GRF_2_l + GRF_3_l + GRF_4_l + GRF_5_l + GRF_6_l;

	// Extract results
	int nc = 3;
	/// Residual forces
	/// OpenSim and Simbody have different state orders so we need to adjust
	auto indicesSimbodyInOS = getIndicesSimbodyInOS(*model);
	for (int i = 0; i < NU; ++i) res[0][i] =
		value<T>(residualMobilityForces[indicesSimbodyInOS[i]]);
	/// ground reaction forces
	for (int i = 0; i < nc; ++i) {
		res[0][i + ndof] = value<T>(GRF_r[1][i]);       /// GRF_r
	}
	for (int i = 0; i < nc; ++i) {
		res[0][i + ndof + nc] = value<T>(GRF_l[1][i]);  /// GRF_l
	}
	/// Joint origins
	for (int i = 0; i < nc; ++i) {
		res[0][i + ndof + nc + nc] = value<T>(calcn_or_r[i]);      /// calcn_or_r
	}
	for (int i = 0; i < nc; ++i) {
		res[0][i + ndof + nc + nc + nc] = value<T>(calcn_or_l[i]); /// calcn_or_l
	}

	return 0;

}


/* In main(), the Recorder is used to save the expression graph of function F.
This expression graph is saved as a MATLAB function named foo.m. From this
function, a c-code can be generated via CasADi and then compiled as a dll. This
dll is then imported in MATLAB as an external function. With this workflow,
CasADi can use algorithmic differentiation to differentiate the function F.
*/
int main() {

    Recorder x[NX];
    Recorder u[NU];
    Recorder tau[NR];

	for (int i = 0; i < NX; ++i) x[i] <<= 0;
	for (int i = 0; i < NU; ++i) u[i] <<= 0;

	const Recorder* Recorder_arg[n_in] = { x,u };
	Recorder* Recorder_res[n_out] = { tau };

    F_generic<Recorder>(Recorder_arg, Recorder_res);

    double res[NR];
    for (int i = 0; i < NR; ++i) Recorder_res[0][i] >>= res[i];

    Recorder::stop_recording();

    return 0;

}
