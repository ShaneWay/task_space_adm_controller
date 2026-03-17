#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <string> 
#include "MathPre.hpp" 
#include "ControlState.h"
#include "BaseParam.h"

class AdmittanceController {
public:
    // Constructor: Now ONLY requires the BaseParam parameter class
    AdmittanceController(const BaseParam& params);

    // Initialize the state (allocates vectors, sets references, and initializes proxy variables)
    void init(ControlState& state, const Eigen::VectorXd& q_init, const Eigen::VectorXd& u_init, const Eigen::VectorXd& tau_init);

    void printParams(ControlState& state);

    // Main control loop update function
    Eigen::VectorXd update(ControlState& state);

    int print_cnt;

private:
    int frame;

    pinocchio::Model model_;
    pinocchio::Data data_;
    BaseParam params_; // Stores system parameters
    
    // Intermediate variables
    Eigen::VectorXd q_x_star;
    Eigen::VectorXd u_x_star;

    // Joint Unwrap Variables (to prevent 360-degree jumps)
    Eigen::VectorXd q_raw_prv_;
    Eigen::VectorXd q_offset_;
    
    // Assuming we are controlling the end-effector, the Frame ID is determined at initialization
    pinocchio::FrameIndex EE_FRAME_ID;
    pinocchio::JointIndex EE_JOINT_ID; 

    // ================== Internal Algorithm Functions ==================
    // Helper function to safely load URDF before initializing pinocchio::Data
    static pinocchio::Model loadModelFromUrdf(const std::string& urdf_path);

    // Helper to unwrap joint angles and maintain continuous trajectories
    Eigen::VectorXd unwrapJointAngles(const Eigen::VectorXd& q_raw);

    // Computes the gravity compensation torque vector
    Eigen::VectorXd computeGravity(const Eigen::VectorXd& q_s);

    void computeTwoProxies(ControlState& state);
    Eigen::VectorXd computeSatPosCtrl(ControlState& state);
    Eigen::MatrixXd continualizedPseudoInverse(const Eigen::MatrixXd& A, double epsilon = 0.03);
};

#endif // CONTROLLER_H