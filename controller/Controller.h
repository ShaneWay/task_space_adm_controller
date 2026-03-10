#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include "MathPre.hpp" 
#include "ControlState.h"
#include "BaseParam.h"

class AdmittanceController {
public:
    // Constructor: Now only requires the robot model and the BaseParam parameter class
    AdmittanceController(const pinocchio::Model& model, const BaseParam& params);

    // Initialize the state (initializes _prv variables by passing the state reference)
    void init(ControlState& state);

    // Main control loop update function
    // state: Contains all measured feedback states and the proxy states to be updated
    // tau_s: Currently measured external joint torque
    // Returns: Calculated desired joint torque tau_m (command sent to the motors)
    Eigen::VectorXd update(ControlState& state, const Eigen::VectorXd& tau_s);

private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    BaseParam params_; // Stores system parameters
    
    // Intermediate variables, kept within the class, no need to expose to the external State
    Eigen::VectorXd q_x_star;
    Eigen::VectorXd u_x_star;
    
    // Assuming we are controlling the end-effector, the Frame ID is determined at initialization
    const int EE_FRAME_ID = 1; 
    const pinocchio::JointIndex EE_JOINT_ID = 1; 

    // ================== Internal Algorithm Functions ==================
    // Passes the state reference to directly read and modify internal state data
    void computeTwoProxies(ControlState& state, const Eigen::VectorXd& tau_s);
    
    Eigen::VectorXd computeSatPosCtrl(ControlState& state);

    // Continualized pseudoinverse algorithm
    Eigen::MatrixXd continualizedPseudoInverse(const Eigen::MatrixXd& A, double epsilon = 0.03);
};

#endif // CONTROLLER_H