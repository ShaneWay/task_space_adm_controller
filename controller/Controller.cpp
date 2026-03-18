#include "Controller.h"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp> 
#include <Eigen/SVD> 
#include <iostream>

pinocchio::Model AdmittanceController::loadModelFromUrdf(const std::string& urdf_path) {
    pinocchio::Model model;
    try {
        pinocchio::urdf::buildModel(urdf_path, model);
        std::cout << "[SUCCESS] Robot URDF loaded successfully from: " << urdf_path << "\n";
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load URDF: " << e.what() << "\n";
    }
    return model;
}

AdmittanceController::AdmittanceController(const BaseParam& params)
    : model_(loadModelFromUrdf(params.urdf_path)),
      data_(model_), 
      params_(params) {
    
    // Initialize intermediate variables within the class
    q_x_star = Eigen::VectorXd::Zero(params_.nv);
    u_x_star = Eigen::VectorXd::Zero(params_.nv);

    // Verify consistency between the loaded URDF and YAML parameters
    if (params_.nv != model_.nv) {
        std::cerr << "[WARNING] DoF mismatch! YAML nv: " << params_.nv 
                  << ", URDF nv: " << model_.nv << "\n";
    }

        EE_JOINT_ID = model_.joints.size() - 1; 

    if (model_.existFrame("EndEffector_Link")) {
        EE_FRAME_ID = model_.getFrameId("EndEffector_Link");
    } else {
        // 退回默认的最后一个 Frame
        EE_FRAME_ID = model_.frames.size() - 1; 
    }
}

void AdmittanceController::init(ControlState& state, const Eigen::VectorXd& q_init, const Eigen::VectorXd& u_init, const Eigen::VectorXd& tau_init) {

    print_cnt = 0;
    // 1. Initialize unwrap tracking variables
    state.initSizes(params_.nv);
    q_raw_prv_ = q_init;
    q_offset_.setZero(params_.nv);

    // 2. Initialize measured states (unwrap the very first frame)
    state.q_s = unwrapJointAngles(q_init);
    state.u_s = u_init;
    state.tau_s = tau_init;
    state.tau = Eigen::VectorXd::Zero(params_.nv);
    state.tau_ext = Eigen::VectorXd::Zero(params_.nv);
    
    state.q_r = params_.q_r;
    state.p_r = params_.p_r;
    state.f_r = params_.f_r;
    
    state.u_r = Eigen::VectorXd::Zero(params_.nv);
    state.aq_r = Eigen::VectorXd::Zero(params_.nv);
    state.v_r = Eigen::VectorXd::Zero(6);
    state.a_r = Eigen::VectorXd::Zero(6);
    
    state.q_x_prv = state.q_s;
    state.u_x_prv = state.u_s;
    state.b_xs_prv = Eigen::VectorXd::Zero(params_.nv);
    
    pinocchio::forwardKinematics(model_, data_, state.q_s);

    pinocchio::updateFramePlacements(model_, data_);
    state.p_x_prv = data_.oMf[EE_FRAME_ID];

    frame = 0;
    std::cout << "[DEBUG] finish init " << std::endl;

    printParams(state);
}

void AdmittanceController::printParams(ControlState& state)
{
        std::cout << "---------------- [Core Parameters] ---------------\n";
        std::cout << "Timestep (T) : " << params_.T << " s\n";
        std::cout << "DoF (nv)     : " << params_.nv << "\n\n";
        
        std::cout << "------------- [Task Space Parameters] ------------\n";
        // Print the diagonal elements of the matrices and transpose them to row vectors for readability
        std::cout << "M_T (diag)   : [" << params_.M_T.diagonal().transpose() << " ]\n";
        std::cout << "B_T (diag)   : [" << params_.B_T.diagonal().transpose() << " ]\n";
        std::cout << "K_T (diag)   : [" << params_.K_T.diagonal().transpose() << " ]\n";
        std::cout << "F_T limits   : [" << params_.F_T.transpose() << " ]\n\n";
        
        std::cout << "------------ [Joint Space Parameters] ------------\n";
        std::cout << "M (diag)     : [" << params_.M.diagonal().transpose() << " ]\n";
        std::cout << "B (diag)     : [" << params_.B.diagonal().transpose() << " ]\n";
        std::cout << "K (diag)     : [" << params_.K.diagonal().transpose() << " ]\n";
        std::cout << "F limits     : [" << params_.F.transpose() << " ]\n\n";
        
        std::cout << "------------- [SatPosCtrl Parameters] ------------\n";
        std::cout << "K_c          : [" << params_.K_c.transpose() << " ]\n";
        std::cout << "B_c          : [" << params_.B_c.transpose() << " ]\n";
        std::cout << "L_c          : [" << params_.L_c.transpose() << " ]\n";
        std::cout << "F_c limits   : [" << params_.F_c.transpose() << " ]\n\n";

        std::cout << "------------- [Reference Parameters] -------------\n";
        std::cout << "q_r          : [" << params_.q_r.transpose() << " ]\n";
        std::cout << "f_r          : [" << params_.f_r.transpose() << " ]\n\n";
        
        std::cout << "p_r (Translation) :\n" << params_.p_r.translation().transpose() << "\n\n";
        std::cout << "p_r (Rotation Matrix) :\n" << params_.p_r.rotation() << "\n";
        std::cout << "==================================================\n";

        std::cout << "------------- [State Parameters] -------------\n";
        std::cout << "q_s          : [" << state.q_s.transpose() << " ]\n";
        std::cout << "u_s          : [" << state.u_s.transpose() << " ]\n\n";
        std::cout << "q_x_prv          : [" << state.q_x_prv.transpose() << " ]\n";
        std::cout << "u_x_prv          : [" << state.u_x_prv.transpose() << " ]\n\n";

}

// ==========================================================
// Joint Angle Unwrap Logic
// ==========================================================
Eigen::VectorXd AdmittanceController::unwrapJointAngles(const Eigen::VectorXd& q_raw) {
    for (int i = 0; i < params_.nv; ++i) {
        double delta = q_raw(i) - q_raw_prv_(i);
        
        // If it drops from 2*PI to 0, add an offset
        if (delta < -M_PI) { 
            q_offset_(i) += 2.0 * M_PI;
        } 
        // If it jumps from 0 to 2*PI, subtract an offset
        else if (delta > M_PI) { 
            q_offset_(i) -= 2.0 * M_PI;
        }
        
        q_raw_prv_(i) = q_raw(i);
    }
    return q_raw + q_offset_;
}

Eigen::MatrixXd AdmittanceController::continualizedPseudoInverse(const Eigen::MatrixXd& A, double epsilon) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd sigmas = svd.singularValues();
    Eigen::VectorXd sigmas_inv(sigmas.size());
    
    for (int i = 0; i < sigmas.size(); ++i) {
        if (std::abs(sigmas(i)) > epsilon) {
            sigmas_inv(i) = 1.0 / sigmas(i);
        } else {
            sigmas_inv(i) = sigmas(i) / (epsilon * epsilon);
        }
    }
    return svd.matrixV() * sigmas_inv.asDiagonal() * svd.matrixU().transpose();
}

void AdmittanceController::computeTwoProxies(ControlState& state) {
    // 1. Get Jacobians
    Eigen::MatrixXd J_s = MathPre::jacobian(model_, data_, state.q_s, EE_FRAME_ID);
    Eigen::MatrixXd J_x_prv = MathPre::jacobian(model_, data_, state.q_x_prv, EE_FRAME_ID);

    state.f_s = J_s.transpose().jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(state.tau_ext);
    
    // 2. Get Jacobian time variation
    Eigen::MatrixXd H_x_prv = MathPre::jacobiansTimeVariation(model_, data_, state.q_x_prv, state.u_x_prv, EE_FRAME_ID);
    
    // 3. Compute J_hat_x
    Eigen::MatrixXd J_hat_x = J_x_prv + params_.T * H_x_prv;
    
    // 4. Compute M^{-h}
    Eigen::MatrixXd M_h = params_.M.llt().matrixL();
    Eigen::MatrixXd M_inv_h = M_h.inverse();
    
    // 5. Compute C_T and C_J
    Eigen::MatrixXd C_T = M_inv_h * J_s.transpose() * (params_.M_T + params_.T * params_.B_T) * J_hat_x;
    Eigen::MatrixXd C_J = M_inv_h * (params_.M + params_.T * params_.B);
    
    // 6. Compute f_re and tau_re
    Eigen::VectorXd err_p = Eigen::VectorXd::Zero(6);
    err_p.head<3>() = state.p_r.translation() - state.p_x_prv.translation();
    // R_err = R_target * R_current^T. pinocchio::log3 finds the shortest rotation vector
    Eigen::Matrix3d R_err = state.p_r.rotation() * state.p_x_prv.rotation().transpose();
    err_p.tail<3>() = pinocchio::log3(R_err);
    Eigen::VectorXd f_re = params_.M_T * state.a_r + params_.B_T * state.v_r + MathPre::sat3(params_.F_T, params_.K_T * err_p);
    
    Eigen::VectorXd err_q = state.q_r - state.q_x_prv;
    Eigen::VectorXd tau_re = params_.M * state.aq_r + params_.B * state.u_r + MathPre::sat1(params_.F, params_.K * err_q);
    
    // 7. Compute b_T and b_J 
    // (Note: tau_ext passed here is the pure external torque after removing gravity)
    Eigen::VectorXd b_T = M_inv_h * (J_s.transpose() * (-params_.B_T * state.v_x_prv - (params_.M_T + params_.T * params_.B_T) * H_x_prv * state.u_x_prv + state.f_r + f_re) + state.tau_ext);
    Eigen::VectorXd b_J = M_inv_h * (-params_.B * state.u_x_prv + tau_re + state.tau_ext);
    
    // 8. Solve the continualized pseudoinverse
    Eigen::MatrixXd C_J_inv = C_J.inverse();
    Eigen::MatrixXd C_TJ = C_T * C_J_inv;
    Eigen::MatrixXd C_TJ_pinv = continualizedPseudoInverse(C_TJ, 0.03); 
    Eigen::MatrixXd I_n = Eigen::MatrixXd::Identity(params_.nv, params_.nv);
    
    Eigen::VectorXd alpha_x_star = C_J_inv * C_TJ_pinv * b_T + C_J_inv * (I_n - C_TJ_pinv * C_TJ) * b_J;
    
    // 9. Update u_x_star and q_x_star (saved internally in the class)
    u_x_star = state.u_x_prv + params_.T * alpha_x_star;
    q_x_star = state.q_x_prv + params_.T * u_x_star;
}

Eigen::VectorXd AdmittanceController::computeSatPosCtrl(ControlState& state) {
    // 1. Compute tau_m_star_star
    Eigen::VectorXd term1 = (state.u_s - (state.q_s - state.q_x_prv) / params_.T);
    Eigen::VectorXd tau_m_star_star = (params_.L_c.array() * state.b_xs_prv.array()) - (params_.B_c.array() * term1.array());
    
    // 2. Compute tau_m_star
    Eigen::VectorXd tau_m_star = ((params_.K_c.array() + params_.B_c.array() / params_.T + params_.L_c.array() * params_.T) * (q_x_star - state.q_s).array()) + tau_m_star_star.array();
    
    // 3. Project (clamp) into the torque limit range
    Eigen::VectorXd tau_m(params_.nv);
    for(int i = 0; i < params_.nv; ++i) {
        tau_m(i) = std::clamp(tau_m_star(i), -params_.F_c(i), params_.F_c(i));
    }
    
    // 4. Compute the feedback q_x (written to state)
    Eigen::VectorXd denominator = params_.B_c.array() / params_.T + params_.K_c.array() + params_.L_c.array() * params_.T;
    state.q_x = state.q_s.array() + (tau_m.array() - tau_m_star_star.array()) / denominator.array();
    
    return tau_m;
}

Eigen::VectorXd AdmittanceController::update(ControlState& state) {
    std::cout << "=================================================================================|" << std::endl;
    std::cout << "frame : [" << frame << "]" << std::endl; 
    std::cout << "q_s = " << state.q_s.transpose() << std::endl;
    
   // ==========================================================
    // Execute Experiment III trajectory generation ONLY when config3.yaml is loaded
    // ==========================================================
    if (params_.config_name == "config3") {
        // Use params_.T to get the timestep, ensuring consistency with the configuration file
        double current_time = frame * params_.T;
        state.updateExperimentIIITrajectory(current_time, params_.p_rA, params_.p_rB);
        // If you need to observe the changes in reference pose, you can uncomment the print statement below:
        // std::cout << "[INFO] config3 Active: Updating Experiment III Trajectory." << std::endl;
    }

    state.q_s = unwrapJointAngles(state.q_s);
    // ==========================================================
    // A. Gravity Compensation
    // ==========================================================
    // Compute gravity torque using the current actual joint positions (q_s)
    Eigen::VectorXd tau_dyn = - pinocchio::computeGeneralizedGravity(model_, data_, state.q_s);

    // Get the raw external contact torque, which still includes friction noise.
    Eigen::VectorXd tau_ext_raw = state.tau_s - tau_dyn;
    
    // ==========================================================
    // [Critical Fix 2]: Add a torque deadband to filter out static friction 
    // and gravity residuals after releasing the robot.
    // ==========================================================
    for (int i = 0; i < params_.nv; ++i) {
        // Based on Kinova Gen3 characteristics: large base joints have higher 
        // friction (~2.5Nm), while distal small joints have lower friction (~0.8Nm).
        // These thresholds can be fine-tuned based on the actual rebound behavior. 
        // Larger values make it rebound easier but feel more "sluggish".
        double deadband = (i < 4) ? 2.5 : 0.8; 
        
        if (std::abs(tau_ext_raw(i)) < deadband) {
            state.tau_ext(i) = 0.0; // Inside deadband: considered completely released.
        } else {
            // Smooth transition to ensure a continuous handling feel 
            // and avoid torque jumps at the deadband edges.
            state.tau_ext(i) = tau_ext_raw(i) - (tau_ext_raw(i) > 0 ? deadband : -deadband);
        }
    }

    std::cout << "tau_gravity = " << (tau_dyn).transpose() << std::endl;
    std::cout << "tau_ext (clean) = " << (state.tau_ext).transpose() << std::endl;
    // ==========================================================
    // B. TBAC Core (Admittance and Saturation Control)
    // ==========================================================
    // 1. Dual proxy computation (passing tau_ext without gravity)
    computeTwoProxies(state);
    
    // 2. Saturated position controller (computes the admittance command torque tau_m)
    Eigen::VectorXd tau_m = computeSatPosCtrl(state);
    
    // ==========================================================
    // C. State Updates
    // ==========================================================
    // 3. Compute the actual proxy velocity
    state.u_x = (state.q_x - state.q_x_prv) / params_.T;
    
    // 4. Error integral update
    Eigen::VectorXd b_xs = state.b_xs_prv + params_.T * (state.q_x - state.q_s);
    
    // 5. Velocity projection (maintains original velocity direction)
    double dot_star = u_x_star.dot(u_x_star);
    if (dot_star > 1e-12) {
        double ratio = u_x_star.dot(state.u_x) / dot_star;
        ratio = std::clamp(ratio, 0.0, 1.0);
        state.u_x = ratio * u_x_star;
    } else {
        state.u_x.setZero();
    }
    
    // 6. Update task space variables via forward kinematics
    pinocchio::forwardKinematics(model_, data_, state.q_x);
    pinocchio::updateFramePlacements(model_, data_);
    state.p_x = data_.oMf[EE_FRAME_ID];

    pinocchio::forwardKinematics(model_, data_, state.q_s);
    pinocchio::updateFramePlacements(model_, data_);
    state.p_s = data_.oMf[EE_FRAME_ID];
    
    Eigen::MatrixXd J_x = MathPre::jacobian(model_, data_, state.q_x, EE_FRAME_ID);
    state.v_x = J_x * state.u_x;
    
    // 7. Push current states to _prv for the next cycle
    state.q_x_prv = state.q_x;
    state.u_x_prv = state.u_x;
    state.b_xs_prv = b_xs;
    state.p_x_prv = state.p_x;
    state.v_x_prv = state.v_x;
    
    // ==========================================================
    // D. Final Command Output
    // ==========================================================
    // Superpose gravity torque onto the admittance controller output 
    // as the final command to the motors.
    frame++;
    state.tau = tau_m - tau_dyn;
    
    std::cout << "--- Debug Info ---" << std::endl;
    std::cout << "Error (qx - qs)[1]: " << state.q_x(3) - state.q_s(3) << std::endl;
    std::cout << "tau_m_star (Raw Pull)[1]: " << tau_m(3) << std::endl;
    std::cout << "tau_dyn (Gravity)[1]: " << tau_dyn(3) << std::endl;
    std::cout << "tau (Final Output)[1]: " << state.tau(3) << std::endl;
    
    std::cout << "=================================================================================|" << std::endl;

    return state.tau;
    
    
}