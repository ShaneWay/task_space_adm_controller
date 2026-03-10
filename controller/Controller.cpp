#include "Controller.h"
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <Eigen/SVD> 

AdmittanceController::AdmittanceController(const pinocchio::Model& model, const BaseParam& params)
    : model_(model), data_(model), params_(params) {
    
    // Initialize intermediate variables within the class
    q_x_star = Eigen::VectorXd::Zero(params_.nv);
    u_x_star = Eigen::VectorXd::Zero(params_.nv);
}

void AdmittanceController::init(ControlState& state) {
    // Initialize the historical proxy states using the currently measured joint states
    state.q_x_prv = state.q_s;
    state.u_x_prv = state.u_s;
    state.b_xs_prv.setZero();
    
    // Compute the initial end-effector pose
    pinocchio::forwardKinematics(model_, data_, state.q_s);
    pinocchio::updateFramePlacements(model_, data_);
    state.p_x_prv = data_.oMf[EE_FRAME_ID];
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

void AdmittanceController::computeTwoProxies(ControlState& state, const Eigen::VectorXd& tau_s) {
    // 1. Get Jacobians
    Eigen::MatrixXd J_s = MathPre::jacobian(model_, data_, state.q_s, EE_JOINT_ID);
    Eigen::MatrixXd J_x_prv = MathPre::jacobian(model_, data_, state.q_x_prv, EE_JOINT_ID);
    
    // 2. Get Jacobian time variation
    Eigen::MatrixXd H_x_prv = MathPre::jacobiansTimeVariation(model_, data_, state.q_x_prv, state.u_x_prv, EE_JOINT_ID);
    
    // 3. Compute J_hat_x
    Eigen::MatrixXd J_hat_x = J_x_prv + params_.T * H_x_prv;
    
    // 4. Compute M^{-h}
    Eigen::MatrixXd M_h = params_.M.llt().matrixL();
    Eigen::MatrixXd M_inv_h = M_h.inverse();
    
    // 5. Compute C_T and C_J
    Eigen::MatrixXd C_T = M_inv_h * J_s.transpose() * (params_.M_T + params_.T * params_.B_T) * J_hat_x;
    Eigen::MatrixXd C_J = M_inv_h * (params_.M + params_.T * params_.B);
    
    // 6. Compute f_re and tau_re
    Eigen::VectorXd err_p = MathPre::ominus(state.p_r, state.p_x_prv); 
    Eigen::VectorXd f_re = params_.M_T * state.a_r + params_.B_T * state.v_r + MathPre::sat3(params_.F_T, params_.K_T * err_p);
    
    Eigen::VectorXd err_q = state.q_r - state.q_x_prv;
    Eigen::VectorXd tau_re = params_.M * state.aq_r + params_.B * state.u_r + MathPre::sat1(params_.F, params_.K * err_q);
    
    // 7. Compute b_T and b_J
    Eigen::VectorXd b_T = M_inv_h * (J_s.transpose() * (-params_.B_T * state.v_x_prv - (params_.M_T + params_.T * params_.B_T) * H_x_prv * state.u_x_prv + state.f_r + f_re) + tau_s);
    Eigen::VectorXd b_J = M_inv_h * (-params_.B * state.u_x_prv + tau_re + tau_s);
    
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
    Eigen::VectorXd tau_m_star = ((params_.K_c.array() + params_.L_c.array() * params_.T) * (q_x_star - state.q_s).array()) + tau_m_star_star.array();
    
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

Eigen::VectorXd AdmittanceController::update(ControlState& state, const Eigen::VectorXd& tau_s) {
    // 1. Dual proxy computation (TwoProxies)
    computeTwoProxies(state, tau_s);
    
    // 2. Saturated position controller (directly updates state.q_x)
    Eigen::VectorXd tau_m = computeSatPosCtrl(state);
    
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
    
    Eigen::MatrixXd J_x = MathPre::jacobian(model_, data_, state.q_x, EE_JOINT_ID);
    state.v_x = J_x * state.u_x;
    
    // 7. Push current states to _prv for the next cycle
    state.q_x_prv = state.q_x;
    state.u_x_prv = state.u_x;
    state.b_xs_prv = b_xs;
    state.p_x_prv = state.p_x;
    state.v_x_prv = state.v_x;
    
    // 8. Return the command torque for the motors
    return tau_m;
}