#ifndef CONTROL_STATE_H
#define CONTROL_STATE_H
#include "MathPre.hpp"

#include <Eigen/Dense>
#include <pinocchio/spatial/se3.hpp>

class ControlState {
public:
    // ================== Measured States ==================
    Eigen::VectorXd q_s; 
    Eigen::VectorXd u_s; 
    pinocchio::SE3 p_s;  
    Eigen::VectorXd tau_s;
    Eigen::VectorXd tau;

    Eigen::VectorXd tau_ext;
    Eigen::Matrix<double, 6, 1> v_s;

    // ================== Task-Space Proxy States ==================
    pinocchio::SE3 p_x;      
    pinocchio::SE3 p_x_prv;  
    Eigen::Matrix<double, 6, 1> v_x;     
    Eigen::Matrix<double, 6, 1> v_x_prv; 

    // ================== Joint-Space Proxy States ==================
    Eigen::VectorXd q_x;     
    Eigen::VectorXd q_x_prv; 
    Eigen::VectorXd u_x;     
    Eigen::VectorXd u_x_prv; 

    // ================== Controller Internal States ==================
    Eigen::VectorXd b_xs_prv; 

    // ================== Reference Inputs ==================
    pinocchio::SE3 p_r;       
    Eigen::Matrix<double, 6, 1> v_r; 
    Eigen::Matrix<double, 6, 1> a_r; 
    Eigen::VectorXd q_r;      
    Eigen::VectorXd u_r;      
    Eigen::VectorXd aq_r;     
    Eigen::Matrix<double, 6, 1> f_r; 
    Eigen::Matrix<double, 6, 1> f_r1; 
    Eigen::Matrix<double, 6, 1> f_r2; 
    
    Eigen::Matrix<double, 6, 1> f_s; // Task-space measured force/torque (6D)

    void initSizes(int nv) {
        q_s.setZero(nv);
        u_s.setZero(nv);
        tau_s.setZero(nv);
        tau.setZero(nv);
        tau_ext.setZero(nv);
        q_x.setZero(nv);
        q_x_prv.setZero(nv);
        u_x.setZero(nv);
        u_x_prv.setZero(nv);
        b_xs_prv.setZero(nv);
        q_r.setZero(nv);
        u_r.setZero(nv);
        aq_r.setZero(nv);
        
        v_s.setZero();
        v_x.setZero();
        v_x_prv.setZero();
        v_r.setZero();
        a_r.setZero();
        f_r.setZero();
        f_r1.setZero();
        f_r2.setZero();
        f_s.setZero();
        
    }

    // ================== Trajectory Generation ==================
    
    /**
     * @brief Experiment III: Generate a smooth sinusoidal acceleration/deceleration end-effector trajectory
     * @param t    Current running time (s)
     * @param p_rA Starting reference pose (as p_rA in the experiment)
     * @param p_rB Target reference pose (as p_rB in the experiment)
     * @param T_S  Trajectory period duration (set to 4.0s in the paper)
     */
    void updateExperimentIIITrajectory(double t, const pinocchio::SE3& p_rA, const pinocchio::SE3& p_rB) 
    {
        double T_S = 10.0;
        const double omega = 2.0 * M_PI / T_S;

        // 1. Calculate the time scaling factor s(t) and its first and second derivatives
        // This ensures the motion smoothly reciprocates between p_rA and p_rB
        double s = 0.5 * (1.0 - std::cos(omega * t));
        double s_dot = 0.5 * omega * std::sin(omega * t);
        double s_ddot = 0.5 * omega * omega * std::cos(omega * t);

        // If you want a one-way point-to-point motion (stopping after reaching p_rB), uncomment the following:
        /*
        if (t >= T_S / 2.0) {
            s = 1.0;
            s_dot = 0.0;
            s_ddot = 0.0;
        }
        */

        // 2. Calculate the difference of B relative to A in the SE(3) space
        Eigen::Matrix<double, 6, 1> diff = MathPre::ominus(p_rB, p_rA);

        // 3. Update the current reference pose p_r
        this->p_r = MathPre::oplus(p_rA, s * diff);

        // 4. Synchronously update the feedforward velocity v_r and acceleration a_r
        this->v_r = s_dot * diff;
        this->a_r = s_ddot * diff;
    }

    /**
     * @brief Experiment IV: Generate step-like reference forces
     * Switches the reference force f_r periodically between 0, f_r1, and f_r2.
     * @param t             Current running time (s)
     * @param step_duration Duration for each force stage in seconds
     */
    void updateExperimentIVForce(double t, double step_duration = 4.0) {
        // Divide the time into stages based on the step_duration
        // E.g., Stage 0: 0 force, Stage 1: f_r1, Stage 2: f_r2
        int stage = static_cast<int>(t / step_duration) % 3;
        
        if (stage == 0) {
            this->f_r = this->f_r;
        } else if (stage == 1) {
            this->f_r = this->f_r1;
        } else {
            this->f_r = this->f_r2;
        }
    }
};

#endif // CONTROL_STATE_H