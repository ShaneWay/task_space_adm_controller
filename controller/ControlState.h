#ifndef CONTROL_STATE_H
#define CONTROL_STATE_H

#include <Eigen/Dense>
#include <pinocchio/spatial/se3.hpp>

class ControlState {
public:
    // ================== Measured States ==================
    Eigen::VectorXd q_s; 
    Eigen::VectorXd u_s; 
    pinocchio::SE3 p_s;  
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

    void initSizes(int nv) {
        q_s.setZero(nv);
        u_s.setZero(nv);
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
    }
};

#endif // CONTROL_STATE_H