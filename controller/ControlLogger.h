#ifndef CONTROL_LOGGER_H
#define CONTROL_LOGGER_H

#include <vector>
#include <Eigen/Dense>
#include "ControlState.h"

class ControlLogger {
private:
    std::string log_dir_ = "./";
    std::vector<double> time_; // Time axis

    // Joint space variables
    std::vector<std::vector<double>> q_s_;
    std::vector<std::vector<double>> u_s_;
    std::vector<std::vector<double>> q_x_;
    std::vector<std::vector<double>> u_x_;

    std::vector<std::vector<double>> tau_s_;   // Measured torque from sensors
    std::vector<std::vector<double>> tau_ext_; // Computed external interaction torque
    std::vector<std::vector<double>> tau_;     // Final commanded torque sent to motors

    // Task space variables
    std::vector<std::vector<double>> p_x_; 
    std::vector<std::vector<double>> v_x_; 
    std::vector<std::vector<double>> p_s_; 
    std::vector<std::vector<double>> v_s_;
    std::vector<std::vector<double>> p_r_; 
    std::vector<std::vector<double>> f_s_; // Log for Task-space measured force
    std::vector<std::vector<double>> f_r_;

public:
    // Initialization and data logging interfaces
    void init(int nv);
    void log(double t, const ControlState& state);

    // Plotting interface, target_joint specifies the joint to observe (default is 0, i.e., the first joint)
    void plot(int target_joint = 0) const;

    // Interface to save all key data to a txt/csv file
    void saveToTxt(const std::string& filename) const;

    void setLogDirectory(const std::string& config_name);

    // Getters
    const std::vector<double>& getTime() const { return time_; }
    const std::vector<double>& get_q_s(int joint) const { return q_s_[joint]; }
    const std::vector<double>& get_q_x(int joint) const { return q_x_[joint]; }
    const std::vector<double>& get_tau_m(int joint) const { return tau_[joint]; }
    const std::vector<double>& get_p_s(int axis) const { return p_s_[axis]; }
    const std::vector<double>& get_p_x(int axis) const { return p_x_[axis]; }
};

#endif // CONTROL_LOGGER_H