#include "ControlLogger.h"
#include "matplotlibcpp.h" // Included only here to avoid polluting the global namespace and slowing down compilation

namespace plt = matplotlibcpp;

void ControlLogger::init(int nv) {
    q_s_.assign(nv, std::vector<double>());
    u_s_.assign(nv, std::vector<double>());
    q_x_.assign(nv, std::vector<double>());
    u_x_.assign(nv, std::vector<double>());
    tau_m_.assign(nv, std::vector<double>());

    p_x_.assign(3, std::vector<double>()); 
    p_s_.assign(3, std::vector<double>());
    v_x_.assign(6, std::vector<double>());
    v_s_.assign(6, std::vector<double>());
}

void ControlLogger::log(double t, const ControlState& state, const Eigen::VectorXd& tau) {
    time_.push_back(t);

    for (int i = 0; i < state.q_s.size(); ++i) {
        q_s_[i].push_back(state.q_s(i));
        u_s_[i].push_back(state.u_s(i));
        q_x_[i].push_back(state.q_x(i));
        u_x_[i].push_back(state.u_x(i));
        tau_m_[i].push_back(tau(i));
    }

    // Log the first three dimensions (translation X, Y, Z)
    for (int i = 0; i < 3; ++i) {
        p_x_[i].push_back(state.p_x.translation()(i));
        p_s_[i].push_back(state.p_s.translation()(i));
    }
    // Log all six dimensions (spatial velocity)
    for (int i = 0; i < 6; ++i) {
        v_x_[i].push_back(state.v_x(i));
        v_s_[i].push_back(state.v_s(i));
    }
}

void ControlLogger::plot(int target_joint) const {
    if (time_.empty()) {
        return; // Return immediately if there is no data to prevent crashes
    }

    // ----------------------------------------------------
    // Figure 1: Joint space position comparison
    // ----------------------------------------------------
    plt::figure();
    plt::plot(time_, q_s_[target_joint], {{"label", "Measured q_s"}, {"color", "red"}, {"linestyle", "-"}});
    plt::plot(time_, q_x_[target_joint], {{"label", "Proxy q_x"}, {"color", "blue"}, {"linestyle", "--"}});
    
    plt::title("Joint " + std::to_string(target_joint + 1) + " Position: Measured vs Proxy");
    plt::xlabel("Time [s]");
    plt::ylabel("Angle [rad]");
    plt::legend();
    plt::grid(true);

    // ----------------------------------------------------
    // Figure 2: Task space trajectory (X-Y Plane)
    // ----------------------------------------------------
    plt::figure();
    plt::plot(p_s_[0], p_s_[1], {{"label", "Measured p_s (XY)"}, {"color", "green"}});
    plt::plot(p_x_[0], p_x_[1], {{"label", "Proxy p_x (XY)"}, {"color", "orange"}, {"linestyle", "--"}});
    
    plt::title("Task Space Trajectory (X-Y Plane)");
    plt::xlabel("X Position [m]");
    plt::ylabel("Y Position [m]");
    plt::legend();
    plt::grid(true);

    // ----------------------------------------------------
    // Figure 3: Joint commanded torque
    // ----------------------------------------------------
    plt::figure();
    plt::plot(time_, tau_m_[target_joint], {{"label", "Command Torque"}, {"color", "black"}});
    
    plt::title("Joint " + std::to_string(target_joint + 1) + " Command Torque");
    plt::xlabel("Time [s]");
    plt::ylabel("Torque [Nm]");
    plt::legend();
    plt::grid(true);

    // Block and show all plotted figures
    plt::show();
}