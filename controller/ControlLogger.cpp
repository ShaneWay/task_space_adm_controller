#include "ControlLogger.h"
#include "matplotlibcpp.h" // Included only here to avoid polluting the global namespace and slowing down compilation
#include <fstream>
#include <iomanip>
#include <iostream>
#include <filesystem>

namespace plt = matplotlibcpp;
namespace fs = std::filesystem;

void ControlLogger::init(int nv) {
    q_s_.assign(nv, std::vector<double>());
    u_s_.assign(nv, std::vector<double>());
    q_x_.assign(nv, std::vector<double>());
    u_x_.assign(nv, std::vector<double>());
    
    // Initialize torque containers
    tau_s_.assign(nv, std::vector<double>());
    tau_ext_.assign(nv, std::vector<double>());
    tau_.assign(nv, std::vector<double>());

    p_x_.assign(3, std::vector<double>()); 
    p_s_.assign(3, std::vector<double>());
    p_r_.assign(3, std::vector<double>());
    v_x_.assign(6, std::vector<double>());
    v_s_.assign(6, std::vector<double>());

    f_s_.assign(6, std::vector<double>());
}

void ControlLogger::log(double t, const ControlState& state) {
    time_.push_back(t);

    for (int i = 0; i < state.q_s.size(); ++i) {
        q_s_[i].push_back(state.q_s(i));
        u_s_[i].push_back(state.u_s(i));
        q_x_[i].push_back(state.q_x(i));
        u_x_[i].push_back(state.u_x(i));

        // Log torque values
        tau_s_[i].push_back(state.tau_s(i));
        tau_ext_[i].push_back(state.tau_ext(i));
        tau_[i].push_back(state.tau(i));
    }

    // Log the first three dimensions (translation X, Y, Z)
    for (int i = 0; i < 3; ++i) {
        p_x_[i].push_back(state.p_x.translation()(i));
        p_s_[i].push_back(state.p_s.translation()(i));
        p_r_[i].push_back(state.p_r.translation()(i));
    }
    // Log all six dimensions (spatial velocity)
    for (int i = 0; i < 6; ++i) {
        v_x_[i].push_back(state.v_x(i));
        v_s_[i].push_back(state.v_s(i));
        f_s_[i].push_back(state.f_s(i));
    }
}

void ControlLogger::setLogDirectory(const std::string& config_name) {
    // You can customize the root folder name; here we use "results/" as the root directory
    log_dir_ = "results/" + config_name + "/"; 
    if (!fs::exists(log_dir_)) {
        fs::create_directories(log_dir_);
    }
    std::cout << "[INFO] Log directory set to: " << log_dir_ << std::endl;
}

// Core function: Export to a comma-separated text file
void ControlLogger::saveToTxt(const std::string& filename) const {
    std::string full_path = log_dir_ + filename;
    std::ofstream outFile(full_path);
    if (!outFile.is_open()) {
        std::cerr << "[ERROR] Cannot open file to save logs: " << full_path << std::endl;
        return;
    }

    int nv = q_s_.size();
    if (nv == 0 || time_.empty()) return;

    // 1. Write the header
    outFile << "time";
    for (int i = 0; i < nv; ++i) outFile << ",tau_s_" << i;
    for (int i = 0; i < nv; ++i) outFile << ",q_s_" << i;
    for (int i = 0; i < nv; ++i) outFile << ",u_s_" << i;
    for (int i = 0; i < nv; ++i) outFile << ",tau_ext_" << i;
    for (int i = 0; i < nv; ++i) outFile << ",tau_" << i;
    for (int i = 0; i < 3; ++i) outFile << ",p_s_" << i;
    for (int i = 0; i < 3; ++i)  outFile << ",p_x_" << i;  // px: Task space translation
    for (int i = 0; i < 3; ++i)  outFile << ",p_r_" << i;
    for (int i = 0; i < 6; ++i)  outFile << ",v_x_" << i;
    for (int i = 0; i < 6; ++i)  outFile << ",f_s_" << i;  // vx: Task space velocity
    outFile << "\n";

    // 2. Write data for each frame row by row
    outFile << std::fixed << std::setprecision(6); // Keep 6 decimal places uniformly
    for (size_t t = 0; t < time_.size(); ++t) {
        outFile << time_[t];
        for (int i = 0; i < nv; ++i) outFile << "," << tau_s_[i][t];
        for (int i = 0; i < nv; ++i) outFile << "," << q_s_[i][t];
        for (int i = 0; i < nv; ++i) outFile << "," << u_s_[i][t];
        for (int i = 0; i < nv; ++i) outFile << "," << tau_ext_[i][t];
        for (int i = 0; i < nv; ++i) outFile << "," << tau_[i][t];
        for (int i = 0; i < 3; ++i) outFile << "," << p_s_[i][t];
        for (int i = 0; i < 3; ++i)  outFile << "," << p_x_[i][t];
        for (int i = 0; i < 3; ++i)  outFile << "," << p_r_[i][t];
        for (int i = 0; i < 6; ++i)  outFile << "," << v_x_[i][t];
        for (int i = 0; i < 6; ++i)  outFile << "," << f_s_[i][t];
        outFile << "\n";
    }

    outFile.close();
    std::cout << "[SUCCESS] Control data has been successfully saved to: " << full_path << std::endl;
}

void ControlLogger::plot(int target_joint) const {
    if (time_.empty()) {
        return; // Return immediately if there is no data to prevent crashes
    }

    // ----------------------------------------------------
    // Figure 1: Task Space Position (X, Y, Z) over Time
    // ----------------------------------------------------
    plt::figure();
    
    // X Axis
    plt::plot(time_, p_s_[0], {{"label", "Actual p_s (X)"}, {"color", "red"}, {"linestyle", "-"}});
    plt::plot(time_, p_x_[0], {{"label", "Proxy p_x (X)"}, {"color", "red"}, {"linestyle", "--"}});
    plt::plot(time_, p_r_[0], {{"label", "Ref p_r (X)"}, {"color", "red"}, {"linestyle", ":"}});
    
    // Y Axis
    plt::plot(time_, p_s_[1], {{"label", "Actual p_s (Y)"}, {"color", "green"}, {"linestyle", "-"}});
    plt::plot(time_, p_x_[1], {{"label", "Proxy p_x (Y)"}, {"color", "green"}, {"linestyle", "--"}});
    plt::plot(time_, p_r_[1], {{"label", "Ref p_r (Y)"}, {"color", "green"}, {"linestyle", ":"}});
    
    // Z Axis
    plt::plot(time_, p_s_[2], {{"label", "Actual p_s (Z)"}, {"color", "blue"}, {"linestyle", "-"}});
    plt::plot(time_, p_x_[2], {{"label", "Proxy p_x (Z)"}, {"color", "blue"}, {"linestyle", "--"}});
    plt::plot(time_, p_r_[2], {{"label", "Ref p_r (Z)"}, {"color", "blue"}, {"linestyle", ":"}});

    plt::title("Task Space Position: p_s vs p_x vs p_r");
    plt::xlabel("Time [s]");
    plt::ylabel("Position [m]");
    plt::legend();
    plt::grid(true);

    plt::save(log_dir_ +"Task_Space_Position_Comparison.pdf");
    // ----------------------------------------------------
    // Figure 2: All Joints Command Torque (tau)
    // ----------------------------------------------------
    plt::figure();
    std::vector<std::string> colors = {"red", "green", "blue", "orange", "purple", "brown", "pink"};
    
    for (int i = 0; i < tau_.size(); ++i) {
        plt::plot(time_, tau_[i], {{"label", "tau_" + std::to_string(i + 1)}, {"color", colors[i % 7]}});
    }
    
    plt::title("All Joints Command Torque (tau)");
    plt::xlabel("Time [s]");
    plt::ylabel("Torque [Nm]");
    plt::legend(); 
    plt::grid(true);
    plt::save(log_dir_ +"All_Joints_Command_Torque.pdf");
    // Block and show all plotted figures
    // plt::show();

    plt::figure();
    // std::vector<std::string> colors = {"red", "green", "blue", "orange", "purple", "brown", "pink"};
    
    for (int i = 0; i < tau_.size(); ++i) {
        plt::plot(time_, tau_ext_[i], {{"label", "tau_ext_" + std::to_string(i + 1)}, {"color", colors[i % 7]}});
    }
    
    plt::title("All_Joints_External_Torque Torque (tau)");
    plt::xlabel("Time [s]");
    plt::ylabel("Torque [Nm]");
    plt::legend(); 
    plt::grid(true);
    plt::save(log_dir_ +"All_Joints_External_Torque.pdf");
    // Block and show all plotted figures
    // plt::show();

    plt::figure();
    // std::vector<std::string> colors = {"red", "green", "blue", "orange", "purple", "brown", "pink"};
    
    for (int i = 0; i < tau_.size(); ++i) {
        plt::plot(time_, tau_s_[i], {{"label", "tau_s_" + std::to_string(i + 1)}, {"color", colors[i % 7]}});
    }
    
    plt::title("All_Joints_Measrued_Torque.pdf");
    plt::xlabel("Time [s]");
    plt::ylabel("Torque [Nm]");
    plt::legend(); 
    plt::grid(true);
    plt::save(log_dir_ +"All_Joints_Measrued_Torque.pdf");
    // Block and show all plotted figures
    // plt::show();
    // ----------------------------------------------------
    // Figure 5: Task Space Measured Force (f_s)
    // ----------------------------------------------------
    plt::figure();
    std::vector<std::string> colors_fs = {"red", "green", "blue", "orange", "purple", "brown"};
    std::vector<std::string> labels_fs = {"F_x", "F_y", "F_z", "M_x", "M_y", "M_z"};
    
    for (int i = 0; i < 6; ++i) {
        plt::plot(time_, f_s_[i], {{"label", labels_fs[i]}, {"color", colors_fs[i]}});
    }
    
    plt::title("Task Space Measured Force (f_s)");
    plt::xlabel("Time [s]");
    plt::ylabel("Force [N] / Torque [Nm]");
    plt::legend(); 
    plt::grid(true);
    plt::save(log_dir_ + "Task_Space_Measured_Force.pdf");
}