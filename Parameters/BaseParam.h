#ifndef BASE_PARAM_H
#define BASE_PARAM_H

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <pinocchio/spatial/se3.hpp>
#include <string>
#include <iostream>
#include <vector>
#include <filesystem>

class BaseParam {
public:

    std::string config_name;

    // ================== Core Parameters ==================
    std::string urdf_path; // Path to the robot URDF file
    double T; // Sampling period
    int nv;   // Degrees of freedom

    // ================== Task Space Parameters ==================
    Eigen::MatrixXd M_T, B_T, K_T; 
    Eigen::Vector2d F_T;           

    // ================== Joint Space Parameters ==================
    Eigen::MatrixXd M, B, K;       
    Eigen::VectorXd F;             

    // ================== SatPosCtrl Parameters ==================
    Eigen::VectorXd L_c, B_c, K_c, F_c; 

    // ================== Reference Parameters ==================
    Eigen::VectorXd q_r; // Joint space reference position
    pinocchio::SE3 p_r;  // Task space reference pose
    Eigen::VectorXd f_r; // Task space reference force (6D)

    // Start and target poses for trajectory generation
    pinocchio::SE3 p_rA; 
    pinocchio::SE3 p_rB;

    // Default constructor: provides safe default values
    BaseParam() : T(0.001), nv(7) {
        p_r.setIdentity(); // Initialize to identity transform
        f_r = Eigen::VectorXd::Zero(6); // Default to zero force
        p_rA.setIdentity(); // Initialize
        p_rB.setIdentity(); // Initialize
    }

    // Core interface: load parameters from a YAML file
    bool loadFromYaml(const std::string& filepath) {
        try {
            YAML::Node config = YAML::LoadFile(filepath);
            std::string actual_filepath = filepath;

            if (config["selected_config"]) {
                std::string sub_config = config["selected_config"].as<std::string>();
                std::filesystem::path master_path(filepath);
                
                // Assuming the sub-yaml file and config.yaml are in the same directory
                actual_filepath = (master_path.parent_path() / sub_config).string();
                
                // Extract the filename without extension, e.g., "config1A"
                config_name = std::filesystem::path(sub_config).stem().string(); 
                
                // Reload the corresponding sub-configuration file
                config = YAML::LoadFile(actual_filepath);
                std::cout << "[INFO] Nested config detected. Loading sub-config: " << actual_filepath << "\n";
            } else {
                // If a sub-configuration is loaded directly, extract its name as well
                config_name = std::filesystem::path(filepath).stem().string();
            }

            // Load URDF path
            if (config["urdf_path"]) {
                urdf_path = config["urdf_path"].as<std::string>();
            } else {
                std::cerr << "[Warning] 'urdf_path' not found in YAML.\n";
            }

            T = config["timestep"].as<double>();
            nv = config["nv"].as<int>();

            // Helper lambda: automatically extracts a Vector
            auto loadVectorXd = [&](const YAML::Node& node) -> Eigen::VectorXd {
                Eigen::VectorXd vec(node.size());
                for (std::size_t i = 0; i < node.size(); ++i) {
                    vec(i) = node[i].as<double>();
                }
                return vec;
            };

            // Helper lambda: loads a 1D array as a diagonal matrix
            auto loadDiagonalMatrix = [&](const YAML::Node& node) -> Eigen::MatrixXd {
                Eigen::VectorXd diag = loadVectorXd(node);
                return diag.asDiagonal();
            };

            // NEW Helper lambda: parse SE3 pose from a YAML node
            auto parseSE3 = [&](const YAML::Node& node, pinocchio::SE3& target_pose, const std::string& name) {
                if (node) {
                    std::vector<double> vec = node.as<std::vector<double>>();
                    if (vec.size() == 7) {
                        Eigen::Vector3d translation(vec[0], vec[1], vec[2]);
                        Eigen::Quaterniond quat(vec[3], vec[4], vec[5], vec[6]);
                        target_pose = pinocchio::SE3(quat.normalized().toRotationMatrix(), translation);
                    } else {
                        std::cerr << "Warning: " << name << " size in YAML is not 7.\n";
                    }
                }
            };

            // Load task space parameters
            M_T = loadDiagonalMatrix(config["M_T"]);
            B_T = loadDiagonalMatrix(config["B_T"]);
            K_T = loadDiagonalMatrix(config["K_T"]);
            
            std::vector<double> ft_vec = config["F_T"].as<std::vector<double>>();
            if (ft_vec.size() == 2) {
                F_T << ft_vec[0], ft_vec[1];
            } else {
                std::cerr << "Warning: F_T size in YAML is not 2.\n";
            }

            // Load joint space parameters
            M = loadDiagonalMatrix(config["M"]);
            B = loadDiagonalMatrix(config["B"]);
            K = loadDiagonalMatrix(config["K"]);
            F = loadVectorXd(config["F"]);

            // Load saturated position controller (SatPosCtrl) parameters
            K_c = loadVectorXd(config["K_c"]);
            B_c = loadVectorXd(config["B_c"]);
            L_c = loadVectorXd(config["L_c"]);
            F_c = loadVectorXd(config["F_c"]);

            // Load Reference Parameters
            q_r = loadVectorXd(config["q_r"]);
            f_r = loadVectorXd(config["f_r"]);
            
            // Use the newly written parseSE3 lambda to parse p_r, p_rA, and p_rB
            parseSE3(config["p_r"], p_r, "p_r");
            parseSE3(config["p_rA"], p_rA, "p_rA");
            parseSE3(config["p_rB"], p_rB, "p_rB");

            return true;
        } catch (const YAML::Exception& e) {
            std::cerr << "YAML Loading Error: " << e.what() << "\n";
            return false;
        }
    }
};

#endif // BASE_PARAM_H