#ifndef BASE_PARAM_H
#define BASE_PARAM_H

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <pinocchio/spatial/se3.hpp>
#include <string>
#include <iostream>
#include <vector>

class BaseParam {
public:
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

    // Default constructor: provides safe default values
    BaseParam() : T(0.001), nv(7) {
        p_r.setIdentity(); // Initialize to identity transform
        f_r = Eigen::VectorXd::Zero(6); // Default to zero force
    }

    // Core interface: load parameters from a YAML file
    bool loadFromYaml(const std::string& filepath) {
        try {
            YAML::Node config = YAML::LoadFile(filepath);

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
            
            // Parse Task Space Reference Pose (p_r)
            std::vector<double> pr_vec = config["p_r"].as<std::vector<double>>();
            if (pr_vec.size() == 7) {
                Eigen::Vector3d translation(pr_vec[0], pr_vec[1], pr_vec[2]);
                // Eigen::Quaterniond constructor takes (w, x, y, z)
                Eigen::Quaterniond quat(pr_vec[3], pr_vec[4], pr_vec[5], pr_vec[6]);
                
                // Construct SE3 from translation and normalized quaternion
                p_r = pinocchio::SE3(quat.normalized().toRotationMatrix(), translation);
            } else {
                std::cerr << "Warning: p_r size in YAML is not 7.\n";
            }

            return true;
        } catch (const YAML::Exception& e) {
            std::cerr << "YAML Loading Error: " << e.what() << "\n";
            return false;
        }
    }
};

#endif // BASE_PARAM_H