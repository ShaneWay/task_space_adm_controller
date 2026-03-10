#ifndef BASE_PARAM_H
#define BASE_PARAM_H

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>

struct BaseParam {
    // ================== Core Parameters ==================
    double T; // Sampling period [cite: 271]
    int nv;   // Degrees of freedom

    // ================== Task Space Parameters ==================
    Eigen::MatrixXd M_T, B_T, K_T; // Inertia, viscosity, stiffness [cite: 417, 418, 419]
    Eigen::Vector2d F_T;           // [Translation limit, Rotation limit] [cite: 423]

    // ================== Joint Space Parameters ==================
    Eigen::MatrixXd M, B, K;       // Inertia, viscosity, stiffness [cite: 426]
    Eigen::VectorXd F;             // Joint torque proxy limits [cite: 432]

    // ================== SatPosCtrl Parameters ==================
    Eigen::VectorXd L_c, B_c, K_c, F_c; // Saturation Position Controller gains & limits [cite: 244, 270]

    // Load parameters from a YAML file
    bool loadFromYaml(const std::string& filepath) {
        try {
            YAML::Node config = YAML::LoadFile(filepath);

            T = config["timestep"].as<double>();
            nv = config["nv"].as<int>();

            // Helper lambda to load VectorXd
            auto loadVectorXd = [&](const YAML::Node& node) -> Eigen::VectorXd {
                Eigen::VectorXd vec(node.size());
                for (std::size_t i = 0; i < node.size(); ++i) vec(i) = node[i].as<double>();
                return vec;
            };

            // Helper lambda to load MatrixXd (assuming diagonal matrices stored as lists in YAML for simplicity)
            auto loadDiagonalMatrix = [&](const YAML::Node& node) -> Eigen::MatrixXd {
                Eigen::VectorXd diag = loadVectorXd(node);
                return diag.asDiagonal();
            };

            // Load Task Space
            M_T = loadDiagonalMatrix(config["M_T"]);
            B_T = loadDiagonalMatrix(config["B_T"]);
            K_T = loadDiagonalMatrix(config["K_T"]);
            
            std::vector<double> ft_vec = config["F_T"].as<std::vector<double>>();
            if (ft_vec.size() == 2) F_T << ft_vec[0], ft_vec[1];

            // Load Joint Space
            M = loadDiagonalMatrix(config["M"]);
            B = loadDiagonalMatrix(config["B"]);
            K = loadDiagonalMatrix(config["K"]);
            F = loadVectorXd(config["F"]);

            // Load SatPosCtrl
            K_c = loadVectorXd(config["K_c"]);
            B_c = loadVectorXd(config["B_c"]);
            L_c = loadVectorXd(config["L_c"]);
            F_c = loadVectorXd(config["F_c"]);

            return true;
        } catch (const YAML::Exception& e) {
            std::cerr << "YAML Loading Error: " << e.what() << "\n";
            return false;
        }
    }
};

#endif // BASE_PARAM_H