#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include "BaseParam.h"

int main()
{
    // 1. Load parameters and Pinocchio model
    BaseParam params;
    std::string yaml_path = "../Parameters/config.yaml"; // Please ensure the path is correct
    if (!params.loadFromYaml(yaml_path)) {
        std::cerr << "[ERROR] Could not load YAML configuration." << std::endl;
        return false;
    }

    pinocchio::Model model;
    try {
        pinocchio::urdf::buildModel(params.urdf_path, model);
        std::cout << "[SUCCESS] Pinocchio Model Loaded from: " << params.urdf_path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load URDF: " << e.what() << std::endl;
        return false;
    }
    pinocchio::Data data(model);
    Eigen::VectorXd q_rad = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd tau_gravity = Eigen::VectorXd::Zero(model.nv);
    float exp_start_angle[] = {0.073, 28.099, 179.942, 281.407, 359.992, 286.364, 180.197};
    for(int i = 0; i < 7; i++)
    {
        q_rad(i) = exp_start_angle[i] / 180. * M_PI;
    }

    tau_gravity = pinocchio::computeGeneralizedGravity(model, data, q_rad);
    for(int i = 0; i < tau_gravity.size(); i++)
    {
        std::cout << tau_gravity[i] << std::endl;
    }
}