#include <iostream>
#include <string>
#include "BaseParam.h"

int main() {
    std::cout << "==================================================\n";
    std::cout << "      YAML Parameter Loader Test Program\n";
    std::cout << "==================================================\n";
    
    BaseParam params;
    
    // Ensure that config.yaml is in the same directory as the executable,
    // or provide the absolute path here, e.g.: "/home/yxw/task_space_adm_controller/config.yaml"
    std::string yaml_path = "../Parameters/config.yaml"; 
    
    if (params.loadFromYaml(yaml_path)) {
        std::cout << "\n[SUCCESS] Successfully loaded parameters from: " << yaml_path << "\n\n";
        
        std::cout << "---------------- [Core Parameters] ---------------\n";
        std::cout << "Timestep (T) : " << params.T << " s\n";
        std::cout << "DoF (nv)     : " << params.nv << "\n\n";
        
        std::cout << "------------- [Task Space Parameters] ------------\n";
        // Print the diagonal elements of the matrices and transpose them to row vectors for readability
        std::cout << "M_T (diag)   : [" << params.M_T.diagonal().transpose() << " ]\n";
        std::cout << "B_T (diag)   : [" << params.B_T.diagonal().transpose() << " ]\n";
        std::cout << "K_T (diag)   : [" << params.K_T.diagonal().transpose() << " ]\n";
        std::cout << "F_T limits   : [" << params.F_T.transpose() << " ]\n\n";
        
        std::cout << "------------ [Joint Space Parameters] ------------\n";
        std::cout << "M (diag)     : [" << params.M.diagonal().transpose() << " ]\n";
        std::cout << "B (diag)     : [" << params.B.diagonal().transpose() << " ]\n";
        std::cout << "K (diag)     : [" << params.K.diagonal().transpose() << " ]\n";
        std::cout << "F limits     : [" << params.F.transpose() << " ]\n\n";
        
        std::cout << "------------- [SatPosCtrl Parameters] ------------\n";
        std::cout << "K_c          : [" << params.K_c.transpose() << " ]\n";
        std::cout << "B_c          : [" << params.B_c.transpose() << " ]\n";
        std::cout << "L_c          : [" << params.L_c.transpose() << " ]\n";
        std::cout << "F_c limits   : [" << params.F_c.transpose() << " ]\n\n";

        std::cout << "------------- [Reference Parameters] -------------\n";
        std::cout << "q_r          : [" << params.q_r.transpose() << " ]\n";
        std::cout << "f_r          : [" << params.f_r.transpose() << " ]\n\n";
        
        std::cout << "p_r (Translation) :\n" << params.p_r.translation().transpose() << "\n\n";
        std::cout << "p_r (Rotation Matrix) :\n" << params.p_r.rotation() << "\n";
        std::cout << "==================================================\n";
        
    } else {
        std::cerr << "\n[ERROR] Failed to load parameters from: " << yaml_path << "\n";
        std::cerr << "Please check if the file exists and the format is correct.\n";
        return -1;
    }

    return 0;
}