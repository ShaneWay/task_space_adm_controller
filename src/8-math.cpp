#include <iostream>
#include <Eigen/Dense>

int main() {
    // 注意：Eigen 的构造函数参数顺序是 (w, x, y, z)
    Eigen::Quaterniond quat(0.0, 0.0, 1.0, 0.0);
    
    // 提取欧拉角 eulerAngles(2, 1, 0) 代表 Z-Y-X 顺序 (Yaw, Pitch, Roll)
    // 返回的结果单位是 弧度 (Radian)
    Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(2,1,0);

    // Eigen::Vector3d euler()

    
    // 将弧度转换为度数以便于阅读
    Eigen::Vector3d euler_deg = euler * (180.0 / M_PI);
    
    std::cout << "Yaw   (Z): " << euler_deg[0] << "°\n";
    std::cout << "Pitch (Y): " << euler_deg[1] << "°\n";
    std::cout << "Roll  (X): " << euler_deg[2] << "°\n";

    return 0;
}