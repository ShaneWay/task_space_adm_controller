#ifndef MATH_PRE_HPP
#define MATH_PRE_HPP

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace MathPre {

    // ==========================================
    // 1. Saturation Functions
    // ==========================================

    /**
     * @brief Equation (2): sat1 function
     * Element-wise saturation of an n-dimensional vector.
     * @param F n-dimensional limit vector (usually positive).
     * @param x n-dimensional input vector.
     * @return Saturated n-dimensional vector.
     */
    inline Eigen::VectorXd sat1(const Eigen::VectorXd& F, const Eigen::VectorXd& x) {
        if (F.size() != x.size()) {
            throw std::invalid_argument("MathPre::sat1 - Vectors F and x must have the same size.");
        }

        Eigen::VectorXd result = Eigen::VectorXd::Zero(x.size());
        for (Eigen::Index i = 0; i < x.size(); ++i) {
            double den = std::max(F(i), std::abs(x(i)));
            if (den > 1e-9) { // Avoid division by zero
                result(i) = (F(i) * x(i)) / den;
            }
        }
        return result;
    }

    /**
     * @brief Equation (3): sat3 function (for 6D force/torque)
     * Saturates the first 3 dimensions (translation) and the last 3 dimensions (rotation) 
     * based on their respective L2 norms.
     * @param F 2-dimensional limit vector [translation limit, rotation limit].
     * @param x 6-dimensional input vector (e.g., Wrench).
     * @return Saturated 6-dimensional vector.
     */
    inline Eigen::Matrix<double, 6, 1> sat3(const Eigen::Vector2d& F, const Eigen::Matrix<double, 6, 1>& x) {
        Eigen::Matrix<double, 6, 1> result = Eigen::Matrix<double, 6, 1>::Zero();

        // 1. Process the first 3 dimensions (translational part)
        double norm_trans = x.head<3>().norm();
        double den_trans = std::max(F(0), norm_trans);
        if (den_trans > 1e-9) {
            result.head<3>() = (F(0) * x.head<3>()) / den_trans;
        }

        // 2. Process the last 3 dimensions (rotational part)
        double norm_rot = x.tail<3>().norm();
        double den_rot = std::max(F(1), norm_rot);
        if (den_rot > 1e-9) {
            result.tail<3>() = (F(1) * x.tail<3>()) / den_rot;
        }

        return result;
    }

    // ==========================================
    // 2. Kinematics Functions
    // ==========================================

    /**
     * @brief Computes the Jacobian matrix J for a specified joint.
     * @param model Pinocchio robot model.
     * @param data  Pinocchio data cache.
     * @param q     Current joint position vector.
     * @param joint_id ID of the joint to compute the Jacobian for.
     * @return 6 x nv Jacobian matrix (in the LOCAL_WORLD_ALIGNED frame).
     */
    inline Eigen::MatrixXd jacobian(const pinocchio::Model& model, 
                                    pinocchio::Data& data, 
                                    const Eigen::VectorXd& q, 
                                    pinocchio::FrameIndex frame_id) {
        // Update kinematics and compute Jacobians for all joints
        pinocchio::computeJointJacobians(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        Eigen::MatrixXd J(6, model.nv);
        J.setZero();
        
        // Extract the Jacobian of the specific joint; LOCAL_WORLD_ALIGNED is recommended 
        // for its intuitive physical meaning in operational space control.
        pinocchio::computeFrameJacobian(model, data, q, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
        return J;
    }

    /**
     * @brief Computes the time variation of the Jacobian matrix dJ for a specified joint.
     * Used to compute the Coriolis/centrifugal acceleration term (dJ * dq).
     * @param model Pinocchio robot model.
     * @param data  Pinocchio data cache.
     * @param q     Current joint position vector.
     * @param v     Current joint velocity vector (dq).
     * @param joint_id ID of the joint to compute dJ for.
     * @return 6 x nv Jacobian time variation matrix.
     */
    inline Eigen::MatrixXd jacobiansTimeVariation(const pinocchio::Model& model, 
                                                  pinocchio::Data& data, 
                                                  const Eigen::VectorXd& q, 
                                                  const Eigen::VectorXd& v, 
                                                  pinocchio::FrameIndex frame_id) {
        // Update kinematics including velocity and compute dJ
        pinocchio::computeJointJacobians(model, data, q);
        pinocchio::updateFramePlacements(model, data);

        pinocchio::computeJointJacobiansTimeVariation(model, data, q, v);
        Eigen::MatrixXd dJ(6, model.nv);
        dJ.setZero();
        
        // Extract dJ for the specific joint
        pinocchio::getFrameJacobianTimeVariation(model, data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, dJ);
        return dJ;
    }

    // ==========================================
    // 3. Quaternion Calculus (Appendix A)
    // ==========================================

    /**
     * @brief Converts a rotation vector to a unit quaternion (Eq. 102).
     * @param r 3D rotation vector.
     * @return Unit quaternion corresponding to the rotation vector.
     */
    inline Eigen::Quaterniond v2q(const Eigen::Vector3d& r) {
        double theta = r.norm();
        if (theta < 1e-6) {
            // Taylor expansion near zero to avoid division by zero
            return Eigen::Quaterniond(1.0, r.x() / 2.0, r.y() / 2.0, r.z() / 2.0).normalized();
        } else {
            // v2q(r) = [cos(||r||/2), r * sinc(||r||/2) / 2]^T
            Eigen::Vector3d n = r / theta;
            return Eigen::Quaterniond(Eigen::AngleAxisd(theta, n));
        }
    }

    /**
     * @brief Converts a unit quaternion to a rotation vector (Eq. 101).
     * @param a Unit quaternion.
     * @return 3D rotation vector.
     */
    inline Eigen::Vector3d q2v(const Eigen::Quaterniond& a) {
        Eigen::Quaterniond q = a;
        if (q.w() < 0) {
            // Equivalent to sgn(a_w) handling in Eq. 101 to take the shortest path
            q.coeffs() = -q.coeffs();
        }
        
        double norm_v = q.vec().norm();
        if (norm_v < 1e-6) {
            // Taylor expansion near zero for 2 * a_xyz / sinc(asin(||a_xyz||))
            return 2.0 * q.vec();
        } else {
            double theta_half = std::asin(norm_v); // equivalent to atan2(norm_v, q.w())
            return 2.0 * theta_half * (q.vec() / norm_v);
        }
    }

    /**
     * @brief Subtraction operator for unit quaternions (Eq. 98).
     * a \ominus b = q2v(a \otimes inv(b))
     * @param a Target unit quaternion.
     * @param b Base unit quaternion.
     * @return 3D rotational difference vector.
     */
    inline Eigen::Vector3d ominus(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
        // Eigen's quaternion multiplication is exactly \otimes, and conjugate is inv()
        return q2v(a * b.conjugate());
    }

    /**
     * @brief Addition operator for unit quaternion and rotation vector (Eq. 99).
     * a \oplus r = v2q(r) \otimes a
     * @param a Base unit quaternion.
     * @param r 3D rotation vector.
     * @return Updated unit quaternion.
     */
    inline Eigen::Quaterniond oplus(const Eigen::Quaterniond& a, const Eigen::Vector3d& r) {
        return v2q(r) * a;
    }

    // ==========================================
    // 4. Decoupled Pose Space P (SE(3) Alternative)
    // ==========================================

    /**
     * @brief Subtraction operator for poses in P space (Eq. 106).
     * Decouples the translation and rotation differences.
     * @param p1 Target pose (represented by pinocchio::SE3).
     * @param p2 Base pose (represented by pinocchio::SE3).
     * @return 6D difference vector [translation_diff, rotation_diff].
     */
    inline Eigen::Matrix<double, 6, 1> ominus(const pinocchio::SE3& p1, const pinocchio::SE3& p2) {
        Eigen::Matrix<double, 6, 1> res;
        // Translational difference (Eq. 106 upper part)
        res.head<3>() = p1.translation() - p2.translation();
        // Rotational difference (Eq. 106 lower part)
        Eigen::Quaterniond q1(p1.rotation());
        Eigen::Quaterniond q2(p2.rotation());
        res.tail<3>() = ominus(q1, q2);
        return res;
    }

    /**
     * @brief Addition operator for pose and 6D velocity in P space (Eq. 107).
     * Decouples the translation and rotation updates.
     * @param p Base pose (represented by pinocchio::SE3).
     * @param v 6D spatial velocity vector [linear_vel, angular_vel].
     * @return Updated pose (represented by pinocchio::SE3).
     */
    inline pinocchio::SE3 oplus(const pinocchio::SE3& p, const Eigen::Matrix<double, 6, 1>& v) {
        pinocchio::SE3 res;
        // Translational addition (Eq. 107 upper part)
        res.translation() = p.translation() + v.head<3>();
        // Rotational addition (Eq. 107 lower part)
        Eigen::Quaterniond q(p.rotation());
        res.rotation() = oplus(q, v.tail<3>()).toRotationMatrix();
        return res;
    }

} // namespace MathPre

#endif // MATH_PRE_HPP