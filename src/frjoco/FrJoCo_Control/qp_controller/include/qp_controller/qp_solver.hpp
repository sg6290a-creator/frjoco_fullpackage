#pragma once

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <rbdl/rbdl.h>

/**
 * QPSolver — OSQP 기반 IK velocity QP
 *
 * 매 사이클 dq* = argmin  ||J·dq - v_des||² + α||dq||²
 *                  s.t.   dq_min ≤ dq ≤ dq_max          (velocity limit)
 *                         q_min  ≤ q + dq·dt ≤ q_max    (joint limit)
 */
class QPSolver {
public:
    static constexpr int DOF = 6;

    struct Limits {
        Eigen::VectorXd q_min;    // rad
        Eigen::VectorXd q_max;    // rad
        double dq_max  = 1.0;     // rad/s
        double alpha   = 1e-4;    // regularisation weight
        double lambda  = 1e-3;    // DLS fallback damping (unused in QP, kept for compat)
    };

    bool init(const Limits& limits);

    /**
     * Solve for dq given current joint config, desired Cartesian twist, and dt.
     * @return dq (rad/s), zero vector on failure
     */
    Eigen::VectorXd solve(const Eigen::VectorXd& q,
                          const Eigen::MatrixXd& J,   // 6×DOF reordered [v;ω]
                          const Eigen::VectorXd& v_des,
                          double dt);

    // FK
    bool initModel();
    Eigen::Isometry3d eePose(const Eigen::VectorXd& q);
    void calcJacobian(const Eigen::VectorXd& q, Eigen::MatrixXd& J_out);

private:
    void buildQP(const Eigen::MatrixXd& J, const Eigen::VectorXd& v_des,
                 const Eigen::VectorXd& q, double dt);

    OsqpEigen::Solver solver_;
    Limits limits_;
    bool initialized_ = false;

    // RBDL model (copied from DLSSolver)
    RigidBodyDynamics::Model model_;
    Eigen::MatrixXd J_;
    unsigned int ee_id_ = 0;
};
