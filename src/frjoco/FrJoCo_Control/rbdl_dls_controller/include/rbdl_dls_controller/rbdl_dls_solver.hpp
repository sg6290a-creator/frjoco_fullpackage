#pragma once

#include <string>
#include <rbdl/rbdl.h>
#include <Eigen/Dense>

class DLSSolver {
public:
    bool init();

    // 목표 Twist와 현재 위치 q, dt로 dq를 계산
    Eigen::VectorXd calculate(const Eigen::VectorXd& q, const Eigen::VectorXd& Desired_V, double dt);

    // 현재 q와 FK로 EE의 Pose 계산
    Eigen::Isometry3d eePose(const Eigen::VectorXd& q);

    void setDqMax(double dq_max) { dq_max_ = dq_max; }

private:
    RigidBodyDynamics::Model model_;
    Eigen::MatrixXd J_;
    unsigned int ee_id_;
    double lambda_ = 1e-3;
    double dq_max_ = 1.0;  // rad/s
};
