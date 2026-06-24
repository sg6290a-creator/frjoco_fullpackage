#include "rbdl_dls_controller/rbdl_dls_solver.hpp"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


bool DLSSolver::init()
{
    auto I3 = [](double ixx, double iyy, double izz) {
        Matrix3d I = Matrix3d::Zero();
        I(0,0) = ixx; I(1,1) = iyy; I(2,2) = izz;
        return I;
    };

    // fixed: base_link → base_link_inertia, rpy=(0,0,π)
    Matrix3d R_base;
    R_base << -1, 0, 0,
               0,-1, 0,
               0, 0, 1;
    unsigned int base_id = model_.AddBody(0,
        SpatialTransform(R_base, Vector3d(0, 0, 0)),
        Joint(JointTypeFixed),
        Body(4.0, Vector3d::Zero(), I3(0.00443333156, 0.00443333156, 0.0072)),
        "base_link_inertia");

    // joint 1: shoulder_pan, xyz=(0, 0, 0.0596), axis=Z
    unsigned int shoulder_id = model_.AddBody(base_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d(0, 0, 0.0596)),
        Joint(JointTypeRevolute, Vector3d::UnitZ()),
        Body(3.7, Vector3d::Zero(), I3(0.010267495893, 0.010267495893, 0.00666)),
        "shoulder_link");

    // joint 2: shoulder_lift, xyz=(0, -0.0606, 0.080), axis=Y
    unsigned int upper_arm_id = model_.AddBody(shoulder_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d(0, -0.0606, 0.080)),
        Joint(JointTypeRevolute, Vector3d::UnitY()),
        Body(8.393, Vector3d::Zero(), I3(0.133885781862, 0.133885781862, 0.0151074)),
        "upper_arm_link");

    // joint 3: elbow, xyz=(0, 0, 0.215), axis=Y
    unsigned int forearm_id = model_.AddBody(upper_arm_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d(0, 0, 0.215)),
        Joint(JointTypeRevolute, Vector3d::UnitY()),
        Body(2.275, Vector3d::Zero(), I3(0.0312093550996, 0.0312093550996, 0.004095)),
        "forearm_link");

    // joint 4: wrist_1, xyz=(0, -0.001, 0.215), axis=Y
    unsigned int wrist1_id = model_.AddBody(forearm_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d(0, -0.001, 0.215)),
        Joint(JointTypeRevolute, Vector3d::UnitY()),
        Body(1.219, Vector3d::Zero(), I3(0.00255989897604, 0.00255989897604, 0.0021942)),
        "wrist_1_link");

    // joint 5: wrist_2, xyz=(0, -0.0425, 0.0375), axis=Z
    unsigned int wrist2_id = model_.AddBody(wrist1_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d(0, -0.0425, 0.0375)),
        Joint(JointTypeRevolute, Vector3d::UnitZ()),
        Body(1.219, Vector3d::Zero(), I3(0.00255989897604, 0.00255989897604, 0.0021942)),
        "wrist_2_link");

    // joint 6: wrist_3, xyz=(0.0375, 0, 0.043), axis=X
    unsigned int wrist3_id = model_.AddBody(wrist2_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d(0.0375, 0, 0.043)),
        Joint(JointTypeRevolute, Vector3d::UnitX()),
        Body(0.1, Vector3d::Zero(), I3(0.0001, 0.0001, 0.0001)),
        "wrist_3_link");

    // fixed: wrist_3_link → tool0
    ee_id_ = model_.AddBody(wrist3_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d::Zero()),
        Joint(JointTypeFixed),
        Body(0.0, Vector3d(0,0,0), Vector3d(0,0,0)),
        "tool0");

    J_.resize(6, model_.dof_count);
    J_.setZero();
    return true;
}


Eigen::VectorXd DLSSolver::calculate(const Eigen::VectorXd& q, const Eigen::VectorXd& v_t, double dt)
{
    J_.setZero();
    // world frame Jacobian: [ω(3); v(3)] × dq
    CalcPointJacobian6D(model_, q, ee_id_, Vector3d::Zero(), J_, true);

    // v_t는 [v; ω] 순서이므로 J를 [v; ω]로 재배열
    Eigen::MatrixXd J_reordered(6, model_.dof_count);
    J_reordered << J_.bottomRows(3), J_.topRows(3);

    Eigen::MatrixXd J_T = J_reordered.transpose();
    Eigen::MatrixXd H = J_reordered * J_T + lambda_ * lambda_ * Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd dq = J_T * H.inverse() * v_t;

    // proportional scaling: 가장 빠른 관절 기준으로 전체 스케일
    double max_val = dq.cwiseAbs().maxCoeff();
    if (max_val > dq_max_)
        dq *= dq_max_ / max_val;

    return q + dq * dt;
}


Eigen::Isometry3d DLSSolver::eePose(const Eigen::VectorXd& q)
{
    Eigen::Isometry3d pose;
    pose.translation() = CalcBodyToBaseCoordinates(model_, q, ee_id_, Vector3d::Zero(), true);
    pose.linear()      = CalcBodyWorldOrientation(model_, q, ee_id_, false).transpose();
    return pose;
}
