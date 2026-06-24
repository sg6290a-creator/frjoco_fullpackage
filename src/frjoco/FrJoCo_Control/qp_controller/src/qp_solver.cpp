#include "qp_controller/qp_solver.hpp"

#include <rbdl/rbdl.h>
#include <rbdl/Kinematics.h>
#include <Eigen/Dense>
#include <cstring>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

// ────────────────────────────────────────────────────────────────
//  RBDL model (same kinematics as DLSSolver)
// ────────────────────────────────────────────────────────────────

bool QPSolver::initModel()
{
    auto I3 = [](double a, double b, double c) {
        Matrix3d I = Matrix3d::Zero();
        I(0,0)=a; I(1,1)=b; I(2,2)=c; return I;
    };

    Matrix3d R_base;
    R_base << -1,0,0, 0,-1,0, 0,0,1;
    unsigned int base_id = model_.AddBody(0,
        SpatialTransform(R_base, Vector3d(0,0,0)),
        Joint(JointTypeFixed),
        Body(4.0, Vector3d::Zero(), I3(0.00443333156,0.00443333156,0.0072)),
        "base_link_inertia");

    unsigned int shoulder_id = model_.AddBody(base_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d(0,0,0.0596)),
        Joint(JointTypeRevolute, Vector3d::UnitZ()),
        Body(3.7, Vector3d::Zero(), I3(0.010267495893,0.010267495893,0.00666)),
        "shoulder_link");

    unsigned int upper_arm_id = model_.AddBody(shoulder_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d(0,-0.0606,0.080)),
        Joint(JointTypeRevolute, Vector3d::UnitY()),
        Body(8.393, Vector3d::Zero(), I3(0.133885781862,0.133885781862,0.0151074)),
        "upper_arm_link");

    unsigned int forearm_id = model_.AddBody(upper_arm_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d(0,0,0.215)),
        Joint(JointTypeRevolute, Vector3d::UnitY()),
        Body(2.275, Vector3d::Zero(), I3(0.0312093550996,0.0312093550996,0.004095)),
        "forearm_link");

    unsigned int wrist1_id = model_.AddBody(forearm_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d(0,-0.001,0.215)),
        Joint(JointTypeRevolute, Vector3d::UnitY()),
        Body(1.219, Vector3d::Zero(), I3(0.00255989897604,0.00255989897604,0.0021942)),
        "wrist_1_link");

    unsigned int wrist2_id = model_.AddBody(wrist1_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d(0,-0.0425,0.0375)),
        Joint(JointTypeRevolute, Vector3d::UnitZ()),
        Body(1.219, Vector3d::Zero(), I3(0.00255989897604,0.00255989897604,0.0021942)),
        "wrist_2_link");

    unsigned int wrist3_id = model_.AddBody(wrist2_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d(0.0375,0,0.043)),
        Joint(JointTypeRevolute, Vector3d::UnitX()),
        Body(0.1, Vector3d::Zero(), I3(0.0001,0.0001,0.0001)),
        "wrist_3_link");

    Matrix3d zero_inertia = Matrix3d::Zero();
    ee_id_ = model_.AddBody(wrist3_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d::Zero()),
        Joint(JointTypeFixed),
        Body(0.0, Vector3d::Zero(), zero_inertia),
        "tool0");

    J_.resize(6, model_.dof_count);
    J_.setZero();
    return true;
}

Eigen::Isometry3d QPSolver::eePose(const Eigen::VectorXd& q)
{
    Eigen::Isometry3d pose;
    pose.translation() = CalcBodyToBaseCoordinates(model_, q, ee_id_, Vector3d::Zero(), true);
    pose.linear()      = CalcBodyWorldOrientation(model_, q, ee_id_, false).transpose();
    return pose;
}

void QPSolver::calcJacobian(const Eigen::VectorXd& q, Eigen::MatrixXd& J_out)
{
    J_.setZero();
    CalcPointJacobian6D(model_, q, ee_id_, Vector3d::Zero(), J_, true);
    // RBDL returns [ω(3); v(3)] — reorder to [v(3); ω(3)]
    J_out.resize(6, DOF);
    J_out << J_.bottomRows(3), J_.topRows(3);
}

// ────────────────────────────────────────────────────────────────
//  QP setup
// ────────────────────────────────────────────────────────────────

bool QPSolver::init(const Limits& limits)
{
    limits_ = limits;

    if (!initModel()) return false;

    // Solver settings
    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);
    solver_.settings()->setMaxIteration(200);
    solver_.settings()->setAbsoluteTolerance(1e-5);
    solver_.settings()->setRelativeTolerance(1e-5);
    solver_.settings()->setPolish(true);

    initialized_ = true;
    return true;
}

// ────────────────────────────────────────────────────────────────
//  QP: min  0.5 dq' H dq + g' dq
//      s.t. lb ≤ C dq ≤ ub
//
//  H = Jt'Jt + α·I   (6×6 task + regularisation)
//  g = -Jt' v_des
//
//  Constraints (2n rows):
//    row 0..n-1  : -dq_max·dt ≤ dq·dt ≤ dq_max·dt   (velocity)   → identity block
//    row n..2n-1 : q_min - q  ≤ dq·dt ≤ q_max - q    (joint lim)  → identity block
//  Both reduce to a single bound on dq with C = I_n (stacked).
// ────────────────────────────────────────────────────────────────

void QPSolver::buildQP(const Eigen::MatrixXd& J, const Eigen::VectorXd& v_des,
                       const Eigen::VectorXd& q, double dt)
{
    const int n = DOF;

    // ── Objective ───────────────────────────────────────────────
    // H = J'J + α I  (n×n)
    Eigen::MatrixXd H_dense = J.transpose() * J + limits_.alpha * Eigen::MatrixXd::Identity(n, n);
    Eigen::VectorXd g = -J.transpose() * v_des;

    // ── Constraints: identity (dq itself bounded) ────────────────
    // We have two sets of bounds on dq:
    //   1. velocity limit:  -dq_max ≤ dq ≤ dq_max
    //   2. joint limit:     (q_min-q)/dt ≤ dq ≤ (q_max-q)/dt
    // Combined: lb[i] = max(-dq_max, (q_min[i]-q[i])/dt)
    //           ub[i] = min( dq_max, (q_max[i]-q[i])/dt)
    // Constraint matrix = I_n

    Eigen::VectorXd lb(n), ub(n);
    for (int i = 0; i < n; ++i) {
        double jl_lo = (limits_.q_min(i) - q(i)) / dt;
        double jl_hi = (limits_.q_max(i) - q(i)) / dt;
        lb(i) = std::max(-limits_.dq_max, jl_lo);
        ub(i) = std::min( limits_.dq_max, jl_hi);
        // Safety: ensure lb ≤ ub
        if (lb(i) > ub(i)) lb(i) = ub(i) = 0.0;
    }

    // Sparse matrices for OSQP
    Eigen::SparseMatrix<double> H_sparse = H_dense.sparseView();
    Eigen::SparseMatrix<double> C_sparse(n, n);
    C_sparse.setIdentity();

    if (!solver_.isInitialized()) {
        solver_.data()->setNumberOfVariables(n);
        solver_.data()->setNumberOfConstraints(n);
        if (!solver_.data()->setHessianMatrix(H_sparse))   return;
        if (!solver_.data()->setGradient(g))               return;
        if (!solver_.data()->setLinearConstraintsMatrix(C_sparse)) return;
        if (!solver_.data()->setLowerBound(lb))            return;
        if (!solver_.data()->setUpperBound(ub))            return;
        solver_.initSolver();
    } else {
        solver_.updateHessianMatrix(H_sparse);
        solver_.updateGradient(g);
        solver_.updateBounds(lb, ub);
    }
}

Eigen::VectorXd QPSolver::solve(const Eigen::VectorXd& q,
                                 const Eigen::MatrixXd& J,
                                 const Eigen::VectorXd& v_des,
                                 double dt)
{
    if (!initialized_) return Eigen::VectorXd::Zero(DOF);

    buildQP(J, v_des, q, dt);

    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        return Eigen::VectorXd::Zero(DOF);
    }

    auto status = solver_.getStatus();
    if (status != OsqpEigen::Status::Solved &&
        status != OsqpEigen::Status::SolvedInaccurate) {
        return Eigen::VectorXd::Zero(DOF);
    }

    return solver_.getSolution();
}
