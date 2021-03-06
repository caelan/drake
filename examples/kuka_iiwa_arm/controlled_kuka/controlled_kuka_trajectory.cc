#include "drake/examples/kuka_iiwa_arm/controlled_kuka/controlled_kuka_trajectory.h"

#include <memory>
#include <string>
#include <vector>

#include "drake/common/find_resource.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::MatrixXd;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

using trajectories::PiecewisePolynomial;

const char kUrdfPath[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

}  // namespace

PiecewisePolynomial<double> MakeControlledKukaPlan() {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow(kUrdfPath), multibody::joints::kFixed, tree.get());

  // Creates a basic pointwise IK trajectory for moving the iiwa arm.
  // It starts in the zero configuration (straight up).
  VectorXd zero_conf = tree->getZeroConfiguration();
  VectorXd joint_lb = zero_conf - VectorXd::Constant(7, 0.01);
  VectorXd joint_ub = zero_conf + VectorXd::Constant(7, 0.01);

  PostureConstraint pc1(tree.get(), Vector2d(0, 0.5));
  VectorXi joint_idx(7);
  joint_idx << 0, 1, 2, 3, 4, 5, 6;
  pc1.setJointLimits(joint_idx, joint_lb, joint_ub);

  // Defines an end effector constraint and makes it active for the time span
  // from 1 to 3 seconds.
  Vector3d pos_end(0.6, 0, 0.325);
  Vector3d pos_lb = pos_end - Vector3d::Constant(0.005);
  Vector3d pos_ub = pos_end + Vector3d::Constant(0.005);
  WorldPositionConstraint wpc1(tree.get(), tree->FindBodyIndex("iiwa_link_ee"),
                               Vector3d::Zero(), pos_lb, pos_ub,
                               Vector2d(1, 3));

  // After the end effector constraint is released, applies the straight
  // up configuration again from time 4 to 5.9.
  PostureConstraint pc2(tree.get(), Vector2d(4, 5.9));
  pc2.setJointLimits(joint_idx, joint_lb, joint_ub);

  // Apply the same end effector constraint from time 6 to 9 of the demo.
  WorldPositionConstraint wpc2(tree.get(), tree->FindBodyIndex("iiwa_link_ee"),
                               Vector3d::Zero(), pos_lb, pos_ub,
                               Vector2d(6, 9));

  // For part of the time wpc2 is active, constrains the second joint while
  // preserving the end effector constraint.
  //
  // Variable `joint_position_start_idx` below is a collection of offsets into
  // the state vector referring to the positions of the joints to be
  // constrained.
  Eigen::VectorXi joint_position_start_idx(1);
  joint_position_start_idx(0) =
      tree->FindChildBodyOfJoint("iiwa_joint_2")->get_position_start_index();
  PostureConstraint pc3(tree.get(), Vector2d(6, 8));
  pc3.setJointLimits(joint_position_start_idx, Vector1d(0.7), Vector1d(0.8));

  const std::vector<double> kTimes{0.0, 2.0, 5.0, 7.0, 9.0};
  MatrixXd q_seed(tree->get_num_positions(), kTimes.size());
  MatrixXd q_nom(tree->get_num_positions(), kTimes.size());
  for (size_t i = 0; i < kTimes.size(); ++i) {
    // Zero configuration is a bad initial guess for IK, to be solved through
    // nonlinear optimization, as the robot configuration is in singularity,
    // and the gradient is zero. So we add 0.1 as the arbitrary perturbation
    // to the zero configuration.
    q_seed.col(i) =
        zero_conf + 0.1 * Eigen::VectorXd::Ones(tree->get_num_positions());
    q_nom.col(i) = zero_conf;
  }

  std::vector<RigidBodyConstraint*> constraint_array;
  constraint_array.push_back(&pc1);
  constraint_array.push_back(&wpc1);
  constraint_array.push_back(&pc2);
  constraint_array.push_back(&pc3);
  constraint_array.push_back(&wpc2);
  IKoptions ikoptions(tree.get());
  std::vector<int> info(kTimes.size(), 0);
  MatrixXd q_sol(tree->get_num_positions(), kTimes.size());
  std::vector<std::string> infeasible_constraint;

  inverseKinPointwise(tree.get(), kTimes.size(), kTimes.data(), q_seed, q_nom,
                      constraint_array.size(), constraint_array.data(),
                      ikoptions, &q_sol, info.data(), &infeasible_constraint);
  bool info_good = true;
  for (size_t i = 0; i < kTimes.size(); ++i) {
    drake::log()->info("INFO[{}] = {} ", i, info[i]);
    if (info[i] != 1) {
      info_good = false;
    }
  }
  printf("\n");

  if (!info_good) {
    throw std::runtime_error(
        "inverseKinPointwise failed to compute a valid solution.");
  }

  std::vector<MatrixXd> knots(kTimes.size());
  for (size_t i = 0; i < kTimes.size(); ++i) {
    // We only use column 0 of the matrix in knots (for joint positions),
    // so we write a vector.
    knots[i] = q_sol.col(i);
  }

  return PiecewisePolynomial<double>::FirstOrderHold(kTimes, knots);
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
