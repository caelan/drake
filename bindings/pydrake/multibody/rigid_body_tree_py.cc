#include <iostream>
#include <memory>

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/parsers/package_map.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"

using std::make_unique;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(rigid_body_tree, m) {
  m.doc() = "Bindings for the RigidBodyTree class";

  using drake::multibody::joints::FloatingBaseType;
  using drake::parsers::PackageMap;
  namespace sdf = drake::parsers::sdf;
  using std::shared_ptr;

  py::module::import("pydrake.multibody.parsers");
  py::module::import("pydrake.multibody.shapes");
  py::module::import("pydrake.util.eigen_geometry");

  py::enum_<FloatingBaseType>(m, "FloatingBaseType")
    .value("kFixed", FloatingBaseType::kFixed)
    .value("kRollPitchYaw", FloatingBaseType::kRollPitchYaw)
    .value("kQuaternion", FloatingBaseType::kQuaternion);

  // TODO(eric.cousineau): Try to decouple these APIs so that `rigid_body_tree`
  // and `parsers` do not form a dependency cycle.
  py::class_<RigidBodyTree<double>>(m, "RigidBodyTree")
    .def(py::init<>())
    .def(py::init(
         [](const std::string& urdf_filename,
            const PackageMap& pmap,
            FloatingBaseType floating_base_type
            ) {
          auto instance = make_unique<RigidBodyTree<double>>();
          drake::parsers::urdf::
            AddModelInstanceFromUrdfFileSearchingInRosPackages(
            urdf_filename,
            pmap,
            floating_base_type,
            nullptr,
            instance.get());
          return instance;
        }),
        py::arg("urdf_filename"),
        py::arg("package_map"),
        py::arg("floating_base_type") = FloatingBaseType::kRollPitchYaw)
    .def(py::init(
         [](const std::string& urdf_filename,
            FloatingBaseType floating_base_type
            ) {
          auto instance = make_unique<RigidBodyTree<double>>();
          drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
            urdf_filename, floating_base_type, instance.get());
          return instance;
        }),
        py::arg("urdf_filename"),
        py::arg("floating_base_type") = FloatingBaseType::kRollPitchYaw)
    .def(py::init(
         [](const std::string& urdf_filename,
            const std::string& joint_type) {
            // FIXED = 0, ROLLPITCHYAW = 1, QUATERNION = 2
            FloatingBaseType floating_base_type;
            std::cerr << "WARNING: passing joint_type as a string is "
              << "deprecated. Please pass a FloatingBaseType value such as "
              << "FloatingBaseType.kRollPitchYaw" << std::endl;
            if (joint_type == "FIXED") {
              floating_base_type = FloatingBaseType::kFixed;
            } else if (joint_type == "ROLLPITCHYAW") {
              floating_base_type = FloatingBaseType::kRollPitchYaw;
            } else if (joint_type == "QUATERNION") {
              floating_base_type = FloatingBaseType::kQuaternion;
            } else {
              throw(std::invalid_argument("Joint type not supported"));
            }
            auto instance = make_unique<RigidBodyTree<double>>();
            drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                urdf_filename, floating_base_type, instance.get());
            return instance;
        }),
        py::arg("urdf_filename"), py::arg("joint_type") = "ROLLPITCHYAW"
      )
    .def("getRandomConfiguration", [](const RigidBodyTree<double>& tree) {
      std::default_random_engine generator(std::random_device {}());
      return tree.getRandomConfiguration(generator);
    })
    .def("getZeroConfiguration", &RigidBodyTree<double>::getZeroConfiguration)
    .def("doKinematics", [](const RigidBodyTree<double>& tree,
                            const Eigen::VectorXd& q) {
      return tree.doKinematics(q);
    })
    .def("doKinematics", [](const RigidBodyTree<double>& tree,
                            const Eigen::VectorXd& q,
                            const Eigen::VectorXd& v) {
      return tree.doKinematics(q, v);
    })
    .def("doKinematics", [](const RigidBodyTree<double>& tree,
                            const VectorX<AutoDiffXd>& q) {
      return tree.doKinematics(q);
    })
    .def("doKinematics", [](const RigidBodyTree<double>& tree,
                            const VectorX<AutoDiffXd>& q,
                            const VectorX<AutoDiffXd>& v) {
      return tree.doKinematics(q, v);
    })
    .def("CalcBodyPoseInWorldFrame", [](const RigidBodyTree<double>& tree,
                                        const KinematicsCache<double> &cache,
                                        const RigidBody<double> &body) {
      return tree.CalcBodyPoseInWorldFrame(cache, body).matrix();
    })
    .def("centerOfMass", &RigidBodyTree<double>::centerOfMass<double>,
         py::arg("cache"),
         py::arg("model_instance_id_set") =
           RigidBodyTreeConstants::default_model_instance_id_set)
    .def("centerOfMassJacobian",
         &RigidBodyTree<double>::centerOfMassJacobian<double>,
         py::arg("cache"),
         py::arg("model_instance_id_set") =
           RigidBodyTreeConstants::default_model_instance_id_set,
         py::arg("in_terms_of_qdot") = false)
    .def("get_num_model_instances", &RigidBodyTree<double>::get_num_model_instances)
    .def("get_num_bodies", &RigidBodyTree<double>::get_num_bodies)
    .def("get_num_frames", &RigidBodyTree<double>::get_num_frames)
    .def("get_num_actuators", &RigidBodyTree<double>::get_num_actuators)
    .def("getBodyOrFrameName",
         &RigidBodyTree<double>::getBodyOrFrameName,
         py::arg("body_or_frame_id"))
    .def("number_of_positions", &RigidBodyTree<double>::get_num_positions)
    .def("get_num_positions", &RigidBodyTree<double>::get_num_positions)
    .def("number_of_velocities", &RigidBodyTree<double>::get_num_velocities)
    .def("get_num_velocities", &RigidBodyTree<double>::get_num_velocities)
    .def("get_body", &RigidBodyTree<double>::get_body,
         py::return_value_policy::reference)
    .def("get_position_name", &RigidBodyTree<double>::get_position_name)
    .def("collisionDetect", [](RigidBodyTree<double>& tree,
                               const KinematicsCache<double>& cache,
                               const std::vector<int>& bodies_idx,
                               bool use_margins) {
      Eigen::VectorXd phi;
      Eigen::Matrix3Xd normal, xA, xB;
      std::vector<int> bodyA_idx, bodyB_idx;
      tree.collisionDetect(cache, 
        phi, normal, xA, xB, bodyA_idx, bodyB_idx, 
        bodies_idx, use_margins);
      return std::make_tuple(phi, normal, xA, xB, bodyA_idx, bodyB_idx);
    })
    .def("allCollisions", [](RigidBodyTree<double>& tree,
                               const KinematicsCache<double>& cache,
                               bool use_margins) {
      Eigen::Matrix3Xd xA, xB;
      std::vector<int> bodyA_idx, bodyB_idx;
      tree.allCollisions(cache, 
          bodyA_idx, bodyB_idx, xA, xB, use_margins);
      return std::make_tuple(xA, xB, bodyA_idx, bodyB_idx);
     })
    .def("transformPoints", [](const RigidBodyTree<double>& tree,
                               const KinematicsCache<double>& cache,
                               const Eigen::Matrix<double, 3,
                                                   Eigen::Dynamic>& points,
                               int from_body_or_frame_ind,
                               int to_body_or_frame_ind) {
      return tree.transformPoints(cache, points,
                                  from_body_or_frame_ind, to_body_or_frame_ind);
    })
    .def("transformPoints", [](const RigidBodyTree<double>& tree,
                               const KinematicsCache<AutoDiffXd>& cache,
                               const Eigen::Matrix<double, 3,
                                                   Eigen::Dynamic>& points,
                               int from_body_or_frame_ind,
                               int to_body_or_frame_ind) {
      return tree.transformPoints(cache, points,
                                  from_body_or_frame_ind, to_body_or_frame_ind);
    })
    .def("relativeTransform", [](const RigidBodyTree<double>& tree,
                                  const KinematicsCache<double>& cache,
                                  int base_or_frame_ind,
                                  int body_or_frame_ind) {
      return tree.relativeTransform(cache, base_or_frame_ind,
        body_or_frame_ind).matrix();
    })
    .def("relativeTransform", [](const RigidBodyTree<double>& tree,
                                  const KinematicsCache<AutoDiffXd>& cache,
                                  int base_or_frame_ind,
                                  int body_or_frame_ind) {
      return tree.relativeTransform(cache, base_or_frame_ind,
        body_or_frame_ind).matrix();
    })
    .def("addFrame", &RigidBodyTree<double>::addFrame)
    .def("FindModelInstanceBodies", &RigidBodyTree<double>::FindModelInstanceBodies)
    .def("FindBaseBodies", &RigidBodyTree<double>::FindBaseBodies, py::arg("model_id") = -1)
    .def("FindBody", [](const RigidBodyTree<double>& self,
                        const std::string& body_name,
                        const std::string& model_name = "",
                        int model_id = -1) {
      return self.FindBody(body_name, model_name, model_id);
    }, py::arg("body_name"),
       py::arg("model_name") = "",
       py::arg("model_id") = -1,
       py::return_value_policy::reference)
    .def("world",
         static_cast<RigidBody<double>& (RigidBodyTree<double>::*)()>(
             &RigidBodyTree<double>::world),
         py::return_value_policy::reference)
    .def("FindBodyIndex", &RigidBodyTree<double>::FindBodyIndex,
         py::arg("body_name"), py::arg("model_instance_id") = -1)
    .def("findJointId", &RigidBodyTree<double>::FindIndexOfChildBodyOfJoint, // findJointId
         py::arg("joint_name"), py::arg("model_id") = -1)
    .def("findFrame", &RigidBodyTree<double>::findFrame,
         py::arg("frame_name"), py::arg("model_id") = -1)
    .def("FindCollisionElement", &RigidBodyTree<double>::FindCollisionElement)
    .def("getTerrainContactPoints",
         [](const RigidBodyTree<double>& self,
            const RigidBody<double>& body,
            const std::string& group_name = "") {
          auto pts = Eigen::Matrix3Xd(3, 0);
          self.getTerrainContactPoints(body, &pts, group_name);
          return pts;
        }, py::arg("body"), py::arg("group_name")="")
    .def("massMatrix", &RigidBodyTree<double>::massMatrix<double>)
    .def("dynamicsBiasTerm", &RigidBodyTree<double>::dynamicsBiasTerm<double>,
         py::arg("cache"), py::arg("external_wrenches"),
         py::arg("include_velocity_terms") = true)
    .def("inverseDynamics", &RigidBodyTree<double>::inverseDynamics<double>,
         py::arg("cache"),
         py::arg("external_wrenches"),
         py::arg("vd"),
         py::arg("include_velocity_terms") = true)
    .def("frictionTorques",
         [](const RigidBodyTree<double>* self, const VectorX<double>& v) {
           return self->frictionTorques(v);
         })
    .def_readonly("B", &RigidBodyTree<double>::B)
    .def_readonly("joint_limit_min", &RigidBodyTree<double>::joint_limit_min)
    .def_readonly("joint_limit_max", &RigidBodyTree<double>::joint_limit_max);

  py::class_<KinematicsCache<double> >(m, "KinematicsCacheDouble")
    .def("getQ", &KinematicsCache<double>::getQ)
    .def("getV", &KinematicsCache<double>::getV)
    .def("getX", &KinematicsCache<double>::getX);
  py::class_<KinematicsCache<AutoDiffXd> >(m, "KinematicsCacheAutoDiffXd");

  py::class_<RigidBody<double> >(m, "RigidBody")
    .def("get_name", &RigidBody<double>::get_name)
    .def("get_model_name", &RigidBody<double>::get_model_name)
    .def("get_model_instance_id", &RigidBody<double>::get_model_instance_id)
    .def("has_joint", &RigidBody<double>::has_joint)
    .def("has_parent_body", &RigidBody<double>::has_parent_body)
    .def("get_parent", &RigidBody<double>::get_parent)
    .def("get_body_index", &RigidBody<double>::get_body_index)
    .def("get_center_of_mass", &RigidBody<double>::get_center_of_mass)
    .def("get_visual_elements", &RigidBody<double>::get_visual_elements)
    .def("IsRigidlyFixedToWorld", &RigidBody<double>::IsRigidlyFixedToWorld)
    .def("adjacentTo", &RigidBody<double>::adjacentTo)
    .def("CanCollideWith", &RigidBody<double>::CanCollideWith)
    .def("get_collision_element_ids", &RigidBody<double>::get_collision_element_ids)
    .def("get_group_to_collision_ids_map", &RigidBody<double>::get_group_to_collision_ids_map)
    .def("get_position_start_index", &RigidBody<double>::get_position_start_index)
    .def("get_velocity_start_index", &RigidBody<double>::get_velocity_start_index)
    //.def("get_joint_name", [](const RigidBody<double>& body) {
    //    if (!body.has_joint()) {
    //      return std::string();
    //    }
    //    return body.getJoint().get_name();
    //  })
    .def("get_num_positions", [](const RigidBody<double>& body) {
        if (!body.has_joint()) {
          return 0;
        }
        return body.getJoint().get_num_positions();
      })
    .def("get_num_velocities", [](const RigidBody<double>& body) {
        if (!body.has_joint()) {
          return 0;
        }
        return body.getJoint().get_num_velocities();
      }); // TODO: name, floating, fixed

  py::class_<RigidBodyFrame<double>,
             shared_ptr<RigidBodyFrame<double> > >(m, "RigidBodyFrame")
    .def(
        py::init<
            const std::string&,
            RigidBody<double>*,
            const Eigen::VectorXd&,
            const Eigen::VectorXd&>(),
        py::arg("name"), py::arg("body"),
        py::arg("xyz") = Eigen::Vector3d::Zero(),
        py::arg("rpy") = Eigen::Vector3d::Zero())
    .def(
        py::init<
            const std::string&,
            RigidBody<double>*,
            const Eigen::Isometry3d&>(),
        py::arg("name"), py::arg("body"),
        py::arg("transform_to_body"))
    .def("get_name", &RigidBodyFrame<double>::get_name)
    .def("get_rigid_body", &RigidBodyFrame<double>::get_rigid_body)
    .def("get_model_instance_id", &RigidBodyFrame<double>::get_model_instance_id)
    .def("get_frame_index", &RigidBodyFrame<double>::get_frame_index);

  m.def("AddModelInstanceFromUrdfStringSearchingInRosPackages",
        py::overload_cast<const std::string&, const PackageMap&,
                          const std::string&, const FloatingBaseType,
                          shared_ptr<RigidBodyFrame<double>>,
                          RigidBodyTree<double>*>(
            &parsers::urdf::
                AddModelInstanceFromUrdfStringSearchingInRosPackages));
  m.def("AddModelInstancesFromSdfString",
        py::overload_cast<const std::string&, const FloatingBaseType,
                          shared_ptr<RigidBodyFrame<double>>,
                          RigidBodyTree<double>*>(
            &sdf::AddModelInstancesFromSdfString));
  m.def("AddModelInstancesFromSdfStringSearchingInRosPackages",
        py::overload_cast<
            const std::string&, const PackageMap&, const FloatingBaseType,
            shared_ptr<RigidBodyFrame<double>>, RigidBodyTree<double>*>(
            &sdf::AddModelInstancesFromSdfStringSearchingInRosPackages)),
  m.def("AddFlatTerrainToWorld", &multibody::AddFlatTerrainToWorld,
        py::arg("tree"), py::arg("box_size") = 1000,
        py::arg("box_depth") = 10);
}

}  // namespace pydrake
}  // namespace drake
