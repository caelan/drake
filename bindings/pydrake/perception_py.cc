#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/bindings/pydrake/util/cpp_param_pybind.h"
#include "drake/perception/depth_image_to_point_cloud.h"
#include "drake/perception/point_cloud.h"

namespace drake {
namespace pydrake {
namespace {

// TODO(eric.cousineau): At present, these bindings expose a minimal subset of
// features (e.g. no descriptors exposed, skipping performance-based args
// (like `skip_initialization`)). Bind these if they are useful.

void init_pc_flags(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::perception::pc_flags;
  constexpr auto& doc = pydrake_doc.drake.perception.pc_flags;

  {
    using Class = BaseField;
    constexpr auto& cls_doc = doc.BaseField;
    py::enum_<Class>(m, "BaseField", cls_doc.doc)
        .value("kNone", Class::kNone, cls_doc.kNone.doc)
        .value("kXYZs", Class::kXYZs, cls_doc.kXYZs.doc);
  }

  {
    using Class = Fields;
    constexpr auto& cls_doc = doc.Fields;
    py::class_<Class>(m, "Fields", cls_doc.doc)
        .def(py::init<BaseFieldT>(), py::arg("base_fields"),
             cls_doc.ctor.doc_4)
        .def("base_fields", &Class::base_fields, cls_doc.base_fields.doc)
        .def("has_base_fields", &Class::has_base_fields,
             cls_doc.has_base_fields.doc)
        // TODO(eric.cousineau): Bind bitwise operator overloads if they're
        // useful.
        .def(py::self == py::self)
        .def(py::self != py::self);
  }
}

void init_perception(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::perception;
  constexpr auto& doc = pydrake_doc.drake.perception;

  using systems::LeafSystem;
  using systems::sensors::CameraInfo;

  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.sensors");

  {
    using Class = PointCloud;
    constexpr auto& cls_doc = doc.PointCloud;
    py::class_<Class> cls(m, "PointCloud", cls_doc.doc);
    cls.attr("T") = GetPyParam<Class::T>()[0];
    cls.attr("D") = GetPyParam<Class::D>()[0];
    // N.B. Workaround linking error for `constexpr` bits.
    cls.attr("kDefaultValue") = Class::T{Class::kDefaultValue};
    cls
        .def_static(
            "IsDefaultValue", &Class::IsDefaultValue, py::arg("value"),
            cls_doc.IsDefaultValue.doc)
        .def_static(
            "IsInvalidValue", &Class::IsInvalidValue, py::arg("value"),
            cls_doc.IsInvalidValue.doc)
        .def(py::init<int, pc_flags::Fields>(),
             py::arg("new_size") = 0,
             py::arg("fields") = pc_flags::Fields(pc_flags::kXYZs),
             cls_doc.ctor.doc)
        .def(py::init<const PointCloud&>(), py::arg("other"),
             cls_doc.ctor.doc_2)
        .def("fields", &Class::fields, cls_doc.fields.doc)
        .def("size", &Class::size, cls_doc.size.doc)
        .def("resize",
             [](PointCloud* self, int new_size) {
               self->resize(new_size);
             },
             py::arg("new_size"), cls_doc.resize.doc)
        .def("has_xyzs", &Class::has_xyzs, cls_doc.has_xyzs.doc)
        .def("xyzs", &Class::xyzs, py_reference_internal, cls_doc.xyzs.doc)
        .def("mutable_xyzs", &Class::mutable_xyzs, py_reference_internal,
             cls_doc.mutable_xyzs.doc)
        .def("xyz", &Class::xyz, py::arg("i"), cls_doc.xyz.doc)
        .def("mutable_xyz", &Class::mutable_xyz, py::arg("i"),
             py_reference_internal, cls_doc.mutable_xyz.doc)
        .def("SetFrom",
             [](PointCloud* self, const PointCloud& other) {
               self->SetFrom(other);
             },
             py::arg("other"), cls_doc.SetFrom.doc);
  }

  pysystems::AddValueInstantiation<PointCloud>(m);

  {
    using Class = DepthImageToPointCloud;
    constexpr auto& cls_doc = doc.DepthImageToPointCloud;
    py::class_<Class, LeafSystem<double>>(
        m, "DepthImageToPointCloud", cls_doc.doc)
        .def(py::init<const CameraInfo&>(), py::arg("camera_info"),
             cls_doc.ctor.doc_3)
        .def("depth_image_input_port", &Class::depth_image_input_port,
             py_reference_internal, cls_doc.depth_image_input_port.doc)
        .def("point_cloud_output_port", &Class::point_cloud_output_port,
             py_reference_internal, cls_doc.point_cloud_output_port.doc);
  }
}

PYBIND11_MODULE(perception, m) {
  // N.B. To stick to directory structure, we do not define this in a
  // `pc_flags` submodule.
  init_pc_flags(m);
  init_perception(m);

  m.doc() = "Python bindings for //perception";
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
