#include <franka/gripper.h> 
#include <franka/gripper_state.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <string>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(gripper, handle)
{
  py::class_<franka::GripperState>(handle, "GripperState")
    .def(py::init<>())
    .def_readwrite("width", &franka::GripperState::width)
    .def_readwrite("max_width", &franka::GripperState::max_width)
    .def_readwrite("is_grasped", &franka::GripperState::is_grasped)
    .def_readwrite("temperature", &franka::GripperState::temperature)
    .def_readwrite("time", &franka::GripperState::time)
    .def("__repr__",
        [](const franka::GripperState &state) {
            return "<GripperState width='" + std::to_string(state.width) + "' max_width='" + std::to_string(state.max_width) + "' is_grasped='" + std::to_string(state.is_grasped) + "' temperature='" + std::to_string(state.temperature) + "' time='" + std::to_string(state.time.toMSec()) + "'>";
        }
    );

  py::class_<franka::Gripper>(handle, "Gripper")
    .def(py::init<const std::string&>())
    .def("homing", &franka::Gripper::homing)
    .def("grasp", &franka::Gripper::grasp)
    .def("move", &franka::Gripper::move)
    .def("stop", &franka::Gripper::stop)
    .def("readOnce", [](franka::Gripper &self) -> py::dict {
        franka::GripperState state = self.readOnce();
        return py::dict("width"_a=state.width, "max_width"_a=state.max_width, "is_grasped"_a=state.is_grasped, "temperature"_a=state.temperature, "time (ms)"_a=state.time.toMSec());
    }, "read the gripper state")
    .def("serverVersion", &franka::Gripper::serverVersion);
}