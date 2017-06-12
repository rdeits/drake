#include "drake/bindings/pybind11/pydrake_symbolic_types.h"
#include "drake/solvers/mathematical_program.h"

#include <cstddef>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>


namespace py = pybind11;
using std::string;

using drake::symbolic::Variable;
using drake::symbolic::Expression;
using drake::symbolic::Formula;
using drake::solvers::Binding;
using drake::solvers::MathematicalProgram;
using drake::solvers::EvaluatorBase;
using drake::solvers::Constraint;
using drake::solvers::LinearConstraint;
using drake::solvers::LinearEqualityConstraint;
using drake::solvers::BoundingBoxConstraint;
using drake::solvers::PositiveSemidefiniteConstraint;
using drake::solvers::Cost;
using drake::solvers::LinearCost;
using drake::solvers::QuadraticCost;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::MatrixXDecisionVariable;
using drake::solvers::SolutionResult;
using drake::solvers::MathematicalProgramSolverInterface;
using drake::solvers::SolverType;

/*
 * Register a Binding template, and add the corresponding overloads to the
 * pybind11 MathematicalProgram class.
 * @param scope The scope this will be added to (e.g., the module).
 * @param pprog_cls Pointer to Python MathematicalProgram class. Overloads will
 * be added to the binding
 * @param name Name of the Cost / Constraint class.
 */
template <typename C>
auto RegisterBinding(py::handle* pscope,
                     py::class_<MathematicalProgram>* pprog_cls,
                     const string& name) {
  auto& scope = *pscope;
  auto& prog_cls = *pprog_cls;
  typedef Binding<C> B;
  string pyname = "Binding_" + name;
  auto binding_cls = py::class_<B>(scope, pyname.c_str())
    .def("constraint", &B::constraint)
    .def("variables", &B::variables);
  // Register overloads for MathematicalProgram class
  prog_cls
    .def("EvalBindingAtSolution",
          (Eigen::VectorXd(MathematicalProgram::*)(
           const B&) const)
          &MathematicalProgram::EvalBindingAtSolution);
  return binding_cls;
}

PYBIND11_PLUGIN(_pydrake_mathematicalprogram) {
  py::module m("_pydrake_mathematicalprogram",
               "Drake MathematicalProgram Bindings");

  py::object variable = (py::object)
    py::module::import("pydrake.symbolic").attr("Variable");
  py::object expression = (py::object)
    py::module::import("pydrake.symbolic").attr("Expression");
  py::object formula = (py::object)
    py::module::import("pydrake.symbolic").attr("Formula");

  py::class_<MathematicalProgramSolverInterface>(
    m, "MathematicalProgramSolverInterface")
    .def("available", &MathematicalProgramSolverInterface::available)
    .def("Solve", &MathematicalProgramSolverInterface::Solve)
    .def("solver_type", &MathematicalProgramSolverInterface::solver_type)
    .def("SolverName", &MathematicalProgramSolverInterface::SolverName);

  py::enum_<SolverType>(m, "SolverType")
    .value("kDReal", SolverType::kDReal)
    .value("kEqualityConstrainedQP", SolverType::kEqualityConstrainedQP)
    .value("kGurobi", SolverType::kGurobi)
    .value("kIpopt", SolverType::kIpopt)
    .value("kLinearSystem", SolverType::kLinearSystem)
    .value("kMobyLCP", SolverType::kMobyLCP)
    .value("kMosek", SolverType::kMosek)
    .value("kNlopt", SolverType::kNlopt)
    .value("kSnopt", SolverType::kSnopt);

  py::class_<MathematicalProgram> prog_cls(m, "MathematicalProgram");
  prog_cls
    .def(py::init<>())
    .def("NewContinuousVariables", (VectorXDecisionVariable
          (MathematicalProgram::*)(
          int,
          const std::string&))
         &MathematicalProgram::NewContinuousVariables,
         py::arg("rows"),
         py::arg("name") = "x")
    .def("NewContinuousVariables", (MatrixXDecisionVariable
          (MathematicalProgram::*)(
          int,
          int,
          const std::string&))
         &MathematicalProgram::NewContinuousVariables,
         py::arg("rows"),
         py::arg("cols"),
         py::arg("name") = "x")
    .def("NewBinaryVariables", (VectorXDecisionVariable
         (MathematicalProgram::*)(
         int,
         const std::string&))
         &MathematicalProgram::NewBinaryVariables,
         py::arg("rows"),
         py::arg("name") = "b")
    .def("NewBinaryVariables", (MatrixXDecisionVariable
         (MathematicalProgram::*)(
         int,
         int,
         const std::string&))
         &MathematicalProgram::NewBinaryVariables,
         py::arg("rows"),
         py::arg("cols"),
         py::arg("name") = "b")
    .def("NewSymmetricContinuousVariables", (MatrixXDecisionVariable
         (MathematicalProgram::*)(
          size_t,
          const std::string&))
         &MathematicalProgram::NewSymmetricContinuousVariables,
         py::arg("rows"),
         py::arg("name") = "Symmetric")
    .def("AddLinearConstraint",
         (Binding<LinearConstraint>
          (MathematicalProgram::*)(
          const Expression&,
          double,
          double))
          &MathematicalProgram::AddLinearConstraint)
    .def("AddLinearConstraint",
         (Binding<LinearConstraint>
          (MathematicalProgram::*)(
          const Formula&))
          &MathematicalProgram::AddLinearConstraint)
    .def("AddPositiveSemidefiniteConstraint",
         (Binding<PositiveSemidefiniteConstraint>
          (MathematicalProgram::*)(
          const Eigen::Ref<const MatrixXDecisionVariable>&))
         &MathematicalProgram::AddPositiveSemidefiniteConstraint)
    .def("AddLinearCost",
         (Binding<LinearCost>
          (MathematicalProgram::*)(
          const Expression&))
          &MathematicalProgram::AddLinearCost)
    .def("AddQuadraticCost", (Binding<QuadraticCost>
         (MathematicalProgram::*)(
          const Eigen::Ref<const Eigen::MatrixXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const VectorXDecisionVariable>&))
         &MathematicalProgram::AddQuadraticCost)
    .def("AddQuadraticCost", (Binding<QuadraticCost>
         (MathematicalProgram::*)(const Expression&))
         &MathematicalProgram::AddQuadraticCost)
    .def("Solve", &MathematicalProgram::Solve)
    .def("linear_constraints", &MathematicalProgram::linear_constraints)
    .def("linear_equality_constraints",
         &MathematicalProgram::linear_equality_constraints)
    .def("bounding_box_constraints",
         &MathematicalProgram::bounding_box_constraints)
    .def("linear_costs", &MathematicalProgram::linear_costs)
    .def("quadratic_costs", &MathematicalProgram::quadratic_costs)
    .def("FindDecisionVariableIndex",
         &MathematicalProgram::FindDecisionVariableIndex)
    .def("num_vars", &MathematicalProgram::num_vars)
    .def("GetSolution", [](const MathematicalProgram& prog,
                            const Variable& var) {
      return prog.GetSolution(var);
    })
    .def("GetSolution",
         [](const MathematicalProgram& prog,
            const VectorXDecisionVariable& var) {
      return prog.GetSolution(var);
    })
    .def("GetSolution",
         [](const MathematicalProgram& prog,
            const MatrixXDecisionVariable& var) {
      return prog.GetSolution(var);
    })
    .def("SetSolverOption", (void(MathematicalProgram::*)(
         SolverType, const std::string&, double))
         &MathematicalProgram::SetSolverOption)
    .def("SetSolverOption", (void(MathematicalProgram::*)(
         SolverType, const std::string&, int))
         &MathematicalProgram::SetSolverOption)
    .def("SetSolverOption", (void(MathematicalProgram::*)(
         SolverType, const std::string&, const std::string&))
         &MathematicalProgram::SetSolverOption);

  py::enum_<SolutionResult>(m, "SolutionResult")
    .value("kSolutionFound", SolutionResult::kSolutionFound)
    .value("kInvalidInput", SolutionResult::kInvalidInput)
    .value("kInfeasibleConstraints",
           SolutionResult::kInfeasibleConstraints)
    .value("kUnbounded", SolutionResult::kUnbounded)
    .value("kUnknownError", SolutionResult::kUnknownError)
    .value("kInfeasible_Or_Unbounded",
           SolutionResult::kInfeasible_Or_Unbounded);

  // Assign the wrapped Constraint class to the name 'constraint'
  // so we can use it in this file to indicate that the other constraint
  // types inherit from it.
  py::class_<EvaluatorBase, std::shared_ptr<EvaluatorBase>>(m, "EvaluatorBase")
    .def("num_constraints", &EvaluatorBase::num_constraints)
    .def("lower_bound", &EvaluatorBase::lower_bound)
    .def("upper_bound", &EvaluatorBase::upper_bound);
  py::class_<Constraint, EvaluatorBase, std::shared_ptr<Constraint>>(
    m, "Constraint");

  py::class_<LinearConstraint, Constraint, std::shared_ptr<LinearConstraint>>(
    m, "LinearConstraint")
    .def("A", &LinearConstraint::A);

  py::class_<LinearEqualityConstraint, LinearConstraint,
             std::shared_ptr<LinearEqualityConstraint>>(
    m, "LinearEqualityConstraint");

  py::class_<BoundingBoxConstraint, LinearConstraint,
             std::shared_ptr<BoundingBoxConstraint>>(
    m, "BoundingBoxConstraint");

  py::class_<PositiveSemidefiniteConstraint, Constraint,
             std::shared_ptr<PositiveSemidefiniteConstraint>>(
    m, "PositiveSemidefiniteConstraint");

  RegisterBinding<LinearConstraint>(&m, &prog_cls, "LinearConstraint");
  RegisterBinding<LinearEqualityConstraint>(&m, &prog_cls,
                                            "LinearEqualityConstraint");
  RegisterBinding<BoundingBoxConstraint>(&m, &prog_cls,
                                         "BoundingBoxConstraint");
  RegisterBinding<PositiveSemidefiniteConstraint>(&m, &prog_cls,
    "PositiveSemidefiniteConstraint");

  // Mirror procedure for costs
  py::class_<Cost, std::shared_ptr<Cost>> cost(
    m, "Cost");

  py::class_<LinearCost, Cost, std::shared_ptr<LinearCost>>(
    m, "LinearCost")
    .def("a", &LinearCost::a)
    .def("b", &LinearCost::b);

  py::class_<QuadraticCost, Cost,
             std::shared_ptr<QuadraticCost>>(m, "QuadraticCost")
    .def("Q", &QuadraticCost::Q)
    .def("b", &QuadraticCost::b)
    .def("c", &QuadraticCost::c);

  RegisterBinding<LinearCost>(&m, &prog_cls, "LinearCost");
  RegisterBinding<QuadraticCost>(&m, &prog_cls, "QuadraticCost");

  return m.ptr();
}
