#include <iostream>
#include <memory>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/examples/Pendulum/pendulum_plant.h"


typedef PiecewisePolynomial<double> PiecewisePolynomialType;

namespace drake {
namespace examples {
namespace pendulum {
namespace {

int do_main(int argc, char* argv[]) {
  auto pendulum = std::make_unique<PendulumPlant<double>>();

  using drake::solvers::MathematicalProgram;
  using drake::solvers::DecisionVariable;
  using drake::solvers::VariableList;
  using Eigen::MatrixXd;
  using Eigen::VectorXd;

  MathematicalProgram prog;

  const int kNumSteps = 30;
  const double kdt = 0.01;

  auto theta = prog.AddContinuousVariables(kNumSteps);
  auto thetadot = prog.AddContinuousVariables(kNumSteps);
//  auto costheta = prog.AddContinuousVariables(kNumSteps);
  auto sintheta = prog.AddContinuousVariables(kNumSteps);
  auto torque = prog.AddContinuousVariables(kNumSteps);

  for (int i=0; i < kNumSteps - 1; i++) {
    {
      MatrixXd A(1, 3);
      A << 1, kdt, -1;
      auto b = VectorXd::Zero(1);
      VariableList vars = {theta(i), thetadot(i), theta(i + 1)};
      prog.AddLinearEqualityConstraint(A, b, vars);
    }

    {
      MatrixXd A(1, 4);
      A << 1, kdt, pendulum->m() * pendulum->g() * kdt, -1;
      auto b = VectorXd::Zero(1);
      VariableList vars = {thetadot(i), torque(i), sintheta(i), thetadot(i + 1)};
      prog.AddLinearEqualityConstraint(A, b, vars);
    }
  }
  prog.AddBoundingBoxConstraint(VectorXd::Constant(1, 1, M_PI), VectorXd::Constant(1, 1, M_PI), {theta(kNumSteps - 1)});
  prog.AddBoundingBoxConstraint(VectorXd::Constant(1, 1, 0.0), VectorXd::Constant(1, 1, 0.0), {thetadot(kNumSteps - 1)});

  prog.AddQuadraticCost(MatrixXd::Identity(kNumSteps, kNumSteps), VectorXd::Zero(kNumSteps), torque);

  prog.Solve();
  for (int i=0; i < kNumSteps; i++) {
    std::cout << "theta: " << theta(i).value() << " thetadot: " << thetadot(i).value() << " torque: " << torque(i).value() << std::endl;
  }
  return 0;

}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::pendulum::do_main(argc, argv);
}
