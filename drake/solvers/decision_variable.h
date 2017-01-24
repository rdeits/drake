#pragma once

#include <cstddef>
#include <list>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

#include "drake/common/symbolic_variable.h"

namespace drake {
namespace solvers {
template <int rows, int cols>
using MatrixDecisionVariable =
    Eigen::Matrix<drake::symbolic::Variable, rows, cols>;
template <int rows>
using VectorDecisionVariable = MatrixDecisionVariable<rows, 1>;

// typedef Eigen::Matrix<drake::symbolic::Variable, Eigen::Dynamic, Eigen::Dynamic> MatrixXDecisionVariable;
using MatrixXDecisionVariable =
    MatrixDecisionVariable<Eigen::Dynamic, Eigen::Dynamic>;

// typedef Eigen::Matrix<drake::symbolic::Variable, Eigen::Dynamic, 1> VectorXDecisionVariable;
using VectorXDecisionVariable = VectorDecisionVariable<Eigen::Dynamic>;

using VariableRefList = std::list<Eigen::Ref<const VectorXDecisionVariable>>;

/**
 * Concatenates each element in \p var_list into a single Eigen vector of
 * decision variables, returns this concatenated vector.
 */
VectorXDecisionVariable ConcatenateVariableRefList(
    const VariableRefList &var_list);
}  // end namespace solvers
}  // end namespace drake
