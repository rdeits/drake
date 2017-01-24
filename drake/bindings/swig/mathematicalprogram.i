%module(package="pydrake") mathematicalprogram

%include "exception_helper.i"
%include <std_string.i>
%include <windows.i>

%{
#ifdef SWIGPYTHON
  #define SWIG_FILE_WITH_INIT
  #include <Python.h>
#endif
#include "drake/solvers/mathematical_program.h"
using drake::VectorX;
%}

%include <typemaps.i>
%include <std_vector.i>
%include <std_map.i>

#define SWIG_SHARED_PTR_NAMESPACE std
// SWIG has built-in support for shared pointers, and can use either
// std::shared_ptr or boost::shared_ptr, since they provide similar enough
// interfaces. Even though the interface file is called 'boost_shared_ptr.i',
// the above #define tells SWIG to use the std:: implementation instead. Note
// that this does NOT result in any boost headers being included.
%include <boost_shared_ptr.i>

%include <eigen.i>

%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>)

%include "drake/common/eigen_autodiff_types.h"
%include "drake/common/polynomial.h"
%include "drake/common/symbolic_variable.h"
%include "drake/solvers/binding.h"
%include "drake/solvers/constraint.h"
%include "drake/solvers/decision_variable.h"
%include "drake/solvers/function.h"
%include "drake/solvers/mathematical_program_solver_interface.h"
%include "drake/solvers/mathematical_program.h"

