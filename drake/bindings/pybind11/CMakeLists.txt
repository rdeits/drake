set(PYDRAKE_INSTALL_DIR lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages/pydrake)

if(APPLE)
  string(REPLACE "-Wl,-undefined -Wl,error" "" CMAKE_MODULE_LINKER_FLAGS ${CMAKE_MODULE_LINKER_FLAGS})
else()
  string(REPLACE "-Wl,--no-undefined" "" CMAKE_MODULE_LINKER_FLAGS ${CMAKE_MODULE_LINKER_FLAGS})
endif()

# Look up the python module suffix (which varies by python version).
# This uses the method from FindPythonLibsNew.cmake shipped with pybind11.
execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c "from distutils import sysconfig; print(sysconfig.get_config_var('SO'))"
    RESULT_VARIABLE PYTHON_MODULE_EXTENSION_SUCCESS
    OUTPUT_VARIABLE PYTHON_MODULE_EXTENSION
    ERROR_VARIABLE PYTHON_MODULE_EXTENSION_ERROR_VALUE
    OUTPUT_STRIP_TRAILING_WHITESPACE)

if(NOT PYTHON_MODULE_EXTENSION_SUCCESS MATCHES 0)
  message(FATAL_ERROR "Could not determine the python module extension:\n${PYTHON_MODULE_EXTENSION_ERROR_VALUE}")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-shadow")
pybind11_add_module(_pydrake_rbtree pydrake_rbtree.cc)
target_link_libraries(_pydrake_rbtree PRIVATE drakeRBM drakeMultibodyParsers Eigen3::Eigen)
install(TARGETS _pydrake_rbtree DESTINATION ${PYDRAKE_INSTALL_DIR})

pybind11_add_module(_pydrake_autodiffutils pydrake_autodiffutils.cc)
target_link_libraries(_pydrake_autodiffutils PRIVATE drakeRBM drakeMultibodyParsers Eigen3::Eigen)
install(TARGETS _pydrake_autodiffutils DESTINATION ${PYDRAKE_INSTALL_DIR})

pybind11_add_module(_pydrake_ik pydrake_ik.cc)
target_link_libraries(_pydrake_ik PRIVATE drakeIK Eigen3::Eigen)
install(TARGETS _pydrake_ik DESTINATION ${PYDRAKE_INSTALL_DIR}/solvers)

pybind11_add_module(_pybind_mathematicalprogram pybind_mathematicalprogram.cc)
target_link_libraries(_pybind_mathematicalprogram PRIVATE drakeOptimization Eigen3::Eigen)
install(TARGETS _pybind_mathematicalprogram DESTINATION ${PYDRAKE_INSTALL_DIR}/solvers)

if(gurobi_FOUND)
  pybind11_add_module(_pybind_gurobi pybind_gurobi.cc)
  target_link_libraries(_pybind_gurobi PRIVATE drakeOptimization)
  set_target_properties(_pybind_gurobi PROPERTIES OUTPUT_NAME "gurobi")
  install(TARGETS _pybind_gurobi DESTINATION ${PYDRAKE_INSTALL_DIR}/solvers)
endif()

if(mosek_FOUND)
  pybind11_add_module(_pybind_mosek pybind_mosek.cc)
  target_link_libraries(_pybind_mosek PRIVATE drakeOptimization)
  set_target_properties(_pybind_mosek PROPERTIES OUTPUT_NAME "mosek")
  install(TARGETS _pybind_mosek DESTINATION ${PYDRAKE_INSTALL_DIR}/solvers)
endif()
