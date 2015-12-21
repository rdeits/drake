#ifndef _PENDULUM_H_
#define _PENDULUM_H_

#include <iostream>
#include <cmath>
#include "System.h"
#include "LQR.h"

using namespace std;

template <typename ScalarType = double>
class PendulumState  { // models the Drake::Vector concept
public:
  PendulumState(void) : theta(0), thetadot(0) {};
  template <typename Derived>
  PendulumState(const Eigen::MatrixBase<Derived>& x) : theta(x(0)), thetadot(x(1)) {};

  template <typename Derived>
  PendulumState& operator=(const Eigen::MatrixBase<Derived>& x) {
    theta = x(0);
    thetadot = x(1);
    return *this;
  }

  operator Eigen::Matrix<ScalarType,2,1> () const {
    Eigen::Matrix<ScalarType,2,1> x;
    x << theta, thetadot;
    return x;
  }

  friend std::ostream& operator<<(std::ostream& os, const PendulumState& x)
  {
    os << "  theta = " << x.theta << endl;
    os << "  thetadot = " << x.thetadot << endl;
    return os;
  }

  enum {
    RowsAtCompileTime = 2
  };
  std::size_t size() { return 2; }

  ScalarType theta;
  ScalarType thetadot;
};


template <typename ScalarType = double>
class PendulumInput {
public:
  PendulumInput(void) : tau(0) {};
  PendulumInput(const Eigen::Matrix<ScalarType,1,1>& x) : tau(x(0)) {};

  template <typename Derived>
  PendulumInput& operator=(const Eigen::MatrixBase<Derived>& x) {
    tau = x(0);
    return *this;
  }

  operator Eigen::Matrix<ScalarType,1,1> () const {
    Eigen::Matrix<ScalarType,1,1> x;
    x << tau;
    return x;
  }

  friend std::ostream& operator<<(std::ostream& os, const PendulumInput& x)
  {
    os << "  tau = " << x.tau << endl;
    return os;
  }

  enum {
    RowsAtCompileTime = 1
  };
  std::size_t size() { return 1; }

  ScalarType tau;
};

class Pendulum : public Drake::System<Pendulum,PendulumState,PendulumInput,PendulumState,false,false> {
public:
  Pendulum() :
          m(1.0), // kg
          l(.5),  // m
          b(0.1), // kg m^2 /s
          lc(.5), // m
          I(.25), // m*l^2; % kg*m^2
          g(9.81) // m/s^2
  {}
  virtual ~Pendulum(void) {};

  template <typename ScalarType>
  PendulumState<ScalarType> dynamicsImplementation(const PendulumState<ScalarType>& x, const PendulumInput<ScalarType>& u) const {
    PendulumState<ScalarType> dot;
    dot.theta = x.thetadot;
    dot.thetadot = (u.tau - m*g*lc*sin(x.theta) - b*x.thetadot)/I;
    return dot;
  }


  template <typename ScalarType>
  PendulumState<ScalarType> outputImplementation(const PendulumState<ScalarType>& x) const {
    return x;
  }

public:
  double m,l,b,lc,I,g;  // pendulum parameters (initialized in the constructor)
};


class PendulumEnergyShapingController : public Drake::System<PendulumEnergyShapingController,Drake::UnusedVector,PendulumState,PendulumInput,false,true> {
public:
  PendulumEnergyShapingController(const Pendulum& pendulum)
          : m(pendulum.m),
            l(pendulum.l),
            b(pendulum.b),
            g(pendulum.g)
  {};

  template <typename ScalarType>
  PendulumInput<ScalarType> outputImplementation(const PendulumState<ScalarType>& x) const {
    ScalarType Etilde = .5 * m*l*l*x.thetadot*x.thetadot - m*g*l*cos(x.theta) - 1.1*m*g*l;
    PendulumInput<ScalarType> u;
    u.tau = b*x.thetadot - .1*x.thetadot*Etilde;
    return u;
  }

  double m,l,b,g;  // pendulum parameters (initialized in the constructor)
};

#endif // _PENDULUM_H_
