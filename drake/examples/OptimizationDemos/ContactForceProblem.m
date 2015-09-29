classdef ContactForceProblem < Handle
  properties
    platforms = struct('point', {}, 'normal', {});
    F;
    constraints;
    objective;
    g = 9.81;
    m = 1;
  end

  methods
    function obj = ContactForceProblem(platforms)
      checkDependency('yalmip');
      yalmip('clear');
      obj.platforms = platforms;
      obj.F = sdpvar(3, numel(obj.platforms), 'full');
      obj.addForceBalance();
      obj.addMomentBalance();
    end

    function addForceBalance(obj)
      obj.constraints = [obj.constraints, sum(obj.F, 2) + [0; 0; -obj.m * obj.g] == 0];
    end
  end
end