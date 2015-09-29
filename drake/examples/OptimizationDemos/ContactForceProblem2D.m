classdef ContactForceProblem2D < ContactForceProblem
  methods
    function obj = ContactForceProblem2D(platforms)
      obj = obj@ContactForceProblem(platforms);
      obj.constraints = [obj.constraints, obj.F(2,:) == 0];
    end
  end
end