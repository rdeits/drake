function [plan, solvertime] = humanoids2014(biped, seed_plan, weights, goal_pos, use_symbolic)
% Footstep planner based on the approach presented in "Footstep Planning on
% Uneven Terrain with Mixed-Integer Convex Optimization" by Robin Deits and
% Russ Tedrake. This implementation uses a mixed-integer
% quadratically-constrained program to plan the number of footsteps to take,
% the position and yaw of those steps, and the assignments of footsteps to
% convex regions of obstacle-free terrain. 
%
% This planner should be used by passing the 'method_handle', @footstepPlanner.humanoids2014 
% option to planFootsteps.m.
% 
% @param seed_plan a blank footstep plan, provinding the structure of the
%                  desired plan. Probably generated with
%                  FootstepPlan.blank_plan()
% @param weights a struct with fields 'goal', 'relative', and
%                'relative_final' describing the various contributions to
%                the cost function. These are described in detail in
%                Biped.getFootstepOptimizationWeights()
% @param goal_pos a struct with fields 'right' and 'left'.
%                 goal_pos.right is the desired 6 DOF pose
%                 of the right foot sole, and likewise for
%                 goal_pos.left
% @option use_symbolic (default: false) whether to use the symbolic yalmip
%                     version of the solver, which is slower to set up
%                     but easier to modify.
% @retval plan a FootstepPlan with the results of the optimization


if nargin < 5
  use_symbolic = 1;
end

nsteps = length(seed_plan.footsteps);

if use_symbolic
  t0 = tic();
  p1 = MixedIntegerFootstepPlanningProblem(biped, seed_plan, true);
  p1.weights = weights;
  p1 = p1.addSinCosLinearEquality(true);
  p1 = p1.addQuadraticRelativeObjective(true);
  p1 = p1.addZAndYawReachability(true);
  p1 = p1.addTrimToFinalPoses(true);
  p1 = p1.addQuadraticGoalObjective(goal_pos, nsteps-1:nsteps, [1,1], true);
  p1 = p1.addTerrainRegionsBinaryString([], true);
  p1 = p1.addXYReachabilityCircles(true);
  if use_symbolic == 2
    fprintf(1, 'yalmip setup: %f\n', toc(t0));
  end
  [p1, solvertime, objval_symb] = p1.solve();
  if use_symbolic == 2
    fprintf(1, 'yalmip total: %f\n', toc(t0));
  end
  plan = p1.getFootstepPlan();
end

if (use_symbolic == 0 || use_symbolic == 2)
  error('non-symbolic version not implemented');
end
