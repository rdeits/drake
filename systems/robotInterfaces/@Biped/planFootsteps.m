function plan = planFootsteps(obj, start_pos_or_q, goal_pos, safe_regions, options)
% planFootsteps: find a set of reachable foot positions from the start to
% the goal.
% @param start_pos_or_q a struct with fields 'right' and 'left' OR a configuration vector
%                  start_pos.right is the 6 DOF initial pose
%                  of the right foot sole and start_pos.left
%                  is the 6 DOF pose of the left sole.
% @param goal_pos a struct with fields 'right' and 'left'.
%                 goal_pos.right is the desired 6 DOF pose
%                 of the right foot, and likewise for
%                 goal_pos.left
% @params safe_regions a list of planar polytopic regions into which the footstep
%                      locations must be placed. Can be empty. If
%                      safe_regions == [], then the footstep locations are
%                      not constrained in this way. Otherwise, each
%                      footstpe must fall within at least one of the
%                      defined safe_regions. safe_regions should be a list
%                      of objects or structures which have fields 'A',
%                      'b', 'point', and 'normal' which define a region in
%                      v = [x,y,z,yaw] as
%                      A*v <= b AND normal'*v(1:3) == normal'*point
% @param options a struct of options
%
% @option method_handle (default: @footstepAlternatingMIQP) the footstep planning
%                method to use, expressed as a function handle
% @option step_params (default: struct()) specific parameters for footstep
%                     planning. Attributes set here overload those in
%                     obj.default_footstep_params

if nargin < 5; options = struct(); end
if nargin < 4; safe_regions = []; end
if ~isfield(options, 'method_handle'); options.method_handle = @footstepAlternatingMIQP; end
if ~isfield(options, 'step_params'); options.step_params = struct(); end
options.step_params = obj.applyDefaultFootstepParams(options.step_params);

if isnumeric(start_pos_or_q)
  start_pos = obj.feetPosition(start_pos_or_q);
else
  typecheck(start_pos_or_q, 'struct');
  start_pos = start_pos_or_q;
end
sizecheck(start_pos.right, [6,1]);
sizecheck(start_pos.left, [6,1]);
sizecheck(goal_pos.right, [6,1]);
sizecheck(goal_pos.left, [6,1]);
start_pos.right(4:5) = 0;
start_pos.left(4:5) = 0;
goal_pos.right(4:5) = 0;
goal_pos.left(4:5) = 0;

if isempty(safe_regions)
  n = rpy2rotmat(start_pos.right(4:6)) * [0;0;1];
  pt = start_pos.right(1:3);
  safe_regions = [struct('A', zeros(0,3), 'b', zeros(0,1), 'point', pt, 'normal', n)];
end
for j = 1:length(safe_regions)
  sizecheck(safe_regions(j).A, [NaN, 3]);
  sizecheck(safe_regions(j).b, [size(safe_regions(j).A, 1), 1]);
  sizecheck(safe_regions(j).point, [3,1]);
  sizecheck(safe_regions(j).normal, [3,1]);
end

% Outer loop to determine required number of footsteps
min_steps_to_cover_distance = ceil(max([norm(goal_pos.right(1:2) - start_pos.right(1:2)) / options.step_params.max_forward_step + 1,...
                                   norm(goal_pos.right(1:2) - start_pos.right(1:2)) / options.step_params.max_forward_step + 1,...
                                   abs(goal_pos.right(6) - start_pos.right(6)) / mean([options.step_params.max_outward_angle, options.step_params.max_inward_angle]) + 1,...
                                   abs(goal_pos.right(3) - start_pos.right(3)) / mean([options.step_params.nom_upward_step, options.step_params.nom_downward_step]) + 1]));
nsteps = min(max(min_steps_to_cover_distance, options.step_params.min_num_steps), options.step_params.max_num_steps);

% Currently we always lead with the right foot, but this should change soon
plan = FootstepPlan.blank_plan(obj, nsteps + 2, [obj.foot_frame_id.right, obj.foot_frame_id.left], options.step_params, safe_regions);
plan.footsteps(1).pos = start_pos.right;
plan.footsteps(2).pos = start_pos.left;

step_excess = [];
goal_dist = [];
weights = getFootstepOptimizationWeights(obj);
while true
  plan = options.method_handle(obj, plan, weights, goal_pos);
  if plan.footsteps(end).frame_id == obj.foot_frame_id.right
    final_error = abs(goal_pos.right - plan.footsteps(end).pos) + abs(goal_pos.left - plan.footsteps(end-1).pos);
  else
    final_error = abs(goal_pos.left - plan.footsteps(end).pos) + abs(goal_pos.right - plan.footsteps(end-1).pos);
  end
  
  lc = lcm.lcm.LCM.getSingleton();
  lc.publish('FOOTSTEP_PLAN_RESPONSE', plan.toLCM());
  nsteps
  steps_rel = plan.relative_step_offsets()
  goal_dist(end+1) = sum(max(final_error - [0.02; 0.02; 0.02; inf; inf; pi/64],0));
  step_excess(end+1) = sum(steps_rel(1,steps_rel(1,:) > options.step_params.nom_forward_step));
  if (goal_dist(end) <= 0 && step_excess(end) <= 0) ...
      || nsteps == options.step_params.max_num_steps
    break
  elseif length(goal_dist) > 1 ...
      && (goal_dist(end) <= 0 && step_excess(end) >= 0.99 * step_excess(end-1))
    break
  else
    nsteps = nsteps + 1;
    plan = plan.extend(nsteps+2);
  end
end


end

