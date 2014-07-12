function [plan, sin_yaw, cos_yaw] = footstepMISOCP_grb(biped, seed_plan, weights, goal_pos)

checkDependency('gurobi');
seed_plan.sanity_check();
rangecheck(seed_plan.footsteps(1).pos(6), -pi, pi);
rangecheck(seed_plan.footsteps(2).pos(6), -pi, pi);

nsteps = length(seed_plan.footsteps);
foci = [[0; 0.15], [0; -0.7]];
ellipse_l = 0.55;

seed_steps = [seed_plan.footsteps.pos];

min_yaw = pi * floor(seed_steps(6,1) / pi - 1);
max_yaw = pi * ceil(seed_steps(6,1) / pi + 1);
cos_boundaries = reshape(bsxfun(@plus, [min_yaw:pi:max_yaw; min_yaw:pi:max_yaw], [-(pi/2-1); (pi/2-1)]), 1, []);
sin_boundaries = reshape(bsxfun(@plus, [min_yaw:pi:max_yaw; min_yaw:pi:max_yaw], [-1; 1]), 1, []);


% Build a struct to hold the sizes and indices of our decision variables
nv = 0;
v = struct();

function add_var(name, type_, size_, lb, ub, start_)
  v.(name) = struct();
  v.(name).type = type_;
  v.(name).size = size_;
  v.(name).i = reshape(nv + (1:prod(v.(name).size)), v.(name).size);
  nv = nv + v.(name).i(end);
  if isscalar(lb)
    lb = repmat(lb, v.(name).size);
  end
  if isscalar(ub)
    ub = repmat(ub, v.(name).size);
  end
  v.(name).lb = lb;
  v.(name).ub = ub;
  if nargin < 6
    start_ = nan;
  end
  if isscalar(start_)
    start_ = repmat(start_, v.(name).size);
  end
  v.(name).start = start_;
end

x_lb = [-100 + repmat(seed_steps(1:3,1), 1, nsteps); 
          repmat(min_yaw, 1, nsteps)];
x_ub = [100 + repmat(seed_steps(1:3,1), 1, nsteps);
          repmat(max_yaw, 1, nsteps)];
add_var('x', 'C', [4, nsteps], x_lb, x_ub);
add_var('cos_yaw', 'C', [1, nsteps], -1, 1);
add_var('sin_yaw', 'C', [1, nsteps], -1, 1);
add_var('region', 'B', [length(seed_plan.safe_regions), nsteps], 0, 1);
add_var('cos_sector', 'B', [length(cos_boundaries)-1, nsteps], 0, 1);
add_var('sin_sector', 'B', [length(sin_boundaries)-1, nsteps], 0, 1);

A = zeros(0, nv);
b = zeros(0, 1);
Aeq = zeros(0, nv);
beq = zeros(0, 1);

% x(:,1) == seed_steps([1,2,3,6],1),...
ai = zeros(4, nv);
ai(:,v.x.i(:,1)) = eye(4);
bi = seed_steps([1,2,3,6], 1);
Aeq = [Aeq; ai]; beq = [beq; bi];

% x(:,2) == seed_steps([1,2,3,6],2),...
ai = zeros(4, nv);
ai(:,v.x.i(:,2)) = eye(4);
bi = seed_steps([1,2,3,6], 2);
Aeq = [Aeq; ai]; beq = [beq; bi];

% sum(region, 1) == 1,...
for j = 1:nsteps
  ai = zeros(1, nv);
  ai(v.region.i(:,j)) = 1;
  bi = 1;
  Aeq = [Aeq; ai]; beq = [beq; bi];
end

% sum(sin_sector, 1) == 1,...
for j = 1:nsteps
  ai = zeros(1, nv);
  ai(v.sin_sector.i(:,j)) = 1;
  bi = 1;
  Aeq = [Aeq; ai]; beq = [beq; bi];
end

% sum(cos_sector, 1) == 1,...
for j = 1:nsteps
  ai = zeros(1, nv);
  ai(v.cos_sector.i(:,j)) = 1;
  bi = 1;
  Aeq = [Aeq; ai]; beq = [beq; bi];
end

% Enforce approximation of cosine
for j = 1:nsteps
  for s = 1:length(cos_boundaries) - 1
    th0 = cos_boundaries(s);
    th1 = cos_boundaries(s+1);
    th = (th0 + th1)/2;
    cos_slope = -sin(th);
    cos_intercept = cos(th) - (cos_slope * th);

    % implies(cos_sector(s, j), th0 <= yaw(j) <= th1),...
    ai = zeros(2, nv);
    bi = zeros(2,1);
    M = 4 * pi;
    ai(:, v.cos_sector.i(s,j)) = M;
    ai(1, v.x.i(4,j)) = -1;
    bi(1) = -th0 + M;
    ai(2, v.x.i(4,j)) = 1;
    bi(2) = th1 + M;
    A = [A; ai];
    b = [b; bi];

    % implies(cos_sector(s, j), cos_yaw(j) == cos_slope * yaw(j) + cos_intercept)];
    ai = zeros(2, nv);
    bi = zeros(2, 1);
    ai(:, v.cos_sector.i(s,j)) = M;
    ai(1, v.cos_yaw.i(j)) = 1;
    ai(1, v.x.i(4,j)) = -cos_slope;
    bi(1) = cos_intercept + M;
    ai(2, v.cos_yaw.i(j)) = -1;
    ai(2, v.x.i(4,j)) = cos_slope;
    bi(2) = -cos_intercept + M;
    A = [A; ai];
    b = [b; bi];
  end
end

% Enforce approximation of sine
for j = 1:nsteps
  for s = 1:length(sin_boundaries) - 1
    th0 = sin_boundaries(s);
    th1 = sin_boundaries(s+1);

    th = (th0 + th1)/2;
    sin_slope = cos(th);
    sin_intercept = sin(th) - (sin_slope * th);

    % implies(sin_sector(s, j), th0 <= yaw(j) <= th1),...
    ai = zeros(2, nv);
    bi = zeros(2,1);
    M = 4 * pi;
    ai(:, v.sin_sector.i(s,j)) = M;
    ai(1, v.x.i(4,j)) = -1;
    bi(1) = -th0 + M;
    ai(2, v.x.i(4,j)) = 1;
    bi(2) = th1 + M;
    A = [A; ai];
    b = [b; bi];

    % implies(sin_sector(s, j), sin_yaw(j) == sin_slope * yaw(j) + sin_intercept)];
    ai = zeros(2, nv);
    bi = zeros(2, 1);
    ai(:, v.sin_sector.i(s,j)) = M;
    ai(1, v.sin_yaw.i(j)) = 1;
    ai(1, v.x.i(4,j)) = -sin_slope;
    bi(1) = sin_intercept + M;
    ai(2, v.sin_yaw.i(j)) = -1;
    ai(2, v.x.i(4,j)) = sin_slope;
    bi(2) = -sin_intercept + M;
    A = [A; ai];
    b = [b; bi];
  end
end

% Enforce range between sin/cos sectors
for j = 1:nsteps
  for k = 1:v.sin_sector.size(1)
    ai = zeros(2, nv);
    bi = zeros(2, 1);
    % sum(sin_sector(max(1,k-1):min(k+1,size(sin_sector,1)),j)) >= cos_sector(k,j),...
    ai(1, v.sin_sector.i(max(1,k-1):min(k+1,v.sin_sector.size(1)),j)) = -1;
    ai(1, v.cos_sector.i(k,j)) = 1;
    % sum(cos_sector(max(1,k-1):min(k+1,size(cos_sector,1)),j)) >= sin_sector(k,j)];
    ai(2, v.cos_sector.i(max(1,k-1):min(k+1,v.cos_sector.size(1)),j)) = -1;
    ai(2, v.sin_sector.i(k,j)) = 1;
    A = [A; ai];
    b = [b; bi];
  end
end

% Reachability between steps
for j = 2:nsteps-1
  % Ensure that the foot doesn't yaw too much per step
  if seed_plan.footsteps(j).frame_id == biped.foot_frame_id.right
    rel_foci = [foci(1,:); -foci(2,:)];
    yaw_range = [0, pi/8];
    for k = 1:v.cos_sector.size(1) - 1
      ai = zeros(2, nv);
      bi = zeros(2, 1);
      % sum(cos_sector(k:k+1,j)) >= cos_sector(k,j-1)
      ai(1, v.cos_sector.i(k,j-1)) = 1;
      ai(1, v.cos_sector.i(k:k+1,j)) = -1;
      % sum(sin_sector(k:k+1,j)) >= sin_sector(k,j-1)];
      ai(2, v.sin_sector.i(k,j-1)) = 1;
      ai(2, v.sin_sector.i(k:k+1,j)) = -1;
      A = [A; ai];
      b = [b; bi];
    end
  else
    rel_foci = foci;
    yaw_range = [-pi/8, 0];
    for k = 2:size(cos_sector, 1)
      ai = zeros(2, nv);
      bi = zeros(2, 1);
      % sum(cos_sector(k-1:k,j)) >= cos_sector(k,j-1),...
      ai(1, v.cos_sector.i(k,j-1)) = 1;
      ai(1, v.cos_sector.i(k-1:k,j)) = -1;
      % sum(sin_sector(k-1:k,j)) >= sin_sector(k,j-1)];
      ai(2, v.sin_sector.i(k,j-1)) = 1;
      ai(2, v.sin_sector.i(k-1:k,j)) = -1;
      A = [A; ai];
      b = [b; bi];
    end
  end

  % yaw_range(1) <= yaw(j+1) - yaw(j) <= yaw_range(2)];
  ai = zeros(2, nv);
  bi = zeros(2, 1);
  ai(1, v.x.i(4,j)) = 1;
  ai(1, v.x.i(4,j+1)) = -1;
  bi(1) = -yaw_range(1);
  ai(2, v.x.i(4,j)) = -1;
  ai(2, v.x.i(4,j+1)) = 1;
  bi(2) = yaw_range(2);
  A = [A; ai];
  b = [b; bi];

%%%%%%% TODO: from here down
  for k = 1:size(rel_foci, 2)
    Constraints = [Constraints, ...
      cone(x(1:2,j) + [cos_yaw(j), -sin_yaw(j); sin_yaw(j), cos_yaw(j)] * rel_foci(:,k) - x(1:2,j+1), ellipse_l),...
      abs(x(3,j+1) - x(3,j)) <= seed_plan.params.nom_upward_step];
  end
end


var_names = fieldnames(v);
clear model params
model.A = sparse([A; Aeq]);
model.rhs = [b; beq];
model.sense = [repmat('<', size(A,1), 1); repmat('=', size(Aeq, 1), 1)];
model.start = nan(nv, 1);
model.obj = zeros(nv, 1);

% Set up defaults so we can fill them in from v
model.vtype = repmat('C', nv, 1);
model.lb = -inf(nv, 1);
model.ub = inf(nv, 1);
for j = 1:length(var_names)
  name = var_names{j};
  i = reshape(v.(name).i, [], 1);
  model.vtype(i) = v.(name).type;
  model.lb(i) = reshape(v.(name).lb, [], 1);
  model.ub(i) = reshape(v.(name).ub, [], 1);
  model.start(i) = reshape(v.(name).start, [], 1);
end

params.mipgap = 1e-3;
params.outputflag = 1;
result = gurobi(model, params);
for j = 1:length(var_names)
  name = var_names{j};
  i = reshape(v.(name).i, [], 1);
  if v.(name).type == 'I' 
    v.(name).value = reshape(round(result.x(i)), v.(name).size);
  elseif v.(name).type == 'B'
    v.(name).value = reshape(logical(round(result.x(i))), v.(name).size);
  else
    v.(name).value = reshape(result.x(i), v.(name).size);
  end
end

for j = 1:nsteps
  for s = 1:length(cos_boundaries) - 1
    th0 = cos_boundaries(s);
    th1 = cos_boundaries(s+1);

    th = (th0 + th1)/2;
    cos_slope = -sin(th);
    cos_intercept = cos(th) - (cos_slope * th);
    if v.cos_sector.value(s,j)
      assert(v.x.value(4,j) >= th0);
      assert(v.x.value(4,j) <= th1);
      assert(abs(v.cos_yaw.value(j) - (cos_slope * v.x.value(4,j) + cos_intercept)) < 1e-3);
    end
  end
end
for j = 1:nsteps
  for s = 1:length(sin_boundaries) - 1
    th0 = sin_boundaries(s);
    th1 = sin_boundaries(s+1);

    th = (th0 + th1)/2;
    sin_slope = cos(th);
    sin_intercept = sin(th) - (sin_slope * th);
    if v.sin_sector.value(s,j)
      assert(v.x.value(4,j) >= th0);
      assert(v.x.value(4,j) <= th1);
      assert(abs(v.sin_yaw.value(j) - (sin_slope * v.x.value(4,j) + sin_intercept)) < 1e-3);
    end
  end
end

steps = v.x.value;
steps = [steps(1:3,:); zeros(2, size(steps, 2)); steps(4,:)];

plan = seed_plan;
for j = 1:nsteps
  plan.footsteps(j).pos = steps(:,j);
end

[region_order, ~] = find(v.region.value);
assert(length(region_order) == size(v.region.value, 2));
plan.region_order = region_order;

end