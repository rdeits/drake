function sol = testQuadrupedPlanner()
% Use the gaited mixed-integer footstep planner to solve a quadruped planning problem. 

prob = NonGaitedFootstepPlanningProblem();

% Pre-generate some safe regions in IRIS region format
safe_regions = ContactRegion.empty();

%%%%%%%% One big region
% V = [-.5, 1.3, 1.3, -.5; -.25, -.25, .25, .25];
% [A, b] = poly2lincon(V(1,:), V(2,:));
% safe_regions(end+1) = ContactRegion.fromTerrain(A, b, [.3;0;0], [0;0;1]);

%%%%%%%% Opposing walls
% V = [-.5, 1.3, 1.3, -.5; -.1, -.1, 0, 0];
% [A, b] = poly2lincon(V(1,:), V(2,:));
% n = [0;1;0.2];
% n = n / norm(n);
% safe_regions(end+1) = ContactRegion.fromTerrain(A, b, [0;-.05;0], n);

% V = [-.5, 1.3, 1.3, -.5; 0,0, .1, .1];
% [A, b] = poly2lincon(V(1,:), V(2,:));
% n = [0;-1;0.2];
% n = n / norm(n);
% safe_regions(end+1) = ContactRegion.fromTerrain(A, b, [0;.05;0], n);

%%%%%%% Wide opposing walls
% V = [-.5, 1.3, 1.3, -.5; -.2, -.2, -0.15, -0.15];
% [A, b] = poly2lincon(V(1,:), V(2,:));
% n = [0;1;0.2];
% n = n / norm(n);
% safe_regions(end+1) = ContactRegion.fromTerrain(A, b, [0;-.15;0], n);

% V = [-.5, 1.3, 1.3, -.5; 0.15,0.15, .2, .2];
% [A, b] = poly2lincon(V(1,:), V(2,:));
% n = [0;-1;0.2];
% n = n / norm(n);
% safe_regions(end+1) = ContactRegion.fromTerrain(A, b, [0;.15;0], n);

%%%%%%%% A small gap
V = [-.5, .4, .4, -.5; -.25, -.25, .25, .25];
[A, b] = poly2lincon(V(1,:), V(2,:));
safe_regions(end+1) = ContactRegion.fromTerrain(A,b, [.3;0;0], [0;0;1]);

V = [.6, 1.5, 1.5, .6; -.25, -.25, .25, .25];
[A, b] = poly2lincon(V(1,:), V(2,:));
safe_regions(end+1) = ContactRegion.fromTerrain(A,b, [.3;0;0], [0;0;1]);

%%%%%%%% Complex stepping stones
% V = [-0.15, 0.2 .2, -0.15; 0.2, 0.2, -0.2, -0.2];
% [A, b] = poly2lincon(V(1,:), V(2,:));
% safe_regions(end+1) = struct('A', A, 'b', b, 'point', [.3;0;1], 'normal', [0;0;1]);

% V = [0.25, 0.4 .4, 0.25; 0, 0, -0.3, -0.3];
% [A, b] = poly2lincon(V(1,:), V(2,:));
% safe_regions(end+1) = struct('A', A, 'b', b, 'point', [.3;0;1], 'normal', [0;0;1]);

% V = [.59, 1.5,1.5, .59; .5, .5, -.5, -.5];
% [A, b] = poly2lincon(V(1,:), V(2,:));
% safe_regions(end+1) = struct('A', A, 'b', b, 'point', [.8;0;1], 'normal', [0;0;1]);

% V = [.5,.51,.5,.51; -.05, -.05, -.06, -.06];
% [A, b] = poly2lincon(V(1,:), V(2,:));
% safe_regions(end+1) = struct('A', A, 'b', b, 'point', [.5;-.05;1], 'normal', [0;0;1]);

% V = [.5,.51,.51,.5; .05, .05, .06, .06];
% [A, b] = poly2lincon(V(1,:), V(2,:));
% safe_regions(end+1) = struct('A', A, 'b', b, 'point', [.5;.05;1], 'normal', [0;0;1]);

% V = [.45,.46,.46,.45; 0, 0, .01, .01];
% [A, b] = poly2lincon(V(1,:), V(2,:));
% safe_regions(end+1) = struct('A', A, 'b', b, 'point', [.5;.05;1], 'normal', [0;0;1]);

prob.safe_regions = safe_regions;
prob.feet = {'rf', 'lf', 'rh', 'lh'};
prob.foci = struct('rf', struct('v', {[0.1; -0.05]}, 'r', {0.05}),...
              'lf', struct('v', {[0.1; 0.05]}, 'r', {0.05}),...
              'rh', struct('v', {[-0.1; -0.05]}, 'r', {0.05}),...
              'lh', struct('v', {[-0.1; 0.05]}, 'r', {0.05}));

% Limits on delta Z and yaw from body to each foot
A = [0,0,1,0,0,0;
     0,0,-1,0,0,0;
     0,0,0,0,0,1;
     0,0,0,0,0,-1];
z_range = [-.2, -.1];
yaw_range = [-pi, pi];
b = [z_range(2);-z_range(1);yaw_range(2);-yaw_range(1)];
lcon_struct = struct('A', A, 'b', b);
prob.body_to_feet_constraints = struct('rf', lcon_struct,...
                                       'lf', lcon_struct,...
                                       'rh', lcon_struct,...
                                       'lh', lcon_struct);

prob.nframes = 13;
prob.swing_speed = .5;
prob.dt = 0.1;
prob.use_angular_momentum = true;
prob.max_angular_momentum = 10;
prob.foot_force = 1.1;

start_pose = struct('body', [0;0;0.15;0;0;0],...
                         'rf', [0.1;-0.05;0;0;0;0],...
                         'lf', [0.1;0.05;0;0;0;0],...
                         'rh', [-0.1;-0.05;0;0;0;0],...
                         'lh', [-0.1;0.05;0;0;0;0]);

% Non-periodic
goal_pose = struct('body', [.8;0;nan;nan;nan;0]);
prob.periodic = false;

% Periodic
% goal_pose = struct('body', [nan;0;nan;nan;nan;0]);
% prob.mean_velocity_bounds(1,:) = [1.1, nan];
% prob.periodic = true;

sol = prob.solveYalmip(start_pose, goal_pose);
% save('sol.mat', 'sol');
[xtraj, v] = sol.getSimpleGaitTrajectory();
v.playback_speed = 0.25;
v.playback(xtraj, struct('slider', true));
save('sol.mat', 'sol');

