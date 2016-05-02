function runGravityCompensation(obj, options)
% put robot in a random x,y,yaw position and balance for 2 seconds

if nargin < 2
  options = struct();
end

options = applyDefaults(options, struct('visualize', true));

checkDependency('gurobi')

% set initial state to fixed point
load(obj.fixed_point_file, 'xstar');
xstar(1) = 0.1*randn();
xstar(2) = 0.1*randn();
xstar(6) = pi*randn();
obj = obj.setInitialState(xstar);

% Construct plan
settings = QPLocomotionPlanSettings.fromStandingState(xstar, obj);
settings.supports = [RigidBodySupportState(obj, []), RigidBodySupportState(obj, [])];
settings.body_motions = [];
settings.gain_set = 'gravity_compensation';
settings.constrained_dofs = obj.findPositionIndices('leg');
standing_plan = QPLocomotionPlanCPPWrapper(settings);

% Construct our control blocks
planeval = bipedControllers.BipedPlanEval(obj, standing_plan);
control = bipedControllers.InstantaneousQPController(obj.getManipulator().urdf{1}, obj.control_config_file);
plancontroller = bipedControllers.BipedPlanEvalAndControlSystem(obj, control, planeval);

sys = feedback(obj, plancontroller);

if options.visualize
  v = obj.constructVisualizer;
  v.display_dt = 0.01;
  S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
  output_select(1).system=1;
  output_select(1).output=1;
  sys = mimoCascade(sys,v,[],[],output_select);
  warning(S);
end
x0 = xstar;
% x0(3) = 1.0; % drop it a bit

traj = simulate(sys,[0 2],x0, struct('gui_control_interface', true));
if options.visualize
  playback(v,traj,struct('slider',true));
end

xf = traj.eval(traj.tspan(2));

err = norm(xf(1:6)-xstar(1:6))
if err > 0.02
  error('drakeBalancing unit test failed: error is too large');
end

end
