function runPassiveWRL

% NOTEST (vrml doesn't work on build machine yet)

p = PlanarRigidBodyManipulator(fullfile(getDrakePath, '+examples', '+Acrobot', 'Acrobot.urdf'));
v = p.constructWRLVisualizer;

x = p.simulate([0 5],randn(4,1));
v.playback(x);