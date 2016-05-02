function settings = gravityCompensationPlan(biped, gravity_compensated_joints, nominal_posture)

settings = QPLocomotionPlanSettings(biped);
settings.constrained_dofs = [];
settings.support_times = [0, inf];
settings.supports = RigidBodySupportstate(biped, []);
settings.qtraj = nominal_posture;

