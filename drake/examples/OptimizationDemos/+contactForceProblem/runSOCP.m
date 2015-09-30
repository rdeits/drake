function F = runSOCP(platforms, x0)
checkDependency('gurobi');
checkDependency('yalmip');

m = 1;
g = 9.81;

F = sdpvar(3, numel(platforms), 'full');

constraints = [...
  sum(F, 2) == [0; 0; m*g],... % Force balance
  ];

total_torque = 0;
for j = 1:numel(platforms)
  total_torque = total_torque + cross(platforms(j).point - x0, F(:,j));
end
constraints = [constraints,...
  total_torque == 0,... % Moment balance
  ];

for j = 1:length(platforms)
  n = platforms(j).normal;
  mu = platforms(j).mu;
  constraints = [constraints,...
    % n' * f / (norm(f)) = cos(theta)
    % where theta = atan(mu)
    % so we constrain that n' * f >= norm(f) * cos(atan(mu))
    % norm(f) <= (n' * f) / cos(atan(mu))
    cone(F(:,j), n' * F(:,j) / cos(atan(mu))),...
    ];
end

objective = sum(sum(F .^ 2));

result = optimize(constraints, objective)

F = value(F)

end
