function F = runLP(platforms, x0)

m = 1;
g = 9.81;

F_idx = reshape(1:2 * numel(platforms), 2, []);

n_var = numel(F_idx);
n_eq = 3;
n_ineq = 2 * numel(platforms); % friction cones

A = zeros(n_ineq, n_var);
b = zeros(n_ineq, 1);
Aeq = zeros(n_eq, n_var);
beq = zeros(n_eq, 1);

% Force balance
  % sum(F, 2) + [0; -m * g] == 0
for j = 1:2
  Aeq(j,F_idx(j,:)) = 1;
end
beq(1:2) = [0; m * g];

% Moment balance
for j = 1:length(platforms)
  Aeq(3, F_idx(1,j)) = -platforms(j).point(2) + x0(2);
  Aeq(3, F_idx(2,j)) = platforms(j).point(1) - x0(1);
end

% Friction cones
for j = 1:length(platforms)
  n = platforms(j).normal;
  mu = platforms(j).mu;
  A(2*j-1, F_idx(:,j)) = n' * [0 -1; 1 0]' - n' * mu;
  A(2*j, F_idx(:,j)) = n' * [0 1; -1 0]' - n' * mu;
end

c = ones(n_var, 1);

[F, optval, exitflag] = linprog(c, A, b, Aeq, beq)
F = F(F_idx);
if exitflag < 0
  F = nan(size(F));
end

% F = sdpvar(2, numel(platforms), 'full');

% constraints = [...
% ];

% total_torque = 0;
% for j = 1:length(platforms)
%   total_torque = total_torque + cross([platforms(j).point - x0; 0], [F(:,j); 0]);
% end
% constraints = [constraints,...
%   total_torque == 0,... % Moment balance
%   ];

% for j = 1:length(platforms)
%   n = platforms(j).normal;
%   mu = platforms(j).mu;
%   constraints = [constraints,...
%     n' * F(:,j) >= 0,...
%     mu * n' * F(:,j) >= n' * [0 -1; 1 0]' * F(:,j),... % 2D friction cone
%     mu * n' * F(:,j) >= n' * [0 1; -1 0]' * F(:,j),... % 2D friction cone
%     ];
% end

% objective = sum(F(:));
% result = optimize(constraints, objective)

% F = value(F);

end

