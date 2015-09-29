function F = runQP(platforms, x0)

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


% Minimize sum(F_{i,j} ^2)
Q = diag(ones(n_var, 1));
q = zeros(n_var, 1);

[F, optval, exitflag] = quadprog(Q, q, A, b, Aeq, beq);
F = F(F_idx);
if exitflag < 0
  F = nan(size(F));
end