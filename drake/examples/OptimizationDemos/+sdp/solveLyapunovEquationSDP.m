function solveLyapunovEquationSDP()
% find Mosek (SDP solver)
checkDependency('mosek');

% find Yalmip (modeling language that makes it easy to setup SDPs)
checkDependency('yalmip');

% create state matrix A
n = 4;
% A = createRandomStableStateMatrix(n);
A = createRandomUnStableStateMatrix(n);
disp('Eigenvalues of A:');
disp(eig(A));

% create P as a symmetric matrix decision variable
P = sdpvar(n);

% create cost matrix: g(x) = x' * Q * x
Q = eye(n);

% create constraints
constraints = [...
  P >= 0,...
  A * P + P * A' + Q == zeros(n)]

% optimize
diagnostics = optimize(constraints)

if diagnostics.problem == 0 % solution found
  % retrieve value of P
  P = value(P);
  disp('P');
  disp(P);
  
  % check that constraints are satisfied
  disp('Equality constraint check:');
  disp(A * P + P * A' + Q);
  disp('Eigenvalues of P:')
  disp(eig(P))
  
  % P_check = lyap(A, Q);
  % disp(P_check - P);
end
end

function A = createRandomStableStateMatrix(n)
A = randn(n);
A = A - (max(real(eig(A))) + 1) * eye(n);
end

function A = createRandomUnStableStateMatrix(n)
A = randn(n);
A = A + (1 - max(real(eig(A)))) * eye(n);
end
