function platforms = generatePlatforms3d(theta1, theta2, mu)
  T1 = makehgtform('xrotate', theta1);
  T2 = makehgtform('yrotate', theta2);
  platforms = struct('point', {[-1;-1;-1], [1; 0.5; -0.75], [0; 0.75; -0.5]},...
                     'normal', {T1(1:3,1:3) * [0; 0; 1], T2(1:3,1:3) * [0; 0; 1], [0; 0;, 1]},...
                     'mu', {mu, mu, mu});
end

