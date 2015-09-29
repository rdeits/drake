function platforms = generatePlatforms2d(theta1, theta2, mu)
  platforms = struct('point', {[-1;-1], [1; -0.75]},...
                     'normal', {rotmat(theta1) * [0; 1], rotmat(theta2) * [0; 1]},...
                     'mu', {mu, mu});
end

function R = rotmat(theta)
  R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
end