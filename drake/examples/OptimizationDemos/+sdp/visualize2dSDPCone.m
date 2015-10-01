function visualize2dSDPCone()
% create grid of points
[x1, x2, x3] = meshgrid(...
  linspace(0, 1, 100),...
  linspace(-1, 1, 100),...
  linspace(0, 1, 100));

% constraint: x * z - y^2>= 0
constraint_left_hand_side = x1 .* x3 - x2.^2;

% plot surface where constraint_left_hand_side is zero
p = patch(isosurface(x1, x2, x3, constraint_left_hand_side, 0));

% graph formatting
isonormals(x1,x2,x3,constraint_left_hand_side, p)
p.FaceColor = 'red';
p.EdgeColor = 'none';
daspect([1 1 1])
view(3)
camlight; lighting phong
xlabel('x_1'); ylabel('x_2'); zlabel('x_3');
grid on;
box on;
set(gca(), 'BoxStyle', 'full');

end