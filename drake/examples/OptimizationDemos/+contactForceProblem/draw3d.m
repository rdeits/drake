function draw3d(platforms, F, x0)
  platform_width = 0.3;
  cone_scale = 0.5;
  force_scale = 0.2;

  [ax, az] = view();
  cla();
  hold on
  for j = 1:length(platforms)
    v1 = cross([1;1;0], platforms(j).normal);
    v1 = v1 / norm(v1);
    v2 = cross(platforms(j).normal, v1);
    v2 = v2 / norm(v2);
    p = platforms(j).point;
    xyz = bsxfun(@plus, p, [v1 + v2, v1 - v2, -v1 - v2, -v1 + v2] * platform_width / 2);
    fill3(xyz(1,:), xyz(2,:), xyz(3,:), 'k');

    n_cone_pts = 40;
    theta = linspace(0, 2*pi, n_cone_pts);
    cone_pts = zeros(3, n_cone_pts);
    for k = 1:n_cone_pts
      cone_pts(:,k) = platforms(j).point + cone_scale * (axis2rotmat([platforms(j).normal; theta(k)]) * v1 * platforms(j).mu + platforms(j).normal);
    end
    plot3(cone_pts(1,:), cone_pts(2,:), cone_pts(3,:), 'k--')
    for k = 1:10:40
      plot3([cone_pts(1,k), platforms(j).point(1)], [cone_pts(2,k), platforms(j).point(2)], [cone_pts(3,k), platforms(j).point(3)], 'k--');
    end
    % v = [0 -1; 1 0] * platforms(j).normal;
    % xy = [platforms(j).point + v * platform_width / 2, platforms(j).point - v * platform_width / 2];
    % plot(xy(1,:), xy(2,:), 'k-', 'LineWidth', 4);

    % cone_edges = [platforms(j).mu * v + platforms(j).normal,...
    %               -platforms(j).mu * v + platforms(j).normal];
    % for k = 1:2
    %   quiver(platforms(j).point(1), platforms(j).point(2), cone_edges(1,k), cone_edges(2,k), 0.5, 'color', 'k', 'LineStyle', '--', 'LineWidth', 2, 'ShowArrowHead', 'off', 'AutoScale', 'off');
    % end

    quiver3(platforms(j).point(1), platforms(j).point(2), platforms(j).point(3), F(1,j), F(2,j), F(3,j), force_scale, 'color', 'r', 'LineWidth', 3, 'MaxHeadSize', 0.5, 'AutoScale', 'off')

    plot3([platforms(j).point(1), x0(1)], [platforms(j).point(2), x0(2)], [platforms(j).point(3), x0(3)], 'b-')
  end

  plot3(x0(1), x0(2), x0(3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b')

  axis equal
  xlim([-1.5, 1.5])
  ylim([-1.5, 1.5])
  zlim([-1.25, 1.25])
  view(ax, az);
  drawnow()
end