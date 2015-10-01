function draw2d(platforms, F, x0)
  platform_width = 0.3;

  cla();
  hold on
  for j = 1:length(platforms)
    v = [0 -1; 1 0] * platforms(j).normal;
    xy = [platforms(j).point + v * platform_width / 2, platforms(j).point - v * platform_width / 2];
    plot(xy(1,:), xy(2,:), 'k-', 'LineWidth', 4);

    cone_edges = [platforms(j).mu * v + platforms(j).normal,...
                  -platforms(j).mu * v + platforms(j).normal];
    for k = 1:2
      quiver(platforms(j).point(1), platforms(j).point(2), cone_edges(1,k), cone_edges(2,k), 0.5, 'color', 'k', 'LineStyle', '--', 'LineWidth', 2, 'ShowArrowHead', 'off', 'AutoScale', 'off');
    end

    quiver(platforms(j).point(1), platforms(j).point(2), F(1,j), F(2,j), 0.1, 'color', 'r', 'LineWidth', 3, 'MaxHeadSize', 0.5, 'AutoScale', 'off')

    plot([platforms(j).point(1), x0(1)], [platforms(j).point(2), x0(2)], 'b-')
  end

  plot(x0(1), x0(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b')

  axis equal
  xlim([-1.5, 1.5])
  ylim([-1.25, 1.25])
  drawnow()
end