function interactiveLP()
  f = figure(14);
  clf();
  ax = axes('Parent',f,'position',[0.13 0.39  0.77 0.54]);
  theta1 = 0;
  theta2 = 0;
  mu = 1;

  function setupAndSolve(theta1, theta2, mu)
    platforms = contactForceProblem.generatePlatforms2d(theta1, theta2, mu);
    x0 = [0; 0];
    F = contactForceProblem.runLP(platforms, x0);
    contactForceProblem.draw2d(platforms, F, x0);
  end

  theta1_control = uicontrol('Parent', f, 'Style', 'slider', 'Position', [81,54,419,23],...
                             'value', theta1, 'min', -pi, 'max', pi);
  function slider_callback (es, ed)
    theta1 = get(theta1_control, 'Value');
    mu = get(mu_control, 'Value');
    setupAndSolve(theta1, theta2, mu);
  end
  set(theta1_control, 'Callback', @slider_callback);
  addlistener(theta1_control, 'ContinuousValueChange', @slider_callback);
  mu_control = uicontrol('Parent', f, 'Style', 'slider', 'Position', [81,30,419,23],...
                             'value', mu, 'min', 0, 'max', 2);
  set(mu_control, 'Callback', @slider_callback);
  addlistener(mu_control, 'ContinuousValueChange', @slider_callback);

  setupAndSolve(theta1, theta2, mu)
end
