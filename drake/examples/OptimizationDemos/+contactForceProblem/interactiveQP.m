function interactiveQP()
  f = figure(15);
  clf();
  ax = axes('Parent',f,'position',[0.13 0.39  0.77 0.54]);
  theta1 = 0;
  theta2 = 0;
  mu = 1;

  function setupAndSolve(theta1, theta2, mu)
    platforms = contactForceProblem.generatePlatforms2d(theta1, theta2, mu);
    x0 = [0; 0];
    F = contactForceProblem.runQP(platforms, x0);
    contactForceProblem.draw2d(platforms, F, x0);
  end

  function slider_callback (es, ed)
    theta1 = get(theta1_control, 'Value');
    theta2 = get(theta2_control, 'Value');
    mu = get(mu_control, 'Value');
    setupAndSolve(theta1, theta2, mu);
  end
  theta1_control = uicontrol('Parent', f, 'Style', 'slider', 'Position', [81,90,419,23],...
                             'value', theta1, 'min', -pi, 'max', pi);
  uicontrol('Parent', f, 'Style', 'text', 'String', 'theta1', 'Position', [505, 90, 60, 23]);
  addlistener(theta1_control, 'ContinuousValueChange', @slider_callback);
  theta2_control = uicontrol('Parent', f, 'Style', 'slider', 'Position', [81,60,419,23],...
                             'value', theta1, 'min', -pi, 'max', pi);
  uicontrol('Parent', f, 'Style', 'text', 'String', 'theta2', 'Position', [505, 60, 60, 23]);
  addlistener(theta2_control, 'ContinuousValueChange', @slider_callback);
  mu_control = uicontrol('Parent', f, 'Style', 'slider', 'Position', [81,30,419,23],...
                             'value', mu, 'min', 0, 'max', 2);
  uicontrol('Parent', f, 'Style', 'text', 'String', 'mu', 'Position', [505, 30, 60, 23]);
  addlistener(mu_control, 'ContinuousValueChange', @slider_callback);

  setupAndSolve(theta1, theta2, mu)
end
