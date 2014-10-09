classdef NonGaitedFootstepPlanningProblem
  properties
    feet = {'left', 'right'};
    foci = struct('right', struct('v', {[0; 0], [0; -0.25]},...
                                             'r', {0.19, 0.16}),...
                                 'left', struct('v', {[0; 0], [0; 0.25]},...
                                             'r', {0.19, 0.16}));
    body_to_feet_constraints = struct('right', struct('A', [0,0,1,0,0,0;
                                                      0,0,-1,0,0,0;
                                                      0,0,0,0,0,1;
                                                      0,0,0,0,0,-1],...
                                                'b', [0.1;
                                                      0.1;
                                                      0.01;
                                                      pi/8]),...
                                     'left', struct('A', [0,0,1,0,0,0;
                                                      0,0,-1,0,0,0;
                                                      0,0,0,0,0,1;
                                                      0,0,0,0,0,-1],...
                                                'b', [0.1;
                                                      0.1;
                                                      pi/8;
                                                      0.01])); % currently must be linear
    nframes = 10; % number of frames of the gait to plan
    swing_speed = 1; % m/s
    body_speed = 0.25; % m/s
    yaw_speed = 1; % rad/s
    safe_regions = ContactRegion();
    dt = 0.25;
    g = 9.81; % accel due to gravity (m/s^2)
    foot_force = 0.6; % body weights
    nominal_com_height = 0.2; % m

    % Leave empty to solve for the gait, or set as a struct with one field for each foot name.
    % If non-empty, then each element gait.(foot)(j) is true iff foot (foot) is in contact with the ground
    % at frame j. 
    % The provided gait can be shorter than obj.nframes. In that case, the gait will be repeated as necessary.
    gait = [];

    % Leave empty to solve for the safe region assignments, or set as struct with one field for each foot name
    % If non-empty, then each element safe_region_assignments.(foot)(j) gives the index of the safe region to 
    % which the position of foot (foot) at frame j is assigned.
    safe_region_assignments = [];

    use_angular_momentum = false;
    max_angular_momentum = 3; 

    periodic = false; % Require velocity, acceleration, and gait to match from the last frame to the first

    mean_velocity_bounds = nan(3,2); % optional bounds on body velocity [x_lower, x_upper; y_lower, y_upper; z_lower, z_upper];
  end

  methods
    function ok = checkProblem(obj)
      ok = true;
      for f = obj.feet
        foot = f{1};
        if ~isfield(obj.foci, foot)
          ok = false;
          break
        end
        if ~isfield(obj.body_to_feet_constraints, foot)
          ok = false;
          break
        end
        if ~isempty(obj.gait)
          if ~isfield(obj.gait, foot)
            ok = false; 
            break
          end
          sizecheck(obj.gait.(foot), [1, nan]);
        end

        if ~isempty(obj.safe_region_assignments)
          if ~isfield(obj.safe_region_assignments, foot)
            ok = false;
            break
          end
          sizecheck(obj.safe_region_assignments.(foot), [1, obj.nframes]);
        end
      end
      if nargout == 0 && ~ok
        error('footstep planning problem self-check failed. Make sure that obj.foci, obj.body_to_feet_constraints, and obj.gait have fields corresponding to each foot');
      end
    end

    function sol = solveYalmip(obj, start_pose, goal_pose)

      if nargin < 3; goal_pose = []; end
      if isempty(goal_pose) && ~obj.periodic
        error('You must either set the "periodic" property to true or provide a goal pose');
      end

      checkDependency('yalmip');
      checkDependency('gurobi');

      obj.checkProblem();

      % A general upper limit on the distance covered in a single plan (in meters);
      MAX_DISTANCE = 30;
      MAX_VELOCITY = MAX_DISTANCE / obj.dt;
      MAX_ACCELERATION = 10 * obj.g;

      degree = 2;

      % Which indices of [x, y, z, roll, pitch, yaw] do we actually use
      POSE_INDICES = [1,2,3,6];


      nregions = length(obj.safe_regions);
      if nregions == 0
        error('safe_regions cannot be empty. Try setting obj.safe_regions = ContactRegion()');
      end

      pose = struct();
      pose.body = sdpvar(4, obj.nframes, 'full');
      velocity.body = sdpvar(3, obj.nframes, 'full');
      acceleration.body = sdpvar(3, obj.nframes, 'full');

      region = struct();
      gait = struct();
      nominal_pose = struct();

      if obj.use_angular_momentum
        angular_momentum = struct();
        angular_momentum.body = sdpvar(3, obj.nframes, 'full');
      end

      contact_force = struct('total', 0);
      for j = 1:length(obj.feet)
        foot = obj.feet{j};
        pose.(foot) = sdpvar(4, obj.nframes, 'full');

        if isempty(obj.gait)
          gait.(foot) = binvar(1, obj.nframes, 'full');
        else
          gait.(foot) = repmat(obj.gait.(foot), 1, ceil(obj.nframes / length(obj.gait.(foot))));
          gait.(foot) = gait.(foot)(1:obj.nframes);
        end
        contact_weighting.(foot) = sdpvar(4, obj.nframes, 'full');
        contact_force.(foot) = sdpvar(3, obj.nframes, 'full');
        contact_force.total = contact_force.total + contact_force.(foot);
        nominal_pose.(foot) = [mean([obj.foci.(foot).v], 2); -0.2];  % TODO: this is hard-coded for LittleDog
        if isempty(obj.safe_region_assignments)
          region.(foot) = binvar(nregions, obj.nframes, 'full');
        else
          region.(foot) = false(nregions, obj.nframes);
          for j = 1:obj.nframes
            region.(foot)(obj.safe_region_assignments.(foot)(j), j) = true;
          end
        end
      end

      body_yaw = pose.body(4,:);
      cos_yaw = sdpvar(1, obj.nframes, 'full');
      sin_yaw = sdpvar(1, obj.nframes, 'full');

      angle_boundaries = (-pi-pi/8):(pi/4):(pi-pi/8);
      yaw_sector = binvar(length(angle_boundaries) - 1, obj.nframes, 'full');

      min_yaw = start_pose.body(6) - pi;
      max_yaw = start_pose.body(6) + pi;

      % Set up the basic constraints, including some general ranges on body pose (to make the mixed-integer formulation easier for yalmip)
      constraints = [...
        pose.body(:,1) == start_pose.body(POSE_INDICES),...
        min_yaw <= body_yaw <= max_yaw,...
        start_pose.body(1) - MAX_DISTANCE <= pose.body(1,:) <= start_pose.body(1) + MAX_DISTANCE,...
        start_pose.body(2) - MAX_DISTANCE <= pose.body(2,:) <= start_pose.body(2) + MAX_DISTANCE,...
        start_pose.body(3) - MAX_DISTANCE <= pose.body(3,:) <= start_pose.body(3) + MAX_DISTANCE,...
        -MAX_VELOCITY <= velocity.body <= MAX_VELOCITY,...
        -MAX_ACCELERATION <= acceleration.body <= MAX_ACCELERATION,...
        sum(yaw_sector, 1) == 1,...
        -1 <= sin_yaw <= 1,...
        -1 <= cos_yaw <= 1,...
        yaw_sector(5,:) == 1,... % Disable yaw
        ];

      if obj.periodic
        % Enforce periodicity
        constraints = [constraints, ...
          velocity.body(:,end) == velocity.body(:,1),...
          acceleration.body(:,end) == acceleration.body(:,1),...
          pose.body(3,end) == pose.body(3,1),...
          ];
        for f = obj.feet
          foot = f{1};
          constraints = [constraints, gait.(foot)(1) == gait.(foot)(end)];
        end
      else
        % Require the final velocity to be zero (to avoid the solution where
        % we just plummet through the ground forever). 
        constraints = [constraints, velocity.body(3,end) == 0,...
          velocity.body(:,1) == 0,...
          ];

      end

      % Constrain the initial conditions
      start_fields = fieldnames(start_pose)';
      for f = start_fields
        field = f{1};
        for k = 1:length(POSE_INDICES)
          if ~isnan(start_pose.(field)(POSE_INDICES(k)))
            constraints = [constraints, pose.(field)(k,1) == start_pose.(field)(POSE_INDICES(k))];
          end
        end
      end

      for j = 1:3
        if ~isnan(obj.mean_velocity_bounds(j,1))
          constraints = [constraints, mean(velocity.body(j,:)) >= obj.mean_velocity_bounds(j,1)];
        end
        if ~isnan(obj.mean_velocity_bounds(j,2))
          constraints = [constraints, mean(velocity.body(j,:)) <= obj.mean_velocity_bounds(j,2)];
        end
      end

      for f = obj.feet
        foot = f{1};
        if obj.periodic
          constraints = [constraints,...
            gait.(foot)(1) == gait.(foot)(end),...
            pose.(foot)(:,1) - pose.body(:,1) == pose.(foot)(:,end) - pose.body(:,end)];
        else
          % Feet on the ground at the start and end
          constraints = [constraints, gait.(foot)([1,end]) == 1];
        end

        % complementarity conditions
        constraints = [constraints,...
          -obj.foot_force * gait.(foot) <= contact_force.(foot)(1,:) <= obj.foot_force * gait.(foot),...
          -obj.foot_force * gait.(foot) <= contact_force.(foot)(2,:) <= obj.foot_force * gait.(foot),...
          -obj.foot_force * gait.(foot) <= contact_force.(foot)(3,:) <= obj.foot_force * gait.(foot),...
          ];

        % Loose bounds on foot poses
        constraints = [constraints,...
          start_pose.body(1) - MAX_DISTANCE <= pose.(foot)(1,:) <= start_pose.body(1) + MAX_DISTANCE,...
          start_pose.body(2) - MAX_DISTANCE <= pose.(foot)(2,:) <= start_pose.body(2) + MAX_DISTANCE,...
          start_pose.body(3) - MAX_DISTANCE <= pose.(foot)(3,:) <= start_pose.body(3) + MAX_DISTANCE,...
          min_yaw <= pose.(foot)(4,:) <= max_yaw,...
          ];

        % Make sure we assign each foot to a region
        if isempty(obj.safe_region_assignments)
          constraints = [constraints, sum(region.(foot), 1) >= gait.(foot)];
        end

        % Stay in convex hull of contact cone
        constraints = [constraints, 0 <= contact_weighting.(foot) <= 1, ...
                       sum(contact_weighting.(foot), 1) <= 1,...
                       -obj.foot_force <= contact_force.(foot) <= obj.foot_force];
      end

      if obj.use_angular_momentum
        if obj.periodic
          constraints = [constraints,...
            angular_momentum.body(:,1) == angular_momentum.body(:,end)];
        else
          constraints = [constraints,...
            angular_momentum.body(:,1) == 0,...
            angular_momentum.body(:,end) == 0,...
            ];
        end
        constraints = [constraints, ...
          abs(angular_momentum.body) <= obj.max_angular_momentum,...
          ];
      end

      for j = 1:obj.nframes
        if j < obj.nframes
          % Enforce ballistic dynamics
          constraints = [constraints, ...
                         pose.body(1:3,j+1) == pose.body(1:3,j) + velocity.body(1:3,j) * obj.dt + acceleration.body(1:3,j) * 0.5 * obj.dt^2,...
                         velocity.body(1:3,j+1) == velocity.body(1:3,j) + acceleration.body(1:3,j) * obj.dt,...
                         acceleration.body(1:3,j) == contact_force.total(1:3,j)*obj.g + [0;0;-obj.g],...
                         ];

          if obj.use_angular_momentum
            new_momentum = angular_momentum.body(:,j);

            for f = obj.feet
              foot = f{1};
              % new_momentum = new_momentum + cross(nominal_pose.(foot), contact_force.(foot)(:,j));
              new_momentum = new_momentum + cross(nominal_pose.(foot), [0;0;1]) * contact_force.(foot)(3,j);
            end
            constraints = [constraints, ...
              angular_momentum.body(:,j+1) == new_momentum,...
              ];
          end

          % Enforce yaw rate
          constraints = [constraints,...
                         abs(pose.body(4,j+1) - pose.body(4,j)) <= obj.dt * obj.yaw_speed,...
                         ];
        end 

        % Enforce angle approximations
        for s = 1:length(angle_boundaries) - 1
          th0 = angle_boundaries(s);
          th1 = angle_boundaries(s+1);

          th = (th0 + th1)/2;
          ct = cos(th);
          st = sin(th);
          k = tan((th1 - th0)/2) / ((th1 - th0) / 2);
          constraints = [constraints,...
                         implies(yaw_sector(s,j), th0 <= body_yaw(j) <= th1)...
                         implies(yaw_sector(s,j), [cos_yaw(j); sin_yaw(j)] == [ct; st] + (body_yaw(j) - th) * k * [-st; ct])];
          if j < obj.nframes
            constraints = [constraints,...
                           sum(yaw_sector(max(s-1, 1):min(s+1, size(yaw_sector,1)), j+1)) >= yaw_sector(s, j)];
          end
          if j < obj.nframes - 1
            constraints = [constraints,...
                           sum(yaw_sector(max(s-1, 1):min(s+1, size(yaw_sector,1)), j+2)) >= yaw_sector(s, j)];
          end
        end

        % Foot-specific constraints
        for f = obj.feet
          foot = f{1};

          % Enforce reachability
          constraints = [constraints,...
            obj.body_to_feet_constraints.(foot).A(:,POSE_INDICES) * (pose.(foot)(:,j) - pose.body(:,j)) <= obj.body_to_feet_constraints.(foot).b];
          for k = 1:length(obj.foci.(foot))
            c = obj.foci.(foot)(k);
            constraints = [constraints, ...
              obj.pcone(pose.(foot)(1:2,j) - (pose.body(1:2,j) + [cos_yaw(j), -sin_yaw(j); sin_yaw(j), cos_yaw(j)] * c.v), c.r, 8)];
          end

          % Enforce region membership
          for r = 1:nregions
            Ar_ineq = obj.safe_regions(r).ineq.A(:,POSE_INDICES);
            br_ineq = obj.safe_regions(r).ineq.b;
            Ar_eq = obj.safe_regions(r).eq.A(:,POSE_INDICES);
            br_eq = obj.safe_regions(r).eq.b;
            if isa(region.(foot)(r,j), 'sdpvar')
              constraints = [constraints,...
                implies(region.(foot)(r,j), Ar_ineq * pose.(foot)(:,j) <= br_ineq),...
                implies(region.(foot)(r,j), Ar_eq * pose.(foot)(:,j) == br_eq),...
                % Friction cone (or simplex)
                implies(region.(foot)(r,j), contact_force.(foot)(:,j) == obj.safe_regions(r).force_basis * contact_weighting.(foot)(:,j)),...
                ];
            else
              constraints = [constraints,...
                Ar_ineq * pose.(foot)(:,j) <= br_ineq,...
                Ar_eq * pose.(foot)(:,j) == br_eq,...
                % Friction cone (or simplex)
                contact_force.(foot)(:,j) == obj.safe_regions(r).force_basis * contact_weighting.(foot)(:,j)',...
                ];
            end
          end
          
          if j < obj.nframes
            % enforce gait and swing speed
            constraints = [constraints,...
              implies(gait.(foot)(j), pose.(foot)(:,j) == pose.(foot)(:,j+1)),...
              implies(gait.(foot)(j), region.(foot)(:,j) == region.(foot)(:,j+1)),...
              obj.pcone((pose.(foot)(1:2,j+1) - pose.(foot)(1:2,j))/obj.dt - velocity.body(1:2,j), obj.swing_speed + MAX_DISTANCE / obj.dt * gait.(foot)(j), 4),...
              ];
          end
        end
      end

      objective = 0;

      if ~isempty(goal_pose)
        fnames = fieldnames(goal_pose)';
        for f = fnames
          field = f{1};
          for k = 1:length(POSE_INDICES)
            if ~isnan(goal_pose.(field)(POSE_INDICES(k)))
              constraints = [constraints, pose.(field)(k,end) == goal_pose.(field)(POSE_INDICES(k))];
            end
          end
        end
      end

      % Penalize contact force
      objective = objective + 0.5 * (sum(sum(contact_force.total.^2,1)) - 1 * (obj.nframes-1));

      % Penalize angular momentum
      if obj.use_angular_momentum
        objective = objective + 100 * sum(sum(abs(angular_momentum.body),1));
      end

      % Keep the legs near the body if possible
      for f = obj.feet
        foot = f{1};
        objective = objective + 0.1 * sum(sum(abs(pose.(foot)-pose.body), 1));
      end

      optimize(constraints, objective, sdpsettings('solver', 'gurobi'));

      % Extract the result
      fnames = fieldnames(pose)';
      for f = fnames
        field = f{1};
        pose.(field) = double(pose.(field));
      end
      region_assignments = struct();

      for f = obj.feet
        foot = f{1};
        region.(foot) = logical(round(double(region.(foot))));
        gait.(foot) = logical(round(double(gait.(foot))));
        for j = 1:obj.nframes
          r = find(region.(foot)(:,j));
          assert(length(r) <= 1);
          region_assignments(j).(foot) = r;
        end
      end

      t = 0:obj.dt:((obj.nframes - 1) * obj.dt);

      sol = GaitedFootstepPlanningSolution();
      sol.t = t;
      sol.pose = pose;
      sol.full_gait = gait;
      sol.safe_regions = obj.safe_regions;
      sol.region_assignments = region_assignments;

      figure(5)
      clf
      hold on
      subplot(311)
      plot(t, double(acceleration.body(3,:)));
      subplot(312)
      plot(t, double(velocity.body(3,:)));
      subplot(313)
      plot(t, double(pose.body(3,:)));

      figure(6);
      clf
      hold on
      subplot(221)
      plot(t, double(sqrt(sum(contact_force.lf.^2 ,1))));
      title('lf')
      subplot(222)
      plot(t, double(sqrt(sum(contact_force.rf.^2 ,1))));
      title('rf')
      subplot(223)
      plot(t, double(sqrt(sum(contact_force.lh.^2 ,1))));
      title('lh')
      subplot(224)
      plot(t, double(sqrt(sum(contact_force.rh.^2 ,1))));
      title('rh')

      figure(7);
      clf
      hold on
      subplot(221)
      plot(t, double(contact_force.lf(3,:)));
      title('lf')
      subplot(222)
      plot(t, double(contact_force.rf(3,:)));
      title('rf')
      subplot(223)
      plot(t, double(contact_force.lh(3,:)));
      title('lh')
      subplot(224)
      plot(t, double(contact_force.rh(3,:)));
      title('rh')

      figure(8);
      clf
      hold on
      subplot(221)
      plot(t(1:end-1), sqrt(sum(double((pose.lf(1:2,2:end) - pose.lf(1:2,1:end-1))/obj.dt - velocity.body(1:2,1:end-1)).^2,1)));
      title('lf')
      subplot(222)
      plot(t(1:end-1), sqrt(sum(double((pose.rf(1:2,2:end) - pose.rf(1:2,1:end-1))/obj.dt - velocity.body(1:2,1:end-1)).^2,1)));
      title('rf')
      subplot(223)
      plot(t(1:end-1), sqrt(sum(double((pose.lh(1:2,2:end) - pose.lh(1:2,1:end-1))/obj.dt - velocity.body(1:2,1:end-1)).^2,1)));
      title('lh')
      subplot(224)
      plot(t(1:end-1), sqrt(sum(double((pose.rh(1:2,2:end) - pose.rh(1:2,1:end-1))/obj.dt - velocity.body(1:2,1:end-1)).^2,1)));
      title('rh')

      double(contact_weighting.rf)
      double(contact_weighting.rh)

    end
  end
  methods(Static)
    function constraint = pcone(v, r, num_pieces)
      % Polynomial approximation of a conic constraint norm(v) <= r
      A = zeros(num_pieces, 2);
      b = repmat(r, num_pieces, 1);
      ths = linspace(0, 2*pi, num_pieces);
      for j = 1:num_pieces
        th = ths(j);
        c = cos(th);
        s = sin(th);
        R = [c, -s; s, c];
        A(j,:) = (R * [1;0])';
      end
      constraint = A * v <= b;
    end
        

  end
end
