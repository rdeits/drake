classdef Quadruped 
  
  methods
    function qdtraj = trot(r,x0,body_spec,foot_spec,options)
      % @param r the robot
      % @param x0 initial state
      % @param body_spec struct with body_spec.body_ind and body_spec.pt for the
      %        "center" of the robot
      % @param foot_spec 4 element struct with footspec(i).body_ind
      %                                       footspec(i).contact_pt_ind
      %
      % @option num_steps will be rounded up to be a multiple of 4
      % @option step_speed in m/s
      % @option step_height in m
      % @option step_length in m
      % @option com_height in m
      % @option front_right_foot  xy vector from body to nominal front right foot
      % @option ignore_terrain
      % @options direction - 0 for forward, <0 for left, >0 for right
      % @options gait - 0 for quasi-static walk, 1 for zmp-walk, 2 for zmp-trot
      % @options duty_factor fraction of total stride time that each foot is in stance

      addpath(fullfile(getDrakePath,'examples','ZMP'));
      typecheck(r,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
      sizecheck(x0,[getNumStates(r) 1]);
      nq = getNumDOF(r);

      fieldcheck(body_spec,'body_ind');
      fieldcheck(body_spec,'pt');
      sizecheck(body_spec.pt,[3 1]);

      sizecheck(foot_spec,4);
      fieldcheck(foot_spec,'body_ind');
      fieldcheck(foot_spec,'contact_pt_ind');

      for i=1:4,
        pts = getContactPoints(getBody(r,foot_spec(i).body_ind));
        foot_spec(i).contact_pt_ind = foot_spec(i).contact_pt_ind(1);
        foot_spec(i).contact_pt = pts(:,foot_spec(i).contact_pt_ind);
        foot_spec(i).support_contact_pt_ind = 1:size(r.getBodyContacts(foot_spec(i).body_ind),2);
      end
      
      if nargin<3 || isempty(options), options=struct(); end
      if ~isfield(options,'num_steps') options.num_steps = 20; end
      if ~isfield(options,'duty_factor') options.duty_factor = 2/3; end
      if ~isfield(options,'step_length') options.step_length = .3; end
      if ~isfield(options,'step_speed') options.step_speed = .5; end
      if ~isfield(options,'step_height') options.step_height = .2; end
      if ~isfield(options,'com_height') options.com_height = .35; end
      if ~isfield(options,'comfortable_footpos') options.comfortable_footpos = [-.7 -.7 .6 .6; .3 -.3 -.3 .3]; end
      if ~isfield(options,'ignore_terrain') options.ignore_terrain = true; end  % todo: make this default to false
      if ~isfield(options,'direction') options.direction = 0; end
      if ~isfield(options,'gait') options.gait = 2; end
      if ~isfield(options,'draw') options.draw = false; end
      if ~isfield(options,'debug') options.debug = false; end
      if ~isfield(options,'x_nom') options.x_nom = x0; end
      if ~isfield(options,'delta_yaw') options.delta_yaw = 5*pi/180; end
      delta_yaw = options.direction*options.delta_yaw;
      q_nom = options.x_nom(1:nq);    
      
      % always take 4 steps at a time
      options.num_strides = ceil(options.num_steps/4);
      options.num_steps = 4*options.num_strides;
      swing_duration = abs(options.step_length/options.step_speed);
      stride_duration = swing_duration/(1-options.duty_factor);
      stance_duration = stride_duration*options.duty_factor;
      actuated = getActuatedJoints(r);
      
      cost = Point(getStateFrame(r),1);
      cost.base_x = 0;
      cost.base_y = 0;
      cost.base_z = 0;
      %cost.base_roll = 100;
      %cost.base_pitch = 10;
      cost.base_roll = 0;
      cost.base_pitch = 0;
      cost.base_yaw = 0;
      cost.back_bkz = 10;
      cost.back_bky = 100;
      cost.back_bkx = 100;
      cost = double(cost);
      options.Q = diag(cost(1:nq));
      options.q_nom = q_nom;
      
      function q = crawlIK(q0,com,fpos,swing_legs)
        if nargin<4, swing_legs=[]; end
        stance_legs = 1:4; stance_legs(swing_legs)=[];
        
        args = {0, com}; % com
        for i=stance_legs
          args = horzcat(args,{foot_spec(i).body_ind,foot_spec(i).contact_pt,fpos(:,i)});
        end
        for i=swing_legs(:)'
          if (fpos(2,i)>0) ymin = fpos(2,i); else ymin = -inf; end
          if (fpos(2,i)<0) ymax = fpos(2,i); else ymax = inf; end
          p.min = [fpos(1,i);ymin;fpos(3,i)];
          p.max = [fpos(1,i);ymax;nan];
          args = horzcat(args,{foot_spec(i).body_ind,foot_spec(i).contact_pt,p});
        end
        
        q = inverseKin(r,q0,args{:},options);
      end

      % ActionSequence crawl;
      crawl_sequence = ActionSequence();
      link_constraints = struct('link_ndx', {}, 'pt', {}, 'min_traj', [], 'max_traj', [], 'traj', {});
      
      % Determine forward crawling direction
      z_proj  = rpy2rotmat(x0(4:6))*[0;0;1];
      up_dir = [0;0;1];
      forward_dir = [z_proj(1:2);0];
      forward_dir = forward_dir/norm(forward_dir);
      left_dir = cross(up_dir,forward_dir);
      
      % Determine nominal forward crawling direction
      z_proj_nom = rpy2rotmat(options.x_nom(4:6))*[0;0;1];
      forward_dir_nom = [z_proj_nom(1:2);0];
      forward_dir_nom = forward_dir_nom/norm(forward_dir_nom);
      left_dir_nom = rotz(pi/2)*forward_dir_nom;
      
      kinsol = doKinematics(r,q_nom);
      for i = 1:4
        fpos_nom(:,i) = forwardKin(r,kinsol,foot_spec(i).body_ind,foot_spec(i).contact_pt);
        fpos_rel_nom(:,i) = fpos_nom(:,i) - q_nom(1:3);
        forward_coord_nom = fpos_rel_nom'*forward_dir_nom;
        left_coord_nom = fpos_rel_nom'*left_dir_nom;
      end
      
      q = x0(1:nq);
      kinsol = doKinematics(r,q);
      for i=1:4
        fpos_initial(:,i) = forwardKin(r,kinsol,foot_spec(i).body_ind,foot_spec(i).contact_pt);
      end
      com_initial = getCOM(r,kinsol);
      z_foot_nom = mean(fpos_initial(3,:));
      pelvis_ind = r.findLinkInd('pelvis');
      pelvis_pos_initial = forwardKin(r,kinsol,pelvis_ind,[0;0;0]);
      %hip0 = forwardKin(r,kinsol,body_spec.body_ind,body_spec.pt);
      %display(q,com,fpos,[]); pause(5);

    end
    
  end
end