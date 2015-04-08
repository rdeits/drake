classdef BodyMotionData
  properties
    ts = [0, inf];
    coefs = zeros(6, 1, 4);
    body_id = [];
    toe_off_allowed = false;
    in_floating_base_nullspace = false;
    control_pose_when_in_contact = false;
  end

  methods
    function obj = BodyMotionData()
    end

    function [x, xd, xdd] = eval(obj, t)
      t_rel = t - obj.ts(1);
      x = obj.coefs(:,1,1)*t_rel^3 + obj.coefs(:,1,2)*t_rel^2 + obj.coefs(:,1,3)*t_rel + obj.coefs(:,1,4);
      xd = 3*obj.coefs(:,1,1)*t_rel^2 + 2*obj.coefs(:,1,2)*t_rel + obj.coefs(:,1,3);
      xdd = 6*obj.coefs(:,1,1)*t_rel + 2*obj.coefs(:,1,2);
    end

    function obj = setTs(obj, ts)
      % assert(all(size(ts) == [1,2]), 'wrong size');
      obj.ts = ts;
    end

    function obj = setCoefs(obj, coefs)
      % assert(all(size(coefs) == [6,1,4]), 'wrong size');
      obj.coefs = coefs;
    end

    function obj = setBodyId(obj, body_id)
      % assert(numel(body_id) == 1, 'wrong size');
      obj.body_id = body_id;
    end

    function obj = setToeOffAllowed(obj, toe_off_allowed)
      % assert(numel(toe_off_allowed) == 1, 'wrong size');
      obj.toe_off_allowed = logical(toe_off_allowed);
    end

    function obj = setInNullspace(obj, in_floating_base_nullspace)
      % assert(numel(in_floating_base_nullspace) == 1, 'wrong size');
      obj.in_floating_base_nullspace = logical(in_floating_base_nullspace);
    end

    function obj = setControlInContact(obj, control_pose_when_in_contact)
      % assert(numel(control_pose_when_in_contact) == 1, 'wrong size');
      obj.control_pose_when_in_contact = logical(control_pose_when_in_contact);
    end
  end
end

