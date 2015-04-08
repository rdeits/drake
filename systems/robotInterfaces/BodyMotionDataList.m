classdef BodyMotionDataList
  properties
    motion_data = BodyMotionData.empty();
  end

  properties(SetAccess=protected)
    body_id
  end

  methods
    function obj = BodyMotionDataList(body_id, ts)
      % Initialize the list of body motion data objects
      obj.motion_data(length(ts) - 1) = BodyMotionData();

      obj = obj.setTs(ts);
      obj = obj.setBodyId(body_id);
    end

    function t_ind = findInd(obj, t)
      if t < obj.motion_data(1).ts(1)
        t_ind = 1;
      elseif t >= obj.motion_data(end).ts(end)
        t_ind = length(obj.motion_data);
      else
        ts = vertcat(obj.motion_data.ts);
        t_ind = find(ts(:,1) <= t, 1, 'last');
      end
    end

    function body_motion_data = slice(obj, ind)
      body_motion_data = obj.motion_data(ind);
    end

    function body_motion_data = sliceAtTime(obj, t)
      body_motion_slice = obj.slice(obj.findInd(t));
    end

    function [x, xd, xdd] = eval(obj, t)
      [x, xd, xdd] = obj.evalAtInd(obj.findInd(t), t);
    end

    function [x, xd, xdd] = evalAtInd(obj, ind, t)
      [x, xd, xdd] = obj.motion_data(ind).eval(t);
    end

    function obj = setBodyId(obj, body_id)
      obj.body_id = body_id;
      for j = 1:length(obj.motion_data)
        obj.motion_data(j) = obj.motion_data(j).setBodyId(body_id);
      end
    end

    function obj = setTs(obj, ts)
      for j = 1:length(ts)-1
        obj.motion_data(j) = obj.motion_data(j).setTs(ts(j:j+1));
      end
      obj.motion_data = obj.motion_data(1:length(ts)-1);
    end

    function obj = setCoefs(obj, coefs)
      for j = 1:size(coefs, 2)
        obj.motion_data(j) = obj.motion_data(j).setCoefs(coefs(:,j,:));
      end
    end

    function obj = setInNullspace(obj, in_floating_base_nullspace)
      if numel(in_floating_base_nullspace) == 1
        in_floating_base_nullspace = repmat(in_floating_base_nullspace, 1, length(obj.motion_data));
      end
      for j = 1:length(obj.motion_data)
        obj.motion_data(j) = obj.motion_data(j).setInNullspace(in_floating_base_nullspace(j));
      end
    end

    function obj = setToeOffAllowed(obj, toe_off_allowed)
      if numel(toe_off_allowed) == 1
        toe_off_allowed = repmat(toe_off_allowed, 1, length(obj.motion_data));
      end
      for j = 1:length(obj.motion_data)
        obj.motion_data(j) = obj.motion_data(j).setToeOffAllowed(toe_off_allowed(j));
      end
    end

    function obj = setControlInContact(obj, control_pose_when_in_contact)
      if numel(control_pose_when_in_contact) == 1
        control_pose_when_in_contact = repmat(control_pose_when_in_contact, 1, length(obj.motion_data));
      end
      for j = 1:length(obj.motion_data)
        obj.motion_data(j) = obj.motion_data(j).setControlInContact(control_pose_when_in_contact(j));
      end
    end

    function [ts, coefs] = getPPComponents(obj)
      ts = vertcat(obj.motion_data.ts);
      ts = [ts(:,1)', ts(end,end)];
      coefs = horzcat(obj.motion_data.coefs);
    end

    function pp = getPP(obj)
      [ts, coefs] = obj.getPPComponents();
      pp = mkpp(ts, coefs, size(coefs, 1));
    end
  end

  methods (Static)
    function obj = from_body_poses(bodi_id, ts, poses)
      obj = BodyMotionDataList(body_id, ts);
      pp = pchip(ts, poses);
      [~, coefs, l, k, d] = unmkpp(pp);
      coefs = reshape(coefs, [d, l, k]);
      obj = obj.setCoefs(coefs);
    end

    function obj = from_body_poses_and_velocities(body_id, ts, poses, dposes)
      obj = BodyMotionDataList(body_id, ts);
      pp = pchipDeriv(ts, poses, dposes);
      [~, coefs, l, k, d] = unmkpp(pp);
      coefs = reshape(coefs, [d, l, k]);
      obj = obj.setCoefs(coefs);
    end
  end

end





