classdef ContactRegion
% A container class for convex regions of safe terrain, typically produced by IRIS
  properties
    ineq = struct('A', [], 'b', []);
    eq = struct('A', [], 'b', []);
    force_basis = 10 * [[1;0;1], [0;1;1], [-1;0;1], [0;-1;1]];
  end

  methods(Static)
    function obj = fromTerrain(A, b, point, normal)
      sizecheck(A, [nan, 2]);
      sizecheck(b, size(A, 1));
      sizecheck(point, [3,1]);
      sizecheck(normal, [3,1]);

      obj = ContactRegion();
      obj.ineq = struct('A', [A, zeros(size(A,1), 4)], 'b', b);
      obj.eq = struct('A', [normal', zeros(1,3)], 'b', normal' * point);

      axis = cross([0;0;1], normal);
      angle = atan2(norm(axis), dot([0;0;1], normal));
      if angle > 1e-6
        R = axis2rotmat([axis; angle]);
      else
        R = eye(3);
      end
      obj.force_basis = 10 * R * [[1;0;1], [0;1;1], [-1;0;1], [0;-1;1]];
    end

    function obj = fromTerrainWithYaw(A, b, point, normal)
      sizecheck(A, [nan, 3]);
      sizecheck(b, size(A, 1));
      sizecheck(point, [3,1]);
      sizecheck(normal, [3,1]);

      obj = ContactRegion();
      obj.ineq = struct('A', [A(:,1:2), zeros(size(A,1), 3), A(:,3)], 'b', b);
      obj.eq = struct('A', [normal', zeros(1,3)], 'b', normal' * point);

      axis = cross([0;0;1], normal);
      angle = atan2(norm(axis), dot([0;0;1], normal));
      R = axis2rotmat([axis; angle]);
      obj.force_basis = 10 * R * [[1;0;1], [0;1;1], [-1;0;1], [0;-1;1]];
    end
  end
end
 