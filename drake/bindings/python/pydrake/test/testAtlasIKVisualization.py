import unittest
import os.path
import numpy as np
from pydrake import rbtree
from pydrake import getDrakePath
from pydrake.trajectories import PiecewisePolynomial
from pydrake.solvers import ik
from pydrake.lcm import DrakeLcm
from pydrake.systems import DrakeVisualizer
from pydrake.parsers import PackageMap


class TestAtlasIKVisualization(unittest.TestCase):
    def test_ik_visualization(self):
        pm = PackageMap()
        model = os.path.join(getDrakePath(),
            "examples", "Atlas", "urdf", "atlas_minimal_contact.urdf")
        pm.PopulateUpstreamToDrake(model)
        robot = rbtree.RigidBodyTree(
            model,
            package_map=pm,
            floating_base_type=rbtree.FloatingBaseType.kRollPitchYaw)

        constraints = [
            ik.WorldPositionConstraint(
                robot,
                robot.findFrame("r_foot_sole").get_frame_index(),
                np.array([0., 0, 0]),
                np.array([0, -0.5, 0]),
                np.array([0, -0.5, 0])),
            ik.WorldPositionConstraint(
                robot,
                robot.FindBody("l_foot").get_body_index(),
                np.array([0., 0, 0]),
                np.array([0., 0, 0]),
                np.array([0., 0, 0])),
            ik.WorldCoMConstraint(
                robot,
                np.array([0, -0.25, 1.05]),
                np.array([0, -0.25, 1.05]))
        ]
        q_seed = robot.getZeroConfiguration()
        options = ik.IKoptions(robot)
        results = ik.InverseKin(robot, q_seed, q_seed, constraints, options)
        x = np.pad(results.q_sol[0],
                   (0, robot.number_of_velocities()), "constant")

        pp = PiecewisePolynomial.ZeroOrderHold([0., 0.001], [x, x])
        lcm = DrakeLcm()
        vis = DrakeVisualizer(robot, lcm)
        vis.PublishLoadRobot()
        vis.PlaybackTrajectory(pp)


if __name__ == '__main__':
    unittest.main()
