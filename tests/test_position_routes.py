"""Synthetic geometry regression checks, not real-world accuracy validation."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/tram_odometry'))
from tram_odometry.position import PositionTracker, Route


def motion(t, distance):
    return dict(t=t, distance=distance, velocity=2., distance_variance=0.)


class RouteInitializationTests(unittest.TestCase):
    def test_configured_pose_projects_without_gnss_and_follows_turn(self):
        route = Route({'points_enu': [[0, 0, 0], [10, 0, 0], [10, 10, 0]]})
        tracker = PositionTracker(allow_gnss=False, route=route, initial_xyz=(5., 0., 0.))
        pose = tracker.pose(motion(4., 8.))
        self.assertEqual((pose['x'], pose['y']), (10., 3.))
        self.assertEqual(pose['status'], 'known_route')
        self.assertAlmostEqual(pose['yaw'], math.pi / 2)

    def test_explicit_station_is_not_rebased_by_initial_gnss(self):
        route = Route({'origin_lla': [55., 37., 150.], 'points_enu': [[0, 0, 0], [100, 0, 0]]})
        tracker = PositionTracker(route=route, route_s=20.)
        tracker.begin(0.)
        tracker.feed_gnss('master_fix', 2., (55., 37., 150., 2), motion(2., 4.))
        self.assertEqual(tracker.pose(motion(2., 4.))['x'], 24.)
        self.assertAlmostEqual(tracker.pose(motion(4., 8.))['x'], 28.)

    def test_auto_projection_preserves_gnss_anchor_distance(self):
        route = Route({'origin_lla': [55., 37., 150.], 'points_enu': [[0, 0, 0], [100, 0, 0]]})
        tracker = PositionTracker(route=route)
        tracker.begin(0.)
        tracker.feed_gnss('master_fix', 2., (55., 37., 150., 2), motion(2., 4.))
        tracker.feed_gnss('master_vel', 2., (2., 0., 0.), motion(2., 4.))
        self.assertEqual(tracker.pose(motion(4., 8.))['x'], 4.)

    def test_reverse_route_direction_and_exhaustion(self):
        route = Route({'points_enu': [[0, 0, 0], [10, 0, 0]]})
        tracker = PositionTracker(allow_gnss=False, route=route, route_s=8., route_direction=-1)
        self.assertEqual(tracker.pose(motion(2., 4.))['x'], 4.)
        self.assertEqual(tracker.pose(motion(6., 12.))['status'], 'route_exhausted')

if __name__ == '__main__':
    unittest.main()
