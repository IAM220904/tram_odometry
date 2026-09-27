"""ROS adapter tests without a DDS participant; source Humble to run these."""
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/tram_odometry'))
try:
    from tram_odometry.node import OdometryNode
    from tram_odometry.core import Config
except ImportError:
    OdometryNode=None


@unittest.skipIf(OdometryNode is None,'ROS Python packages unavailable; source Humble and workspace')
class NodeRobustnessTests(unittest.TestCase):
    def test_clock_reset_allows_immediate_diagnostics(self):
        settings=dict(configured_origin=False,allow_initial_gnss=True,initialization_seconds=3.,
            initial_yaw=0.,initial_x=0.,initial_y=0.,initial_z=0.,route_start_s=-1.,route_direction=1,dual_gnss_heading=True)
        node=SimpleNamespace(config=Config(),settings=settings,route=None,path_map=None,last_diagnostic=1234.,last_published=1234.)
        OdometryNode.reset_state(node)
        self.assertTrue(10.-node.last_diagnostic>=1.,'Diagnostics must resume in the new clock episode')
        self.assertIsNone(node.last_published)

    def test_invalid_ros_input_is_counted_without_state_access(self):
        node=SimpleNamespace(arrival_clock=lambda:10.,invalid_inputs=0,header_time=OdometryNode.header_time)
        for value in [math.nan,math.inf]:
            message=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10,nanosec=0)),velocity=value)
            OdometryNode.input_callback(node,'front',message)
        self.assertEqual(node.invalid_inputs,2)

if __name__=='__main__':unittest.main()
