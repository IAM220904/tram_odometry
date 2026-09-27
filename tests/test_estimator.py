import math
import struct
import sys
import unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/tram_odometry'),str(ROOT/'tools')]
from tram_odometry.core import Config,Estimator
from tram_odometry.position import PositionTracker,Route,LocalFrame
from bag_io import decode


class EstimatorTests(unittest.TestCase):
    def test_constant_speed_distance_and_duplicates(self):
        e=Estimator()
        for k in range(201):
            t=k*.1;e.feed('command',t,0);e.feed('front',t,36.);e.feed('rear',t,36.)
        self.assertAlmostEqual(e.v,10,delta=.01)
        self.assertAlmostEqual(e.s,200,delta=.3)
        snapshot=(e.v,e.s,e.pv)
        self.assertFalse(e.feed('front',20.,0.))
        self.assertEqual(snapshot,(e.v,e.s,e.pv))

    def test_invalid_values_and_old_messages(self):
        e=Estimator();e.feed('front',10,36)
        for x in [math.nan,math.inf,-math.inf,1000.]:self.assertFalse(e.feed('front',10.1,x))
        self.assertFalse(e.feed('command',10.1,16))
        self.assertFalse(e.feed('command',10.1,.5))
        self.assertFalse(e.feed('rear',9,36))
        self.assertTrue(math.isfinite(e.v));self.assertEqual(e.v,10)

    def test_one_axle_spike_and_two_axle_short_lock(self):
        e=Estimator()
        for k in range(101):
            t=k*.1;e.feed('command',t,0)
            front=72. if 2<t<4 else 36.
            rear=36.
            if 6<t<7:front=rear=0.
            e.feed('front',t,front);e.feed('rear',t,rear)
            if t>1:self.assertAlmostEqual(e.v,10,delta=.3)
        self.assertGreater(e.rejected,10)

    def test_finite_dropout_and_recovery(self):
        e=Estimator()
        for k in range(20):e.feed('front',k*.1,36);e.feed('rear',k*.1,36)
        e.predict(10.)
        self.assertEqual(e.snapshot()['status'],'model_only')
        self.assertTrue(all(math.isfinite(x) for x in [e.s,e.v,e.ps,e.pv]))
        for k in range(101,151):e.feed('front',k*.1,28.8);e.feed('rear',k*.1,28.8)
        self.assertAlmostEqual(e.v,8,delta=.1)

    def test_time_chunking_distance(self):
        e=Estimator();e.feed('front',0,0);e.predict(200.)
        self.assertEqual(e.v,0);self.assertEqual(e.s,0)
        self.assertGreater(e.pv,1)

    def test_gnss_quarantine(self):
        p=PositionTracker(init_seconds=3.)
        motion={'t':0.,'distance':0.,'velocity':2.,'distance_variance':0.}
        p.begin(0);p.feed_gnss('master_fix',0,(55.,37.,150.,2),motion)
        motion.update(t=4.,distance=8.)
        before=p.pose(motion)
        self.assertFalse(p.feed_gnss('master_fix',4,(60.,40.,1000.,2),motion))
        after=p.pose(motion)
        self.assertEqual((before['x'],before['y'],before['z']),(after['x'],after['y'],after['z']))
        self.assertTrue(after['gnss_locked'])

    def test_known_route_turn_and_end(self):
        r=Route({'points_enu':[[0,0,0],[10,0,0],[10,10,0]]})
        xyz,yaw,end=r.at(15)
        self.assertEqual(xyz,(10.,5.,0.));self.assertAlmostEqual(yaw,math.pi/2);self.assertFalse(end)
        self.assertTrue(r.at(21)[2])
        s,direction=r.project((5,1,0),0)
        self.assertAlmostEqual(s,5);self.assertEqual(direction,1)

    def test_enu_origin_and_orientation(self):
        f=LocalFrame((55.,37.,150.))
        self.assertEqual(f.enu(55.,37.,150.),(0.,0.,0.))
        self.assertGreater(f.enu(55.,37.001,150.)[0],60.)
        self.assertGreater(f.enu(55.001,37.,150.)[1],100.)

    def test_cdr_endianness_and_alignment(self):
        for endian,flag in [('<',1),('>',0)]:
            raw=bytes([0,flag,0,0])+struct.pack(endian+'iII',123,456,10)+b'base_link\x00'
            raw+=b'\x00'*((-(len(raw)-4))%8)+struct.pack(endian+'d',36.)
            stamp,values=decode(raw,'front');self.assertEqual(stamp,123000000456);self.assertEqual(values,(36.,))
        with self.assertRaises(ValueError):decode(b'bad','front')

if __name__=='__main__':unittest.main()
