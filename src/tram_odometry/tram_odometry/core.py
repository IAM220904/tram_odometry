"""Causal nonlinear model observer with adaptive wheel trust.

Time is seconds in the input Header clock domain. Each input is assimilated once.
GNSS is intentionally absent from this module's API.
"""
from dataclasses import dataclass, field, fields
import math


def clip(x, low, high):
    return max(low, min(high, x))


@dataclass
class Config:
    wheel_scale: float = 1.0 / 3.6
    coefficients: list = field(default_factory=lambda: [1.8, 0., 1.87, 0., .018, .061, 0.])
    drive_tau: float = .25
    wheel_sigma: float = .025
    acceleration_noise: float = .7
    slip_threshold: float = .65
    wheel_difference_threshold: float = .45
    max_acceleration: float = 3.
    stale_timeout: float = .5
    command_timeout: float = .5
    coast_after: float = 2.
    out_of_order_tolerance: float = .25
    stop_velocity: float = .025
    reacquire_after: float = 3.

    @classmethod
    def from_dict(cls, data):
        known = {f.name for f in fields(cls)}
        extra = set(data) - known
        if extra:
            raise ValueError('Unknown model settings: ' + ', '.join(sorted(extra)))
        result = cls(**data)
        if len(result.coefficients) != 7 or not all(math.isfinite(v) and v >= 0 for v in result.coefficients):
            raise ValueError('Expected seven finite nonnegative acceleration coefficients')
        for f in fields(cls):
            if f.name != 'coefficients' and (not math.isfinite(getattr(result, f.name)) or getattr(result, f.name) <= 0):
                raise ValueError('Invalid positive setting: ' + f.name)
        return result


class Dynamics:
    def __init__(self, config): self.c = config

    def acceleration(self, command, speed):
        v = max(0., speed)
        p = max(command, 0.) / 15.; b = max(-command, 0.) / 15.
        c = self.c.coefficients
        traction = (c[0]*p+c[1]*p*p)/(1.+v/12.)
        brake = c[2]*b+c[3]*b*b
        drag = c[4]*math.tanh(v/.3)+c[5]*v/15.+c[6]*v*v/225.
        return clip(traction-brake-drag, -self.c.max_acceleration, self.c.max_acceleration)


class Estimator:
    """Two-state Kalman correction (distance, velocity), nonlinear prediction.

    Acceleration is a bounded auxiliary observer; its model residual is adapted
    only from accepted wheel measurements. This is not an UKF.
    """
    def __init__(self, config=None):
        self.c = config or Config()
        self.model = Dynamics(self.c)
        self.t = None
        self.s = 0.; self.v = 0.; self.a = 0.; self.disturbance = 0.
        self.ps = 0.; self.pv = 1.; self.psv = 0.
        self.command = 0.; self.command_t = -math.inf
        self.last_seen = {}; self.wheels = {}; self.accepted = {}
        self.suspicion = {'front': -math.inf, 'rear': -math.inf}
        self.rejected = 0; self.out_of_order = 0; self.duplicates = 0
        self.initialized = False
        self.accepted_count = 0

    def predict(self, t):
        if not math.isfinite(t): return False
        if self.t is None:
            self.t = t
            return True
        if t <= self.t: return False
        remaining = t-self.t
        # Bounded work for arbitrarily large gaps; never integrate thousands of steps.
        if remaining > 2.:
            self.predict(self.t+2.)
            rest = t-self.t
            self.s += self.v*rest
            self.ps += self.pv*rest*rest + self.c.acceleration_noise**2*rest**4/4
            self.pv += self.c.acceleration_noise**2*rest*rest
            self.a = 0.; self.t = t
            return True
        while self.t < t - 1e-9:
            dt = min(.1, t-self.t)
            now = self.t+dt
            last_good = max((x[0] for x in self.accepted.values()), default=-math.inf)
            stale = now-last_good > self.c.coast_after
            command = self.command if now-self.command_t <= self.c.command_timeout else 0.
            target = self.model.acceleration(command,self.v)+self.disturbance
            if stale: target = 0.
            self.a += (target-self.a)*(1.-math.exp(-dt/self.c.drive_tau))
            self.a = clip(self.a,-self.c.max_acceleration,self.c.max_acceleration)
            v1 = clip(self.v+self.a*dt,0.,40.)
            self.s += .5*(self.v+v1)*dt
            q = self.c.acceleration_noise**2 * (4. if stale else 1.)
            self.ps += 2*dt*self.psv+dt*dt*self.pv+.25*q*dt**4
            self.psv += dt*self.pv+.5*q*dt**3
            self.pv += q*dt*dt
            self.v = v1; self.t = now
        self.t = t
        return True

    def feed(self, kind, t, value):
        if kind not in ('front','rear','command'):
            raise ValueError('Only the three permitted inputs are accepted')
        if not math.isfinite(t) or not math.isfinite(value):
            self.rejected += 1; return False
        if kind == 'command':
            if abs(value)>15 or value != round(value):
                self.rejected += 1; return False
        elif abs(value*self.c.wheel_scale)>40:
            self.rejected += 1; return False
        previous_t = self.last_seen.get(kind, -math.inf)
        if t <= previous_t:
            self.duplicates += int(t == previous_t)
            self.out_of_order += int(t < previous_t)
            return False
        if self.t is not None and t < self.t-self.c.out_of_order_tolerance:
            self.out_of_order += 1; return False
        self.last_seen[kind] = t
        if self.t is None: self.t = t
        if t > self.t: self.predict(t)
        if kind == 'command':
            self.command = value; self.command_t = t
            return True
        z = max(0.,value*self.c.wheel_scale)
        previous = self.wheels.get(kind)
        self.wheels[kind] = (t,z)
        if not self.initialized:
            self.v = z; self.pv = self.c.wheel_sigma**2
            self.initialized = True; self.accepted[kind] = (t,z)
            return True
        age = max(0.,self.t-t)
        z_now = max(0.,z+self.a*age)
        residual = z_now-self.v
        other = self.wheels.get('rear' if kind == 'front' else 'front')
        disagree = False
        if other and abs(t-other[0]) <= self.c.stale_timeout:
            other_now = max(0.,other[1]+self.a*max(0.,self.t-other[0]))
            disagree = abs(z_now-other_now)>self.c.wheel_difference_threshold
            # When one source disagrees, prefer the source closest to prediction.
            if disagree and abs(residual)>abs(other_now-self.v)+.15:
                self.suspicion[kind] = self.t
                self.rejected += 1; return False
        gate = self.c.slip_threshold + .015*self.v
        jumping = False
        if previous and 0 < t-previous[0] < .4:
            jumping = abs(z-previous[1]) > self.c.max_acceleration*(t-previous[0])+.2
        bad = abs(residual)>gate or jumping
        # Reacquire sustained, mutually consistent measurements after loss/model drift.
        last_good = max((x[0] for x in self.accepted.values()), default=-math.inf)
        recover = (self.t-last_good > self.c.reacquire_after and other is not None
                   and abs(t-other[0])<.25 and not disagree and not jumping)
        if bad and not recover:
            self.suspicion[kind] = self.t
            self.rejected += 1; return False
        R = self.c.wheel_sigma**2 + (.25*age)**2
        if disagree: R *= 9.
        if recover:
            self.pv = max(self.pv, .3**2)
            R *= 4.
        denom = self.pv+R
        kv = self.pv/denom; ks = self.psv/denom
        self.v = clip(self.v+kv*residual,0.,40.)
        self.s += ks*residual
        old_psv = self.psv
        self.ps = max(0.,self.ps-ks*old_psv)
        self.psv *= (1.-kv)
        self.pv = max(1e-9, self.pv*(1.-kv))
        accepted_previous = self.accepted.get(kind)
        if accepted_previous and .04<=t-accepted_previous[0]<=.5 and not disagree and not recover:
            measured_a = clip((z-accepted_previous[1])/(t-accepted_previous[0]),-2.5,2.5)
            self.a += .4*(measured_a-self.a)
            target_bias = measured_a-self.model.acceleration(self.command,self.v)
            self.disturbance += .015*(clip(target_bias,-.6,.6)-self.disturbance)
        self.accepted[kind] = (t,z); self.accepted_count += 1
        # No zero-velocity reset from a locked axle alone.
        if other and abs(t-other[0])<.25 and z<self.c.stop_velocity and other[1]<self.c.stop_velocity and self.v<.12:
            self.v = 0.; self.a = 0.; self.disturbance *= .9
        return True

    def snapshot(self):
        t = self.t if self.t is not None else 0.
        missing = [k for k in ('front','rear') if t-self.accepted.get(k,(-math.inf,0.))[0]>self.c.stale_timeout]
        slip = [k for k in ('front','rear') if t-self.suspicion[k]<.5]
        state = 'waiting' if not self.initialized else 'model_only' if len(missing)==2 else 'degraded' if missing or slip else 'ok'
        return dict(t=t,velocity=self.v,distance=self.s,acceleration=self.a,
                    velocity_variance=self.pv,distance_variance=self.ps,
                    status=state,slip=slip,missing=missing,rejected=self.rejected,
                    out_of_order=self.out_of_order,duplicates=self.duplicates,
                    model_disturbance=self.disturbance)


class SourceClock:
    """Monotone Header-domain clock anchored to arrival time of the newest input.

    In ROS the arrival clock is ROS time with --clock; for live use it is steady
    time. It preserves Header time in outputs and advances between messages.
    """
    def __init__(self): self.stamp=None; self.arrival=None

    def observe(self, stamp, arrival):
        if self.stamp is None or stamp > self.stamp:
            self.stamp=stamp; self.arrival=arrival

    def now(self, arrival):
        return None if self.stamp is None else self.stamp+max(0.,arrival-self.arrival)
