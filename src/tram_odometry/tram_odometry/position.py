"""Initial GNSS alignment, relative odometry and optional known-route geometry.

Only the initialization window accepts GNSS. Afterwards position is a function
of estimated path distance, initial conditions and an explicitly supplied route.
"""
import bisect
import json
import math
from pathlib import Path


def ecef(lat, lon, alt):
    lat=math.radians(lat);lon=math.radians(lon)
    a=6378137.;e2=6.6943799901413165e-3
    n=a/math.sqrt(1.-e2*math.sin(lat)**2)
    return ((n+alt)*math.cos(lat)*math.cos(lon),(n+alt)*math.cos(lat)*math.sin(lon),(n*(1-e2)+alt)*math.sin(lat))


class LocalFrame:
    def __init__(self,origin):
        self.origin=tuple(origin); self.xyz=ecef(*origin)
        lat,lon=map(math.radians,origin[:2])
        self.sl=math.sin(lat);self.cl=math.cos(lat);self.so=math.sin(lon);self.co=math.cos(lon)
    def enu(self,lat,lon,alt):
        x,y,z=(a-b for a,b in zip(ecef(lat,lon,alt),self.xyz))
        return (-self.so*x+self.co*y,-self.sl*self.co*x-self.sl*self.so*y+self.cl*z,
                self.cl*self.co*x+self.cl*self.so*y+self.sl*z)


class Route:
    """One ordered polyline, not a branch selector. Units are meters, frame ENU."""
    def __init__(self,data):
        self.origin=data.get('origin_lla');self.points=[tuple(map(float,p)) for p in data['points_enu']]
        if len(self.points)<2 or any(len(p)!=3 or not all(map(math.isfinite,p)) for p in self.points):
            raise ValueError('Route needs at least two finite 3D points')
        self.arc=[0.]
        for a,b in zip(self.points,self.points[1:]):
            ds=math.dist(a,b)
            if ds<=1e-6:raise ValueError('Repeated route point')
            self.arc.append(self.arc[-1]+ds)
    @classmethod
    def load(cls,path):return cls(json.loads(Path(path).read_text()))
    def at(self,s,direction=1):
        exhausted=s<0 or s>self.arc[-1]
        s=max(0.,min(self.arc[-1],s));i=max(0,min(len(self.points)-2,bisect.bisect_right(self.arc,s)-1))
        a=self.points[i];b=self.points[i+1];ratio=(s-self.arc[i])/(self.arc[i+1]-self.arc[i])
        xyz=tuple(a[k]+ratio*(b[k]-a[k]) for k in range(3))
        yaw=math.atan2(direction*(b[1]-a[1]),direction*(b[0]-a[0]))
        return xyz,yaw,exhausted
    def project(self,xyz,yaw,max_distance=20.):
        candidates=[]
        for i,(a,b) in enumerate(zip(self.points,self.points[1:])):
            d=[b[k]-a[k] for k in range(3)];den=sum(x*x for x in d)
            r=max(0.,min(1.,sum((xyz[k]-a[k])*d[k] for k in range(3))/den))
            p=[a[k]+r*d[k] for k in range(3)];distance=math.dist(p,xyz)
            heading=math.atan2(d[1],d[0]);dot=math.cos(heading-yaw)
            if distance<=max_distance and abs(dot)>.7:
                candidates.append((distance,self.arc[i]+r*(self.arc[i+1]-self.arc[i]),1 if dot>=0 else -1))
        if not candidates:raise ValueError('Initial pose does not match route')
        candidates.sort();best=candidates[0]
        if any(c[0]<best[0]+1. and abs(c[1]-best[1])>15. for c in candidates[1:]):
            raise ValueError('Ambiguous route crossing: configure initial route_s explicitly')
        return best[1],best[2]


class PathMap:
    """Directed track-node map (a pathgraph without explicit edges).

    Each node is a mean position [m, ENU] with a unit travel direction, so the two
    tracks of a double line and the two travel directions stay separate. Runtime
    follows the local node tangent that agrees with the current heading; no GNSS.
    """
    def __init__(self,data):
        self.origin=data['origin_lla'];self.cell=float(data.get('cell_m',2.))
        self.nodes=[tuple(map(float,n[:5])) for n in data['nodes']]
        if not self.nodes:raise ValueError('Empty path map')
        self.grid={}
        for i,n in enumerate(self.nodes):
            self.grid.setdefault((math.floor(n[0]/self.cell),math.floor(n[1]/self.cell)),[]).append(i)
    @classmethod
    def load(cls,path):return cls(json.loads(Path(path).read_text()))
    def nearest(self,x,y,radius=5.):
        r=int(math.ceil(radius/self.cell));cx=math.floor(x/self.cell);cy=math.floor(y/self.cell)
        best=None;bd=radius*radius
        for i in range(cx-r,cx+r+1):
            for j in range(cy-r,cy+r+1):
                for k in self.grid.get((i,j),()):
                    n=self.nodes[k];d2=(n[0]-x)**2+(n[1]-y)**2
                    if d2<=bd:best=n;bd=d2
        return best
    def local(self,x,y,hx,hy,radius=5.,min_dot=.7):
        """Weighted mean (x,y,z,tx,ty) of nearby nodes heading like (hx,hy), or None."""
        r=int(math.ceil(radius/self.cell));cx=math.floor(x/self.cell);cy=math.floor(y/self.cell)
        sw=sx=sy=sz=stx=sty=0.
        for i in range(cx-r,cx+r+1):
            for j in range(cy-r,cy+r+1):
                for k in self.grid.get((i,j),()):
                    n=self.nodes[k];d2=(n[0]-x)**2+(n[1]-y)**2
                    if d2>radius*radius or n[3]*hx+n[4]*hy<min_dot:continue
                    w=math.exp(-d2/4.);sw+=w;sx+=w*n[0];sy+=w*n[1];sz+=w*n[2];stx+=w*n[3];sty+=w*n[4]
        if sw<1e-9:return None
        norm=math.hypot(stx,sty)
        if norm<1e-9:return None
        return sx/sw,sy/sw,sz/sw,stx/norm,sty/norm


class MapFollower:
    """Advance a pose by path distance along PathMap; hold heading when off-map."""
    def __init__(self,path_map,xyz,yaw,s):
        self.map=path_map;self.x,self.y,self.z=xyz;self.hx=math.cos(yaw);self.hy=math.sin(yaw);self.s=s
        self.off_map_m=0.;self.on_map=False
        self.advance(s)
    def advance(self,s):
        ds=s-self.s;self.s=s
        steps=max(1,int(math.ceil(abs(ds)/1.)))
        for _ in range(steps if ds!=0 else 1):
            step=ds/steps
            local=self.map.local(self.x,self.y,self.hx,self.hy)
            self.on_map=local is not None
            if local is not None:
                mx,my,mz,tx,ty=local
                self.hx,self.hy=tx,ty
                # Pull toward the track centreline across the heading only.
                lateral=(mx-self.x)*-ty+(my-self.y)*tx
                self.x+=-ty*lateral*.5;self.y+=tx*lateral*.5;self.z+=(mz-self.z)*.2
            else:self.off_map_m+=abs(step)
            self.x+=self.hx*step;self.y+=self.hy*step
    def yaw(self):return math.atan2(self.hy,self.hx)


class PositionTracker:
    def __init__(self,init_seconds=3.,allow_gnss=True,initial_yaw=0.,initial_xyz=(0.,0.,0.),
                 origin=None,route=None,route_s=None,route_direction=1,dual_heading=True,path_map=None):
        self.init_seconds=init_seconds;self.allow_gnss=allow_gnss;self.dual_heading=dual_heading
        self.start=None;self.locked=not allow_gnss;self.yaw=initial_yaw;self.heading_known=not allow_gnss
        self.anchor_xyz=tuple(initial_xyz);self.anchor_s=0.;self.fix_seen=False
        self.route=route;self.route_s=route_s;self.route_direction=1 if route_direction>=0 else -1
        # Explicit route_s is the station at the start of this odometry episode.
        # GNSS updates the free-space anchor, not this route distance reference.
        self.route_anchor_s=0.
        self.path_map=path_map;self.follower=None;self.heading_from_map=False
        if origin is None and route is not None:origin=route.origin
        if origin is None and path_map is not None:origin=path_map.origin
        self.frame=LocalFrame(origin) if origin is not None else None
        self.fixes={};self.gnss_used=0;self.gnss_ignored=0;self.route_error=''
        if self.route is not None and self.route_s is None and not allow_gnss:
            try:self.route_s,self.route_direction=self.route.project(self.anchor_xyz,self.yaw)
            except ValueError as exc:self.route_error=str(exc)
    def begin(self,t):
        if self.start is None:self.start=t
    def advance(self,t):
        self.begin(t)
        if not self.locked and t>self.start+self.init_seconds:
            self.locked=True
            if self.route and self.route_s is None:
                if self.fix_seen and self.heading_known:
                    try:
                        self.route_s,self.route_direction=self.route.project(self.anchor_xyz,self.yaw)
                        self.route_anchor_s=self.anchor_s
                    except ValueError as exc:self.route_error=str(exc)
                else:self.route_error='Route alignment needs an initial GNSS pose or explicit route_s'
    def feed_gnss(self,kind,t,values,motion):
        self.advance(motion['t'])
        if self.locked or not self.allow_gnss or t<self.start-.5 or t>self.start+self.init_seconds:
            self.gnss_ignored+=1;return False
        if not all(math.isfinite(x) for x in values):return False
        if kind.endswith('_fix'):
            lat,lon,alt,status=values
            if status<0 or abs(lat)>90 or abs(lon)>180:return False
            if self.frame is None:
                # Master defines origin when available; rover may arrive first.
                if kind!='master_fix':self.fixes[kind]=(t,values);return False
                self.frame=LocalFrame((lat,lon,alt))
            self.fixes[kind]=(t,values)
            if kind=='master_fix':
                self.anchor_xyz=self.frame.enu(lat,lon,alt)
                self.anchor_s=motion['distance']-motion['velocity']*max(0.,motion['t']-t)
                self.fix_seen=True
            if self.dual_heading and 'master_fix' in self.fixes and 'rover_fix' in self.fixes:
                mt,m=self.fixes['master_fix'];rt,r=self.fixes['rover_fix']
                if abs(mt-rt)<=.15:
                    mp=self.frame.enu(*m[:3]);rp=self.frame.enu(*r[:3])
                    d=[rp[i]-mp[i] for i in range(3)]
                    # Dataset calibration: rover ahead of master, separation ~12.4 m.
                    if 9.<math.hypot(d[0],d[1])<16.:
                        self.yaw=math.atan2(d[1],d[0]);self.heading_known=True
        elif kind.endswith('_vel'):
            east,north,_up=values
            if math.hypot(east,north)>.8:
                self.yaw=math.atan2(north,east);self.heading_known=True
        else:return False
        self.gnss_used+=1;return True
    def pose(self,motion):
        self.advance(motion['t']);ds=motion['distance']-self.anchor_s
        if self.route is not None and self.route_s is not None:
            route_ds=motion['distance']-self.route_anchor_s
            xyz,yaw,exhausted=self.route.at(self.route_s+self.route_direction*route_ds,self.route_direction)
            status='route_exhausted' if exhausted else 'known_route'
        else:
            xyz=(self.anchor_xyz[0]+ds*math.cos(self.yaw),self.anchor_xyz[1]+ds*math.sin(self.yaw),self.anchor_xyz[2])
            yaw=self.yaw
            status='heading_held_no_map' if self.heading_known else 'relative_distance_only'
            if self.path_map is not None and self.locked and self.fix_seen and not self.heading_known:
                # Stationary start without dual-antenna heading: each physical track of the
                # double line carries one direction, so take it from the nearest map node.
                nearest=self.path_map.nearest(self.anchor_xyz[0],self.anchor_xyz[1])
                if nearest is not None:self.yaw=math.atan2(nearest[4],nearest[3]);self.heading_known=True;self.heading_from_map=True
            if self.path_map is not None and self.locked and self.fix_seen and self.heading_known:
                # Path-map following starts when the GNSS window closes; GNSS unused after.
                if self.follower is None:self.follower=MapFollower(self.path_map,xyz,yaw,motion['distance'])
                else:self.follower.advance(motion['distance'])
                xyz=(self.follower.x,self.follower.y,self.follower.z);yaw=self.follower.yaw()
                status='path_map' if self.follower.on_map else 'path_map_off_map'
        # Heading cannot be observed from three scalar inputs after initialization.
        unknown=(self.route is None or self.route_s is None) and not (self.follower and self.follower.on_map)
        cross_var=1e6 if unknown else 4.
        return dict(x=xyz[0],y=xyz[1],z=xyz[2],yaw=yaw,status=status,
                    longitudinal_variance=max(motion['distance_variance'],.25),cross_variance=cross_var,
                    yaw_variance=math.pi**2 if unknown else .04,gnss_locked=self.locked,
                    gnss_used=self.gnss_used,gnss_ignored=self.gnss_ignored,route_error=self.route_error)
