"""Fictional station geometry and interacting JuPedSim pedestrians.

Crowd dynamics are planar; the G1 body runs separately in MuJoCo. A refreshed
stationary circle represents the measured G1 footprint in the crowd engine.
"""
from dataclasses import dataclass
import xml.etree.ElementTree as ET
import numpy as np
from shapely.geometry import Point, LineString, box
from shapely.ops import unary_union


@dataclass(frozen=True)
class Fixture:
    name: str
    x: float
    y: float
    sx: float  # horizontal half extents; cylinder uses sx as radius
    sy: float
    height: float
    kind: str = 'box'

    @property
    def polygon(self):
        if self.kind == 'cylinder':
            return Point(self.x, self.y).buffer(self.sx, quad_segs=16)
        return box(self.x-self.sx, self.y-self.sy, self.x+self.sx, self.y+self.sy)


def fixtures():
    result = [Fixture('wall_n', 12, 7.15, 15, .15, 2.8),
              Fixture('wall_s', 12, -7.15, 15, .15, 2.8)]
    result += [Fixture(f'pillar_{i}_{j}', x, y, .38, .38, 3.6, 'cylinder')
               for i, x in enumerate((5., 15., 23.)) for j, y in enumerate((-3., 3.))]
    result += [Fixture(f'bench_{i}_{j}', x, y, 1.2, .42, .55)
               for i, x in enumerate((3., 18.)) for j, y in enumerate((-5.6, 5.6))]
    result += [Fixture(f'gate_{i}', 10., y, .7, .22, 1.05)
               for i, y in enumerate((-6., -4.5, -1.5, 1.5, 4.5, 6.))]
    return result


class StationLayout:
    def __init__(self):
        self.fixtures = fixtures()
        self.solid = unary_union([f.polygon for f in self.fixtures])
        self.walkable = box(-3, -7, 27, 7).difference(self.solid)
        self.discs = []
        for f in self.fixtures:
            if f.kind == 'cylinder':
                self.discs.append([f.x, f.y, 0, 0, f.sx])
                continue
            nx, ny = int(np.ceil(2*f.sx/.6)), int(np.ceil(2*f.sy/.6))
            dx, dy = 2*f.sx/nx, 2*f.sy/ny
            radius = float(np.hypot(dx, dy)/2)
            self.discs += [[f.x-f.sx+(i+.5)*dx, f.y-f.sy+(j+.5)*dy, 0, 0, radius]
                           for i in range(nx) for j in range(ny)]

    def clearance(self, xy, radius=.35):
        return float(self.solid.distance(Point(*xy)) - radius)

    def visible(self, pose, target, people=(), fov=140., sensor_range=8.):
        delta = np.asarray(target) - pose[:2]
        distance = np.linalg.norm(delta)
        angle = (np.arctan2(delta[1], delta[0]) - pose[2] + np.pi) % (2*np.pi) - np.pi
        if distance > sensor_range or abs(angle) > np.deg2rad(fov/2):
            return False
        line = LineString([pose[:2], target])
        if line.intersects(self.solid):
            return False
        for x, y, _, _, radius in people:
            # A center ray is a conservative idealized visibility proxy.
            if line.distance(Point(x, y)) < radius:
                return False
        return True

    def obstacles(self, xy, people):
        return [o for o in [*self.discs, *people]
                if np.linalg.norm(np.asarray(o[:2])-xy) - o[4] < 5.]


class StationCrowd:
    """Four origin/destination streams with continuous, collision-aware motion."""
    def __init__(self, layout, count=20, seed=42, target_speed=.4,
                 crowd_speed=.8, pattern='bidirectional'):
        import jupedsim as jps
        self.jps = jps
        self.rng = np.random.default_rng(seed)
        self.layout = layout
        self.count, self.pattern = count, pattern
        self.target_speed, self.crowd_speed = target_speed, crowd_speed
        self.time, self.spawned, self.exited = 0., 0, 0
        self.sim = jps.Simulation(model=jps.CollisionFreeSpeedModel(),
                                  geometry=layout.walkable, dt=.02)
        # Target remains at its final waypoint instead of disappearing.
        target_points = [(7., 0.), (13., 0.), (18., 1.), (24., 1.)]
        target_stages = [self.sim.add_waypoint_stage(p, .45) for p in target_points]
        journey = jps.JourneyDescription(target_stages)
        for first, second in zip(target_stages, target_stages[1:]):
            journey.set_transition_for_stage(first, jps.Transition.create_fixed_transition(second))
        jid = self.sim.add_journey(journey)
        self.target_id = self.sim.add_agent(jps.CollisionFreeSpeedModelAgentParameters(
            position=(2., 0.), journey_id=jid, stage_id=target_stages[0],
            radius=.25, desired_speed=target_speed, time_gap=1.2))
        self.target_points, self.target_stages = target_points, target_stages
        exits = [box(25.5, -6.8, 26.8, 6.8), box(-2.8, -6.8, -1.5, 6.8),
                 box(12., -6.8, 21., -6.1), box(12., 6.1, 21., 6.8)]
        self.journeys = []
        for exit_polygon in exits:
            sid = self.sim.add_exit_stage(exit_polygon)
            self.journeys.append((self.sim.add_journey(jps.JourneyDescription([sid])), sid))
        self.ids = [None] * count
        self.last_positions = {}
        self.velocities = {}
        self.heading = 0.
        self.robot_position = np.array([0., 0.])
        shadow_stage = self.sim.add_direct_steering_stage()
        self.shadow_journey = self.sim.add_journey(jps.JourneyDescription([shadow_stage]))
        self.shadow_stage = shadow_stage
        self.shadow_id = None
        self.shadow_update_failures = 0
        self._add_shadow()
        for i in range(count if pattern == 'bidirectional' else count//2):
            self._spawn(i, initial=True)
        self._remember()

    def _add_shadow(self):
        # Public JuPedSim positions are read-only. Recreate the robot proxy
        # using its measured position instead of changing private native state.
        try:
            self.shadow_id = self.sim.add_agent(self.jps.CollisionFreeSpeedModelAgentParameters(
                position=tuple(self.robot_position), journey_id=self.shadow_journey,
                stage_id=self.shadow_stage, radius=.45, desired_speed=0.))
            self.sim.agent(self.shadow_id).target = tuple(self.robot_position)
        except RuntimeError:
            # A failed placement means a person is already too close. Preserve
            # the failure in metrics; MuJoCo contact checks remain authoritative.
            self.shadow_id = None
            self.shadow_update_failures += 1

    def _spawn(self, slot, initial=False):
        stream = slot % 4
        # Initial occupants spread throughout the concourse; replacement agents
        # appear only at entrances, away from the robot and existing occupants.
        for _ in range(200):
            if stream < 2:
                x = self.rng.uniform(-1, 25) if initial else (-1. if stream == 0 else 25.)
                y = self.rng.uniform(-5, 5)
            else:
                x = self.rng.uniform(13., 21.)
                y = self.rng.uniform(-5.5, 5.5) if initial else (5.8 if stream == 2 else -5.8)
            point = Point(x, y)
            if not self.layout.walkable.buffer(-.45).contains(point):
                continue
            if np.linalg.norm(np.array([x, y])-self.robot_position) < 2.0:
                continue
            if any(np.linalg.norm(np.array([x, y])-a.position) < .85 for a in self.sim.agents()):
                continue
            jid, sid = self.journeys[stream]
            speed = self.crowd_speed * self.rng.uniform(.75, 1.2)
            self.ids[slot] = self.sim.add_agent(self.jps.CollisionFreeSpeedModelAgentParameters(
                position=(float(x), float(y)), journey_id=jid, stage_id=sid,
                radius=.25, desired_speed=float(speed), time_gap=float(self.rng.uniform(.9, 1.5))))
            self.spawned += 1
            return True
        return False

    def _remember(self):
        self.last_positions = {a.id: np.asarray(a.position) for a in self.sim.agents()}

    def step(self, robot_xy, stop_at=20., stop_for=3.):
        self.robot_position = np.asarray(robot_xy).copy()
        paused = stop_for > 0 and stop_at <= self.time < stop_at + stop_for
        self.sim.agent(self.target_id).model.desired_speed = 0. if paused else self.target_speed
        self._remember()
        if self.shadow_id is not None:
            self.sim.mark_agent_for_removal(self.shadow_id)
        # Removal is committed at the next native iteration. There is a .02 s
        # handoff gap, followed by .08 s reacting to the refreshed footprint.
        self.sim.iterate(count=1)
        self._add_shadow()
        self.sim.iterate(count=4)
        self.time += .1
        alive = {a.id for a in self.sim.agents()}
        self.velocities = {a.id: (np.asarray(a.position)-self.last_positions.get(a.id, np.asarray(a.position)))/.1
                           for a in self.sim.agents()}
        for slot, aid in enumerate(self.ids):
            if aid is not None and aid not in alive:
                self.ids[slot] = None
                self.exited += 1
        # Arrival wave fills the remaining slots at three people per second.
        allowed = self.count if self.pattern == 'bidirectional' else min(
            self.count, self.count//2 + max(0, int((self.time-12.)*3)))
        for slot in range(allowed):
            if self.ids[slot] is None:
                self._spawn(slot)

    def target(self):
        a = self.sim.agent(self.target_id)
        velocity = self.velocities.get(a.id, np.zeros(2))
        if np.linalg.norm(velocity) > .01:
            self.heading = float(np.arctan2(velocity[1], velocity[0]))
        return np.asarray(a.position).copy(), velocity.copy(), self.heading

    def people(self):
        result = []
        for aid in self.ids:
            if aid is None:
                result.append(None)
            else:
                a = self.sim.agent(aid)
                result.append([*a.position, *self.velocities.get(aid, [0, 0]), .25])
        return result


def _geom(parent, **kwargs):
    return ET.SubElement(parent, 'geom', **{k: str(v) for k, v in kwargs.items()})


def decorate_station(root, world, count, seed):
    """Procedural station; people use mocap segments, leaving G1's DOFs intact."""
    global_visual = root.find('visual/global')
    global_visual.set('offwidth', '1280'); global_visual.set('offheight', '720')
    for body in list(world.findall('body')):
        if body.get('name', '').startswith(('follow_target', 'pedestrian_')):
            world.remove(body)
    texture = root.find("asset/texture[@name='groundplane']")
    texture.set('rgb1', '.61 .64 .65'); texture.set('rgb2', '.68 .70 .71')
    texture.set('markrgb', '.43 .46 .47')
    material = root.find("asset/material[@name='groundplane']")
    material.set('texrepeat', '1 1'); material.set('reflectance', '.03')
    material.set('specular', '0'); material.set('shininess', '0')
    for f in fixtures():
        color = '.55 .58 .60 1'
        if f.name.startswith('bench'):
            color = '.24 .32 .37 1'
        if f.name.startswith('gate'):
            color = '.21 .33 .42 1'
        kind = 'cylinder' if f.kind == 'cylinder' else 'box'
        size = f'{f.sx} {f.height/2}' if kind == 'cylinder' else f'{f.sx} {f.sy} {f.height/2}'
        _geom(world, name=f'station_collision_{f.name}', type=kind, size=size,
              pos=f'{f.x} {f.y} {f.height/2}', rgba=color, friction='1 .01 .001')
        if f.name.startswith('gate'):
            _geom(world, type='box', size='.16 .23 .025', pos=f'{f.x-.4} {f.y} {f.height+.03}',
                  rgba='.15 .9 .45 1', contype=0, conaffinity=0)
        if f.name.startswith('bench'):
            _geom(world, type='box', size=f'{f.sx} .08 .3',
                  pos=f'{f.x} {f.y+np.sign(f.y)*.35} .85', rgba=color, contype=0, conaffinity=0)
    # Tactile paving and flow markings have no physical collision surface.
    _geom(world, type='box', size='14 .12 .002', pos='12 -.85 .003',
          rgba='.85 .68 .16 1', contype=0, conaffinity=0)
    for x in (1., 12., 21.):
        _geom(world, type='box', size='1.1 .08 .3', pos=f'{x} 3.1 3.1',
              rgba='.08 .25 .34 1', contype=0, conaffinity=0)
        _geom(world, type='box', size='.65 .085 .035', pos=f'{x} 3.1 3.1',
              rgba='.85 .95 .87 1', contype=0, conaffinity=0)
    for x in (2., 11., 20.):
        ET.SubElement(world, 'light', pos=f'{x} 0 4.8', dir='0 0 -1',
                      diffuse='.45 .45 .43', directional='false', castshadow='false')
    rng = np.random.default_rng(seed)
    for i in range(count+1):
        name = f'person_{i}'
        color = [.1, .65, .3] if i == 0 else rng.uniform(.15, .75, 3)
        rgba = ' '.join(map(str, [*color, 1.]))
        body = ET.SubElement(world, 'body', name=name, mocap='true', pos='0 0 -10')
        _geom(body, name=f'{name}_collision', type='capsule',
              fromto='0 0 .35 0 0 1.35', size='.25', rgba='0 0 0 0', group=3)
        _geom(body, type='ellipsoid', size='.19 .13 .32', pos='0 0 1.12',
              rgba=rgba, contype=0, conaffinity=0)
        _geom(body, type='sphere', size='.12', pos='0 0 1.58',
              rgba='.7 .5 .36 1', contype=0, conaffinity=0)
        for limb in range(4):
            segment = ET.SubElement(world, 'body', name=f'{name}_limb_{limb}', mocap='true', pos='0 0 -10')
            _geom(segment, type='capsule', size=f'{.055 if limb < 2 else .045} .32',
                  rgba='.14 .18 .23 1' if limb < 2 else rgba, contype=0, conaffinity=0)


def animate_person(model, data, index, xy, velocity, heading, phase):
    name = f'person_{index}'
    mid = model.body(name).mocapid[0]
    if xy is None:
        data.mocap_pos[mid] = [0, 0, -10]
        for limb in range(4):
            data.mocap_pos[model.body(f'{name}_limb_{limb}').mocapid[0]] = [0, 0, -10]
        return
    data.mocap_pos[mid] = [*xy, 0]
    data.mocap_quat[mid] = [np.cos(heading/2), 0, 0, np.sin(heading/2)]
    moving = min(float(np.linalg.norm(velocity))/.6, 1.)
    rotation = np.array([[np.cos(heading), -np.sin(heading)], [np.sin(heading), np.cos(heading)]])
    for limb in range(4):
        side = 1 if limb % 2 == 0 else -1
        swing = .20 * np.sin(phase + (0 if side == 1 else np.pi)) * moving
        if limb >= 2:
            swing = -swing
        a = np.array([0, side*(.11 if limb < 2 else .23), .83 if limb < 2 else 1.35])
        b = a + [swing, 0, -.64]
        a[:2] = xy + rotation @ a[:2]; b[:2] = xy + rotation @ b[:2]
        direction = (a-b)/np.linalg.norm(a-b)
        quat = np.array([1+direction[2], -direction[1], direction[0], 0.])
        quat /= np.linalg.norm(quat)
        lid = model.body(f'{name}_limb_{limb}').mocapid[0]
        data.mocap_pos[lid] = (a+b)/2
        data.mocap_quat[lid] = quat
