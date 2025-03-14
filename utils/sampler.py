import os
import sys
import git
import math
import numpy as np

git_repo = git.Repo(__file__, search_parent_directories=True)
git_root = git_repo.git.rev_parse("--show-toplevel")
sys.path.append(git_root)

from external.motion_planners.motion_planners.primitives import get_difference_fn
from external.motion_planners.motion_planners.utils import interval_generator, get_distance, wrap_interval, get_difference, \
    UNBOUNDED_LIMITS, INF

STEP_SIZE = 1e-2
MIN_PROXIMITY = 1e-3

def is_collision_free(line, **kwargs):
    return not line_collides(line, **kwargs)

def line_collides(line, step_size=STEP_SIZE, **kwargs):
    return any(point_collides(point, **kwargs) for point in sample_line(line, step_size=step_size))

def point_collides(point, sim, conf_ids, geom_info, buffer=MIN_PROXIMITY, **kwargs):
    # point in range
    sim.data.qpos[conf_ids] = point
    sim.forward()
    for contact in sim.data.contact:
        if (contact.geom1 in geom_info['robot'].values()) and (contact.geom2 in geom_info['obstacle'].values()) or\
           (contact.geom1 in geom_info['obstacle'].values()) and (contact.geom2 in geom_info['robot'].values()):
            print(contact)
            return True
    return False

def sample_line(segment, step_size=STEP_SIZE):
    (q1, q2) = segment
    diff = np.array(q2) - np.array(q1)
    dist = np.linalg.norm(diff)
    for l in np.arange(0., dist, step_size):
        yield tuple(np.array(q1) + l * diff / dist)
    yield q2

def get_distance_fn(weights, difference_fn=get_difference):
    # TODO: careful with circular joints
    def fn(q1, q2):
        diff = np.array(difference_fn(q2, q1))
        return np.sqrt(np.dot(weights, diff * diff))
    return fn


##################################################

def wrap_sample_fn(sample_fn):
    samples = []

    def new_sample_fn(*args, **kwargs):
        q = sample_fn(*args, **kwargs)
        samples.append(q)
        return q

    return new_sample_fn, samples


def get_sample_fn(conf_region, only_cfree=True, **kwargs): #, check_collisions=False):
    # TODO: additional rejection function
    # TODO: Gaussian sampling for narrow passages
    collision_fn = get_collision_fn(**kwargs)
    lower, upper = conf_region
    generator = interval_generator(lower, upper, **kwargs)

    def region_gen():
        #area = np.product(upper - lower) # TODO: sample_fn proportional to area
        for q in generator:
            #q = sample_box(region)
            if only_cfree and collision_fn(q):
                continue
            return q # TODO: sampling with state (e.g. deterministic sampling)

    return region_gen


def get_connected_test(obstacles, max_distance=0.25): # 0.25 | 0.2 | 0.25 | 0.5 | 1.0
    roadmap = []

    def connected_test(q1, q2):
        threshold = max_distance
        are_connected = (get_distance(q1, q2) <= threshold) and is_collision_free((q1, q2), obstacles)
        if are_connected:
            roadmap.append((q1, q2))
        return are_connected
    return connected_test, roadmap


def get_threshold_fn(d=2):
    # http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.419.5503&rep=rep1&type=pdf
    vol_free = (1 - 0) * (1 - 0)
    vol_ball = math.pi * (1 ** 2)
    gamma = 2 * ((1 + 1. / d) * (vol_free / vol_ball)) ** (1. / d)
    threshold_fn = lambda n: gamma * (math.log(n) / n) ** (1. / d)
    return threshold_fn


def wrap_collision_fn(collision_fn):
    colliding = []
    cfree = []
    # TODO: KDTree for hyperspheres
    # TODO: Signed Distance Function (SDF)

    def new_collision_fn(q, *args, **kwargs):
        result = collision_fn(q, *args, **kwargs)
        if result:
            colliding.append(q)
        else:
            cfree.append(q)
        return result

    return new_collision_fn, colliding, cfree


def get_collision_fn(**kwargs):

    def collision_fn(q):
        return point_collides(q, **kwargs)

    return collision_fn


##################################################


def wrap_extend_fn(extend_fn):
    roadmap = []

    def new_extend_fn(q1, q2, *args, **kwargs):
        raise NotImplementedError()

    return new_extend_fn, roadmap


def get_extend_fn(circular={}, step_size=STEP_SIZE, norm=INF):
    #difference_fn = get_difference
    difference_fn = get_difference_fn(circular=circular)
    def fn(q1, q2):
        # steps = int(np.max(np.abs(np.divide(difference_fn(q2, q1), resolutions))))
        # steps = int(np.linalg.norm(np.divide(difference_fn(q2, q1), resolutions), ord=norm))
        steps = int(np.linalg.norm(np.array(difference_fn(q2, q1)) / step_size, ord=norm))
        num_steps = steps + 1
        q = q1
        for i in range(num_steps):
            q = (1. / (num_steps - i)) * np.array(difference_fn(q2, q)) + q
            q = [wrap_interval(v, circular.get(i, UNBOUNDED_LIMITS)) for i, v in enumerate(q)]
            q = np.array(q) # tuple
            yield q
    return fn


def get_wrapped_extend_fn(environment, obstacles=[], **kwargs):
    collision_fn = get_collision_fn(environment, obstacles)
    roadmap = []

    def extend_fn(q1, q2):
        path = [q1]
        for q in sample_line(segment=(q1, q2), **kwargs):
            yield q
            if collision_fn(q):
                path = None
            if path is not None:
                roadmap.append((path[-1], q))
                path.append(q)

    return extend_fn, roadmap