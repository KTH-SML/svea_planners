import pdb
import math
import operator
import itertools
import numpy as np


def dist(a, b):
    a = np.asarray(a)
    b = np.asarray(b)
    return np.linalg.norm(a - b)

class TrajectoryPlanner(object):

    def __init__(self, xs, ys):
        self.xs = xs
        self.ys = ys

    def create_path(self):

        traj_x, traj_y = [], []
        for i in range(len(self.xs)-1):
            traj_x += np.linspace(self.xs[i], self.xs[i+1], 100).tolist()
            traj_y += np.linspace(self.ys[i], self.ys[i+1], 100).tolist()

        traj = []
        for x, y in zip(traj_x, traj_y):
            traj.append((x, y))

        return traj


class AStarNode(object):

    def __init__(self, world, ind, p, h):
        self.world = world
        self.state = np.asarray(ind)
        self.parent = p
        self.h_cost = h
        self.g_cost = self.calc_g()

    def __repr__(self):
        fmt = '<AStarNode: {}, h={}, g={}>'
        return fmt.format(self.state, self.h_cost, self.g_cost)

    def __hash__(self):
            return hash(tuple(self.state))

    def __eq__(self, other):
        if isinstance(other, AStarNode):
            return np.all(self.state == other.state)
        return NotImplemented

    def calc_g(self):
        if self.parent is None:
            return 0
        elif not self.is_free:
            return float('inf')
        else:
            cost_to_parent = dist(self.state, self.parent.state)
            return self.parent.g_cost + cost_to_parent

    def family_tree(self):
        if self.parent is not None:
            for n in self.parent.family_tree():
                yield n
        yield self

    @property
    def is_free(self):
        return self.world.is_free_ind(self.state)

    @property
    def pos(self):
        return self.world.ind_to_pos(self.state)

    @property
    def f_cost(self):
        return self.g_cost + self.h_cost


class AStarWorld(object):

    DELTA = None
    LIMIT = None

    OBSTACLE_MARGIN = 0
    OBS = None

    _res = None
    _occupancy_grid = None

    def __init__(self, delta=None, limit=None, obstacles=None, obs_margin=0):

        self.DELTA = np.asarray(delta or self.DELTA)
        self.LIMIT = np.asarray(limit or self.LIMIT)
        self.OBS = obstacles or self.OBS
        self.OBSTACLE_MARGIN = obs_margin or self.OBSTACLE_MARGIN


        if self._res is None:
            self._res = (self.LIMIT[:, 1] - self.LIMIT[:, 0]) / self.DELTA
            self._res = tuple(self._res.astype(int))

        if self._occupancy_grid is None:
            self._occupancy_grid = np.zeros(self._res).astype(bool)

            obs = np.array([[x, y, rad + self.OBSTACLE_MARGIN]
                            for x, y, rad in self.OBS])

            self._select_circle_occupants(obs)

    def _select_circle_occupants(self, obs):
        occ_grid = np.zeros(self._res)
        for xc, yc, r in obs:
            xx, yy = np.meshgrid(np.linspace(self.LIMIT[0,0], self.LIMIT[0,1], self._res[0]),
                                 np.linspace(self.LIMIT[1,0], self.LIMIT[1,1], self._res[1]))
            dist = np.hypot(xx-xc, yy-yc)
            inside = dist < r
            occ_grid += inside.astype(int).T
        self._occupancy_grid = occ_grid.astype(bool)


    def _select_rectangle_occupants(self, obs):
        for xc, yc, r in obs:
            for x in np.arange(xc-r, xc+r, self.DELTA[0]):
                for y in np.arange(yc-r, yc+r, self.DELTA[0]):
                    i, j = self.pos_to_ind((x, y))
                    i = max(0, min(i, self._res[0]-1))
                    j = max(0, min(j, self._res[1]-1))
                    self._occupancy_grid[i,j] = True

    def __contains__(self, pos):
        ret = 1
        for x, (mn, mx) in zip(pos, self.LIMIT):
            ret &= int(mn <= x <= mx)
        return bool(ret)

    def __and__(self, other):

        assert np.all(self.DELTA == other.DELTA), 'Different discretization'

        mns = np.minimum(self.LIMIT[:, 0], other.LIMIT[:, 0])
        mxs = np.maximum(self.LIMIT[:, 1], other.LIMIT[:, 1])

        class World(AStarWorld):
            DELTA = self.DELTA
            LIMIT = np.array([mns, mxs]).T
            _occupancy_grid = self._occupancy_grid | other._occupancy_grid

        return World()

    def adjacent(self, ind):
        v = np.asarray(ind)
        dv = np.array([v-1, v, v+1])            # get +1/-1 index
        adj = np.array(np.meshgrid(*dv.T))      # get combinations
        return adj.T.reshape(-1, v.shape[0])    # reshape into correct shape

    def is_free_ind(self, ind):
        i, j = ind[:2]
        if not 0 <= i < self._res[0]:
            return False
        if not 0 <= j < self._res[1]:
            return False
        return not self._occupancy_grid[i,j]

    def pos_to_ind(self, pos):
        return np.round((np.asarray(pos) - self.LIMIT[:, 0]) / self.DELTA).astype(int)

    def ind_to_pos(self, ind):
        return np.asarray(ind) * self.DELTA + self.LIMIT[:, 0]


class AStarPlanner(object):

    world = None
    init = None
    goal = None

    contour = None
    visited = None

    def __init__(self, world, init_pos, goal_pos):
        self.world = world
        self.init = self.world.pos_to_ind(init_pos)
        self.goal = self.world.pos_to_ind(goal_pos)

        init_node = AStarNode(self.world, self.init, None, dist(self.init, self.goal))

        self.contour = set()
        self.visited = set()

        self.contour.add(init_node)

    def iter_new_children(self, node):
        for ind in self.world.adjacent(node.state):
            yield AStarNode(self.world, ind, node, dist(ind, self.goal))

    def _run(self):

        while self.contour:

            best = min(self.contour, key=lambda n: n.f_cost)
            if best.f_cost == float('inf'):
                break

            self.contour.remove(best)
            self.visited.add(best)

            if dist(best.state, self.goal) <= 1:
                return best

            for child in self.iter_new_children(best):

                if child in self.visited:
                    continue

                if child in self.contour:

                    # set are funny
                    other = self.contour.intersection({child}).pop()

                    if child.g_cost <= other.g_cost:
                        # sets are funny pt. 2
                        self.contour.remove(other)
                        self.contour.add(child)

                else:
                    self.contour.add(child)

        raise Exception('Could not find a path')

    def create_path(self):
        final_node = self._run()
        return list(map(lambda n: n.pos, final_node.family_tree()))

