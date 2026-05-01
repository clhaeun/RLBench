from typing import List
from rlbench.backend.task import Task
from pyrep.const import PrimitiveShape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.conditions import DetectedCondition, ConditionSet
from rlbench.backend.conditions import JointCondition
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
from pyrep.objects.dummy import Dummy
from rlbench.backend.spawn_boundary import SpawnBoundary
import numpy as np
from rlbench.backend.conditions import EmptyCondition
from rlbench.backend.task_utils import sample_procedural_objects

DIRT_POINTS = 50

class MultitaskCleanup(Task):

    def init_task(self) -> None:
        self.put_rubbish_in_bin = Dummy('put_rubbish_in_bin')
        success_sensor = ProximitySensor('success')
        self.rubbish = Shape('rubbish')
        self.register_success_conditions(
            [DetectedCondition(self.rubbish, success_sensor)])

        self.dirt_spots = []
        self.sponge = Shape('sponge')
        self.sensor = ProximitySensor('sponge_sensor')
        self.register_graspable_objects([self.rubbish, self.sponge])

        boundaries = [Shape('dirt_boundary')]
        _, _, self.z_boundary = boundaries[0].get_position()
        self.b = SpawnBoundary(boundaries)

        self.large_container = Shape('large_container')
        self.target_container0 = Shape('small_container0')
        self.success_detector0 = ProximitySensor('success0')
        self.target_waypoint = Dummy('waypoint17')
        self.spawn_boundary = SpawnBoundary([Shape('spawn_boundary')])
        self.register_waypoint_ability_start(14, self._move_above_object)
        self.register_waypoints_should_repeat(self._repeat)
        self.bin_objects = []

    def init_episode(self, index: int) -> List[str]:
        self._variation_index = index
        self._place_dirt()
        curr_conditions = self.get_success_conditions()
        assert curr_conditions is not None and len(curr_conditions) > 0
        self.register_success_conditions(curr_conditions + [EmptyCondition(self.dirt_spots)])

        self.bin_objects = sample_procedural_objects(self.get_base(), 1)
        self.bin_objects_not_done = list(self.bin_objects)
        assert self.get_graspable_objects() is not None and len(self.get_graspable_objects()) > 0
        self.register_graspable_objects(self.get_graspable_objects() + self.bin_objects_not_done)
        self.spawn_boundary.clear()
        for ob in self.bin_objects:
            ob.set_position(
                [0, 0., 0.02], relative_to=self.large_container,
                reset_dynamics=False)
            self.spawn_boundary.sample(
                ob, ignore_collisions=True, min_distance=0.01)
        conditions = []
        # target_pos = [-5.9605e-8, -2.5005e-1, +1.7e-1]
        for ob in self.bin_objects:
            conditions.append(DetectedCondition(ob, self.success_detector0))
        # self.target_waypoint.set_position(
        #     target_pos, relative_to=self.large_container, reset_dynamics=True)
        curr_conditions = self.get_success_conditions()
        assert curr_conditions is not None and len(curr_conditions) > 0
        self.register_success_conditions(curr_conditions + [ConditionSet(conditions, simultaneously_met=True)])
        
        return ['']

    def variation_count(self) -> int:
        return 1
    
    def step(self) -> None:
        for d in self.dirt_spots:
            if self.sensor.is_detected(d):
                self.dirt_spots.remove(d)
                d.remove()
        
        for ob in self.bin_objects_not_done:
            if self.success_detector0.is_detected(ob):
                self.bin_objects_not_done.remove(ob)

    def cleanup(self) -> None:
        for d in self.dirt_spots:
            d.remove()
        self.dirt_spots = []

        [ob.remove() for ob in self.bin_objects if ob.still_exists()]
        self.bin_objects = []
    
    def _place_dirt(self):
        for i in range(DIRT_POINTS):
            spot = Shape.create(type=PrimitiveShape.CUBOID,
                                size=[.005, .005, .001],
                                mass=0, static=True, respondable=False,
                                renderable=True,
                                color=[0.58, 0.29, 0.0])
            spot.set_parent(self.get_base())
            spot.set_position([-1, -1, self.z_boundary + 0.001])
            self.b.sample(spot, min_distance=0.00,
                          min_rotation=(0.00, 0.00, 0.00),
                          max_rotation=(0.00, 0.00, 0.00))
            self.dirt_spots.append(spot)
        self.b.clear()
    
    def _move_above_object(self, waypoint):
        if len(self.bin_objects_not_done) <= 0:
            raise RuntimeError('Should not be here.')
        bin_obj = self.bin_objects_not_done[0]
        way_obj = waypoint.get_waypoint_object()
        way_obj.set_position(bin_obj.get_position())
        x, y, _ = way_obj.get_orientation()
        _, _, z = bin_obj.get_orientation(relative_to=way_obj)
        way_obj.set_orientation([x, y, z])
    
    def _repeat(self):
        return len(self.bin_objects_not_done) > 0
