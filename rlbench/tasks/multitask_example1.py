from typing import List
from rlbench.backend.task import Task
from pyrep.const import PrimitiveShape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.conditions import DetectedCondition
from rlbench.backend.conditions import JointCondition
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
from pyrep.objects.dummy import Dummy
from rlbench.backend.spawn_boundary import SpawnBoundary
import numpy as np
from rlbench.backend.conditions import EmptyCondition

DIRT_POINTS = 50

class MultitaskExample1(Task):

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

        

    def init_episode(self, index: int) -> List[str]:
        self._variation_index = index
        self._place_dirt()
        curr_conditions = self.get_success_conditions()
        assert curr_conditions is not None and len(curr_conditions) > 0
        self.register_success_conditions(curr_conditions + [EmptyCondition(self.dirt_spots)])
        
        return ['']

    def variation_count(self) -> int:
        return 1
    
    def step(self) -> None:
        for d in self.dirt_spots:
            if self.sensor.is_detected(d):
                self.dirt_spots.remove(d)
                d.remove()

    def cleanup(self) -> None:
        for d in self.dirt_spots:
            d.remove()
        self.dirt_spots = []
    
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