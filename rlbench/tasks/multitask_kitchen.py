from typing import List, Tuple
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.object import Object
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.joint import Joint
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.spawn_boundary import SpawnBoundary
import random

UNIQUE_PEPPERS_TO_PLACE = 3

GROCERY_NAMES = [
    # 'crackers',
    # 'chocolate jello',
    'strawberry jello',
    'soup',
    'tuna',
    'spam',
    # 'coffee',
    'mustard',
    # 'sugar',
]

class MultitaskKitchen(Task):

    def init_task(self) -> None:
        self.needle = Shape('scales_meter_needle')
        self.needle_pivot = Shape('scales_meter_pivot')
        self.top_plate = Shape('scales_tray_visual')
        self.joint = Joint('scales_joint')

        _, _, starting_z = self.top_plate.get_position()
        self.top_plate_starting_z = starting_z
        self.needle_starting_ori = self.needle.get_orientation(
            relative_to=self.needle_pivot)

        self.peppers = [Shape('pepper%d' % i) for i in range(3)]
        self.boundary = Shape('peppers_boundary')
        self.success_detector = ProximitySensor('success_detector')
        self.needle_detector = ProximitySensor('needle_sensor')
        self.success_conditions = [NothingGrasped(self.robot.gripper),
                                   DetectedCondition(
                                       self.needle, self.needle_detector)
                                   ]
        ###
        self.w6 = Dummy('waypoint6')
        self.w6_rel_pos = [+2.6203 * 10 ** (-3), -1.7881 * 10 ** (-7),
                           +1.5197 * 10 ** (-1)]
        self.w6_rel_ori = [-3.1416, -1.7467 * 10 ** (-2), -3.1416]

        wine_bottle = Shape('wine_bottle')
        self.register_success_conditions(
            [DetectedCondition(wine_bottle, ProximitySensor('success'))])

        ###
        self.groceries = [Shape(name.replace(' ', '_'))
                          for name in GROCERY_NAMES]
        self.grasp_points = [Dummy('%s_grasp_point' % name.replace(' ', '_'))
                             for name in GROCERY_NAMES]
        self.waypoint12 = Dummy('waypoint12')
        self.boundary_groceries = SpawnBoundary([Shape('boundary_root')])

        self.register_graspable_objects(self.peppers + [wine_bottle] + self.groceries)
        

    def init_episode(self, index: int) -> List[str]:
        self.target_pepper_index = index    #random.randint(0, UNIQUE_PEPPERS_TO_PLACE - 1)
        print(f"pepper_index: {self.target_pepper_index}")
        while len(self.success_conditions) > 2:
            self.success_conditions.pop()
        self.success_conditions.append(DetectedCondition(
            self.peppers[self.target_pepper_index], self.success_detector))
        self.register_success_conditions(self.get_success_conditions() + self.success_conditions)
        b = SpawnBoundary([self.boundary])
        for p in self.peppers:
            b.sample(p, ignore_collisions=False, min_distance=0.12,
                     min_rotation=(0.00, 0.00, -3.14),
                     max_rotation=(0.00, 0.00, +3.14)
                     )

        self.w6.set_position(self.w6_rel_pos,
                             relative_to=self.peppers[
                                 self.target_pepper_index],
                             reset_dynamics=False)
        self.w6.set_orientation(self.w6_rel_ori,
                                relative_to=self.peppers[
                                    self.target_pepper_index],
                                reset_dynamics=False)
        pepper = {0: 'green', 1: 'red', 2: 'yellow'}

        ###
        # randomize groceries index
        grocery_index = random.randint(0, len(GROCERY_NAMES) - 1)
        print(f"grocery_index: {grocery_index}, name: {GROCERY_NAMES[grocery_index]}")
        self.boundary_groceries.clear()
        [self.boundary_groceries.sample(g, min_distance=0.1) for g in self.groceries]
        self.waypoint12.set_pose(self.grasp_points[grocery_index].get_pose())
        self.register_success_conditions(self.get_success_conditions() + [
            DetectedCondition(self.groceries[grocery_index], ProximitySensor('success0')), NothingGrasped(self.robot.gripper)
        ])
        
        ### additional success conditions
        # put other peppers in cupboard
        other_peppers_conditions = [DetectedCondition(self.peppers[i], ProximitySensor('success0')) for i in range(len(self.peppers)) if i != self.target_pepper_index]
        self.register_success_conditions(self.get_success_conditions() + other_peppers_conditions)

        return ['weigh the %s pepper' % pepper[index],
                'pick up the %s pepper and set it down on the weighing scales'
                % pepper[index],
                'put the %s pepper on the scales' % pepper[index],
                'place the %s pepper onto the scales tray' % pepper[index],
                'lift up the %s pepper onto the weighing scales'
                % pepper[index],
                'grasp the %s pepper from above and lower it onto the tray'
                % pepper[index]]

    def variation_count(self) -> int:
        return UNIQUE_PEPPERS_TO_PLACE

    def step(self):
        _, _, pos_z = self.top_plate.get_position()
        dz = self.top_plate_starting_z - pos_z
        d_alpha = -120 * dz
        new_needle_ori = [self.needle_starting_ori[0] + d_alpha,
                          self.needle_starting_ori[1],
                          self.needle_starting_ori[2]]

        self.needle.set_orientation(new_needle_ori,
                                    relative_to=self.needle_pivot,
                                    reset_dynamics=False)

    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float],
                                            Tuple[float, float, float]]:
        return (0.0, 0.0, -1.), (0.0, 0.0, 1.)
    
    # def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
    #     return [0.0, 0.0, -0.25 * np.pi], [0.0, 0.0, +0.25 * np.pi]
    
    # for stack_wine task
    # def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
    #     return [0, 0, -np.pi / 4.], [0, 0, np.pi / 4.]