from typing import List, Tuple
from rlbench.backend.task import Task
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.spawn_boundary import SpawnBoundary

SHAPE_NAMES = ['cube', 'cylinder', 'triangular prism', 'star', 'moon']

WAYPOINT_OFFSET = 9

class TidyUpDesk(Task):

    def init_task(self) -> None:
        # Task 1: unplug_charger
        charger_success = ProximitySensor('charger_success')
        charger = Shape('charger')

        # Task 2: phone_on_base
        phone = Shape('phone')

        # Task 3: place_shape_in_shape_sorter
        self.shape_sorter = Shape('shape_sorter')
        self.success_sensor = ProximitySensor('success')
        self.shapes = [Shape(ob.replace(' ', '_')) for ob in SHAPE_NAMES]
        self.drop_points = [
            Dummy('%s_drop_point' % ob.replace(' ', '_'))
            for ob in SHAPE_NAMES]
        self.grasp_points = [
            Dummy('%s_grasp_point' % ob.replace(' ', '_'))
            for ob in SHAPE_NAMES]
        self.waypoint10 = Dummy('waypoint10')
        self.waypoint13 = Dummy('waypoint13')
        self.register_graspable_objects(self.shapes)

        self.register_waypoint_ability_start(0 + WAYPOINT_OFFSET, self._set_grasp)
        self.register_waypoint_ability_start(3 + WAYPOINT_OFFSET, self._set_drop)
        self.boundary = SpawnBoundary([Shape('boundary')])

        # register objects and success conditions
        self.register_graspable_objects([charger, phone])
        self.register_success_conditions([
            DetectedCondition(charger, charger_success),
            DetectedCondition(phone, ProximitySensor('success')),
            NothingGrasped(self.robot.gripper),
        ])
        

    def init_episode(self, index: int) -> List[str]:
        unplug_charger = [
            'unplug charger',
            'take the charger out of the wall',
            'grip the black charger and pull it out of its socket',
            'slide the plug out from the wall',
            'remove the charger from the mains',
            'get the charger from the wall plug'
        ]
        
        phone_on_base = [
            'put the phone on the base',
            'put the phone on the stand',
            'put the hone on the hub',
            'grasp the phone and put it on the base',
            'place the phone on the base',
            'put the phone back on the base',
        ]

        self.variation_index = index
        shape = SHAPE_NAMES[index]
        self.register_success_conditions(
            [DetectedCondition(self.shapes[index], self.success_sensor)])

        self.boundary.clear()
        [self.boundary.sample(s, min_distance=0.05) for s in self.shapes]

        place_shape_in_shape_sorter = [
            'put the %s in the shape sorter' % shape,
            'pick up the %s and put it in the sorter' % shape,
            'place the %s into its slot in the shape sorter' % shape,
            'slot the %s into the shape sorter' % shape
        ]

        return unplug_charger + phone_on_base + place_shape_in_shape_sorter

    def variation_count(self) -> int:
        return len(SHAPE_NAMES)

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -3.14 / 4.], [0, 0, 3.14 / 4.]
    
    def _set_grasp(self, _):
        gp = self.grasp_points[self.variation_index]
        self.waypoint10.set_pose(gp.get_pose())

    def _set_drop(self, _):
        dp = self.drop_points[self.variation_index]
        self.waypoint13.set_pose(dp.get_pose())
