from typing import List, Tuple
from rlbench.backend.task import Task
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.conditions import JointCondition

class TidyUpDeskVer2(Task):

    def init_task(self) -> None:
        # Task 1: unplug_charger
        charger_success = ProximitySensor('charger_success')
        charger = Shape('charger')

        # Task 2: phone_on_base
        phone = Shape('phone')

        # Task 3: close_laptop_lid
        lid_joint_cond = JointCondition(Joint('joint'), 1.79)

        # register objects and success conditions
        self.register_graspable_objects([charger, phone])
        self.register_success_conditions([
            DetectedCondition(charger, charger_success),
            DetectedCondition(phone, ProximitySensor('success')),
            NothingGrasped(self.robot.gripper),
            lid_joint_cond,
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

        close_laptop_lid = [
            'close laptop lid',
            'close the laptop',
            'shut the laptop lid'
        ]

    
        return unplug_charger + phone_on_base + close_laptop_lid

    def variation_count(self) -> int:
        return 1
    
    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -3.14 / 4.], [0, 0, 3.14 / 4.]
    
