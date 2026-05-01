from typing import List
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, ConditionSet, \
    GraspedCondition, JointCondition
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
from rlbench.backend.spawn_boundary import SpawnBoundary
from pyrep.objects.proximity_sensor import ProximitySensor

from rlbench.const import colors
import numpy as np

class LampOnAndPickAndLift(Task):

    def init_task(self) -> None:
        ### lamp_on
        self.bulb_glass_visual = Shape('bulb')
        self.bulb_glass_visual.set_color([0, 0, 0])
        self.joint = Joint('target_button_joint')
        self.condition = JointCondition(self.joint, 0.003)
        
        ### pick_and_lift
        self.target_block = Shape('pick_and_lift_target')
        self.distractors = [
            Shape('stack_blocks_distractor%d' % i)
            for i in range(2)]
        self.register_graspable_objects([self.target_block])
        self.boundary = SpawnBoundary([Shape('pick_and_lift_boundary')])
        self.success_detector = ProximitySensor('pick_and_lift_success')

    def init_episode(self, index: int) -> List[str]:
        ### lamp_on
        self.bulb_glass_visual.set_color([0, 0, 0])
        
        ### pick_and_lift
        block_color_name, block_rgb = colors[index]
        self.target_block.set_color(block_rgb)

        color_choices = np.random.choice(
            list(range(index)) + list(range(index + 1, len(colors))),
            size=2, replace=False)
        for i, ob in enumerate(self.distractors):
            name, rgb = colors[color_choices[int(i)]]
            ob.set_color(rgb)

        self.boundary.clear()
        self.boundary.sample(
            self.success_detector, min_rotation=(0.0, 0.0, 0.0),
            max_rotation=(0.0, 0.0, 0.0))
        for block in [self.target_block] + self.distractors:
            self.boundary.sample(block, min_distance=0.1)
        
        ### Set success conditions and descriptions
        print("index: ", index)
        if index == 0:
            self.register_success_conditions([self.condition])
            
            return ['turn on the light',
                    'press the button to turn on the lamp',
                    'press the light switch',
                    'turn the lamp on',
                    'close the gripper and press on the button until the light '
                    'turns on']
        else:
            cond_set = ConditionSet([
                GraspedCondition(self.robot.gripper, self.target_block),
                DetectedCondition(self.target_block, self.success_detector)
            ])
            self.register_success_conditions([cond_set])
            return ['pick up the %s block and lift it up to the target' %
                block_color_name,
                'grasp the %s block to the target' % block_color_name,
                'lift the %s block up to the target' % block_color_name]

    def variation_count(self) -> int:
        return 2

    def step(self) -> None:
        if self.condition.condition_met()[0]:
            self.bulb_glass_visual.set_color([1, 1, 1])
    
    def get_low_dim_state(self) -> np.ndarray:
        # One of the few tasks that have a custom low_dim_state function.
        return np.concatenate([self.target_block.get_position(), self.success_detector.get_position()], 0)

