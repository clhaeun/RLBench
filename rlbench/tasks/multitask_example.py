from typing import List
from rlbench.backend.task import Task
from rlbench.backend.conditions import JointCondition
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
from rlbench.backend.spawn_boundary import SpawnBoundary

class MultitaskExample(Task):

    def init_task(self) -> None:
        # self.boundary_task0 = SpawnBoundary([Shape('task_area0')])
        self.bulb_glass_visual0 = Shape('bulb0')
        self.bulb_glass_visual0.set_color([1, 1, 1])
        self.joint0 = Joint('target_button_joint0')
        self.condition0 = JointCondition(self.joint0, 0.003)

        self.is_static_workspace = lambda: True

    def init_episode(self, index: int) -> List[str]:
        # self.boundary_task0.clear()
        self.bulb_glass_visual0.set_color([1, 1, 1])
        self.register_success_conditions([self.condition0])
        return ['']

    def variation_count(self) -> int:
        # TODO: The number of variations for this task.
        return 1

    def step(self) -> None:
        if self.condition0.condition_met() == (True, True):
            self.bulb_glass_visual0.set_color([0, 0, 0])
