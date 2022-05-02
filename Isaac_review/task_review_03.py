from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.franka.tasks import PickPlace
from omni.isaac.franka.controllers import PickPlaceController
from omni.isaac.jetbot import Jetbot
from omni.isaac.motion_generation import WheelBasePoseController
from omni.isaac.jetbot.controllers import DifferentialController
from omni.isaac.core.tasks import BaseTask
import numpy as np


class RobotsPlaying(BaseTask):
    def __init__(
        self,
        name
    ):
        super().__init__(name=name, offset=None)
        self._jetbot_goal_position = np.array([130, 30, 0])
        self._pick_place_task = PickPlace(cube_initial_position=np.array([10, 30, 5]),
                                        target_position=np.array([70, -30, 5.15 / 2.0]))
        self._task_event = 0
        return

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._pick_place_task.set_up_scene(scene)
        self._jetbot = scene.add(Jetbot(prim_path="/World/Fancy_Jetbot",
                                        name="fancy_jetbot",
                                        position=np.array([0, 30, 0])))
        pick_place_params = self._pick_place_task.get_params()
        self._franka = scene.get_object(pick_place_params["robot_name"]["value"])
        self._cube = scene.get_object(pick_place_params["cube_name"]["value"])
        self._franka.set_world_pose(position=np.array([100, 0, 0]))
        self._franka.set_default_state(position=np.array([100, 0, 0]))
        return

    def get_observations(self):
        current_jetbot_position, current_jetbot_orientation = self._jetbot.get_world_pose()
        observations= {
            "task_event": self._task_event, 
            self._jetbot.name: {
                "position": current_jetbot_position,
                "orientation": current_jetbot_orientation,
                "goal_position": self._jetbot_goal_position
            }
        }
        # dictionary update는 dict을 병합하는 것이다. 
        observations.update(self._pick_place_task.get_observations())
        # 즉 pick place task 까지 관찰 가능하다는 것.
        return observations

    def get_params(self):
        pick_place_params = self._pick_place_task.get_params()
        params_representation = pick_place_params
        params_representation["jetbot_name"] = {"value": self._jetbot.name, "modifiable": False}
        params_representation["franka_name"] = pick_place_params["robot_name"]
        
        return params_representation

    def pre_step(self, control_index, simulation_time):
        # print("event_stage now: ", self._task_event)
        current_cube_position, _ = self._cube.get_world_pose()
        cube_name = self._pick_place_task.get_params()["cube_name"]["value"]
        cube_target_position = self.get_observations()[cube_name]["target_position"]
        if self._task_event == 0:
            current_jetbot_position, _ = self._jetbot.get_world_pose()
            if np.mean(np.abs(current_jetbot_position[:2] - self._jetbot_goal_position[:2])) < 4 :
                self._task_event = 1
                self._cube_arrive_step_index = control_index
        elif self._task_event == 1:
            # 이 event에 도달하면 젯봇은 뒤로 200바퀴 돌아갈 것임.
            if control_index - self._cube_arrive_step_index == 200:
                self._task_event += 1

        elif self._task_event == 2 and np.mean(np.abs(cube_target_position - current_cube_position)) < 2:
            # Franka가 움직여야함.
            self._cube.get_applied_visual_material().set_color(color=np.array([1, 0, 0]))

        return

    def post_reset(self):
        self._task_event = 0
        return 

class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        world.add_task(RobotsPlaying(name="awesome_task"))
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        task_params = self._world.get_task("awesome_task").get_params()
        self._jetbot = self._world.scene.get_object(task_params["jetbot_name"]["value"])
        self._franka = self._world.scene.get_object(task_params["franka_name"]["value"])
        self._cube_name = task_params["cube_name"]["value"]
        self._franka_controller = PickPlaceController(
            name="franka_controller",
            gripper_dof_indices=self._franka.gripper.dof_indices,
            robot_prim_path=self._franka.prim_path,
        )
        self._jetbot_controller = WheelBasePoseController(name="cool_controller",
                                                        open_loop_wheel_controller=DifferentialController(name="open_loop_controller"))
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        await self._world.play_async()
        return

    async def setup_post_reset(self):
        self._franka_controller.reset()
        self._jetbot_controller.reset()
        await self._world.play_async()
        return

    def physics_step(self, step_size):
        current_observations = self._world.get_observations()
        if current_observations["task_event"] == 0:
            self._jetbot.apply_wheel_actions(
                self._jetbot_controller.forward(
                    start_position=current_observations[self._jetbot.name]["position"],
                    start_orientation=current_observations[self._jetbot.name]["orientation"],
                    goal_position=current_observations[self._jetbot.name]["goal_position"]))
        elif current_observations["task_event"] == 1:
            self._jetbot.apply_wheel_actions(
                ArticulationAction(
                    joint_velocities=np.array([-10, -10])
                )
            )
        elif current_observations["task_event"] == 2:
            self._jetbot.apply_wheel_actions(
                ArticulationAction(
                    joint_velocities=[0,0]
                )
            )
            actions = self._franka_controller.forward(
                picking_position=current_observations[self._cube_name]["position"],
                placing_position=current_observations[self._cube_name]["target_position"],
                current_joint_positions=current_observations[self._franka.name]["joint_positions"]
            )        
            self._franka.apply_action(actions)
        if self._franka_controller.is_done():
            current_observations["task_event"] = 3
            self._world.pause()

        return
