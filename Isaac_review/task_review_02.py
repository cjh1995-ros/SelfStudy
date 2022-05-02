from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.franka.franka import Franka
from omni.isaac.franka.tasks import PickPlace
from omni.isaac.jetbot import Jetbot
from omni.isaac.motion_generation import WheelBasePoseController
from omni.isaac.jetbot.controllers import DifferentialController
from omni.isaac.core.tasks import BaseTask, pick_place
import numpy as np

class RobotsPlaying(BaseTask):
    def __init__(
        self,
        name
    ):
        super().__init__(name=name, offset=None)
        self._jetbot_goal_position = np.array([130, 30, 0])
        self._pick_place_task = PickPlace(
            cube_initial_orientation=np.array([10, 30, 5]),
            target_position=np.array([70, -30, 5.15 / 2.0]),
        )
        return

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._pick_place_task.set_up_scene(scene)
        self._jetbot = scene.add(
            Jetbot(
                prim_path="/World/Fancy_Jetbot",
                name="fancy_jetbot",
                position=np.array([0, 30, 0]),
            )
        )
        pick_place_params = self._pick_place_task.get_params()
        self._franka = scene.get_object(pick_place_params["robot_name"]["value"])
        self._franka.set_world_pose(position=np.array([100, 0, 0]))
        self._franka.set_default_state(position=np.array([100, 0, 0]))
        return 

    def get_observations(self):
        current_jetbot_position, current_jetbot_orientation = self._jetbot.get_world_pose()
        observations = {
            self._jetbot.name : {
                "position": current_jetbot_position,
                "orientation": current_jetbot_orientation,
                "goal_position": self._jetbot_goal_position,
            },
        }
        return observations

    def get_params(self):
        pick_place_params = self._pick_place_task.get_params()
        params_representation = pick_place_params
        params_representation["jetbot_name"]={
            "value": self._jetbot.name,
            "modifiable": False,
        }
        params_representation["franka_name"] = pick_place_params["robot_name"]
        return params_representation

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
        self._cube_name = task_params["cube_name"]["value"]
        self._jetbot_controller = WheelBasePoseController(
            name="cool_controller",
            open_loop_wheel_controller=DifferentialController(name="open_loop_controller")
        )
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)

        await self._world.play_async()
        return 

    async def setup_post_reset(self):
        self._jetbot_controller.reset()
        await self._world.play_async()
        return

    def physics_step(self, step_size):
        current_observations = self._world.get_observations()
        self._jetbot.apply_wheel_actions(
            self._jetbot_controller.forward(
                start_position=current_observations[self._jetbot.name]["position"],
                start_orientation=current_observations[self._jetbot.name]["orientation"],
                goal_position=current_observations[self._jetbot.name]["goal_position"]
            )
        )
        return
