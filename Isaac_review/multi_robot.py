from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.franka.tasks import PickPlace
from omni.isaac.franka.controllers import PickPlaceController
from omni.isaac.jetbot import Jetbot
from omni.isaac.motion_generation import WheelBasePoseController
from omni.isaac.jetbot.controllers import DifferentialController
from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.utils.types import ArticulationAction
# Find a unique string name, to use it for prim paths and scene names
from omni.isaac.core.utils.string import find_unique_string_name        # Creates a unique prim path
from omni.isaac.core.utils.prims import is_prim_path_valid              # Checks if a prim path is valid
from omni.isaac.core.objects.cuboid import VisualCuboid
import numpy as np

class RobotPlaying(BaseTask):
    def __init__(self, name, offset=None):
        super().__init__(name=name, offset=offset)
        self._task_event = 0
        # offset parameter는 task의 모든 assets를 이동시킬 수 있게 해준다.
        self._jetbot_goal_position = np.array([130, 30, 0]) + self._offset
        self._pick_place_task = PickPlace(
            cube_initial_position=np.array([10, 30, 5]),
            target_position=np.array([70, -30, 5.15 / 2.0]),
            offset=offset
        )
        return 

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        # offset 반영된 scene
        self._pick_place_task.set_up_scene(scene)
        # find a unique scene name
        jetbot_name = find_unique_string_name(
            intitial_name="fancy_jetbot",
            is_unique_fn=lambda x : self.scene.object_exists(x)
        )
        jetbot_prim_path = find_unique_string_name(
            intitial_name="/World/Fancy_Jetbot",
            is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        self._jetbot = scene.add(
            Jetbot(
                prim_path=jetbot_prim_path,
                name=jetbot_name,
                position=np.array([0, 30, 0])
            )
        )
        # Add Jetbot to this task object
        self._task_objects[self._jetbot.name] = self._jetbot
        pick_place_params = self._pick_place_task.get_params()
        self._franka = scene.get_object([pick_place_params["robot_name"]["value"]])
        # franka 로봇을 x 방향으로 100 이동 시키기.
        current_position, _ = self._franka.get_world_pose()
        self._franka.set_world_pose(position=current_position + np.array([100, 0, 0]))
        self._franka.set_default_state(position=current_position + np.array([100, 0, 0]))
        # BaseTask에서 정의된 만큼 offset할 것이다. 
        self._move_task_objects_to_their_frame()
        return 

    def get_observations(self):
        current_jetbot_position, current_jetbot_orientation = self._jetbot.get_world_pose()
        observations = {
            "task_event": self._task_event,
            self._jetbot.name: {
                "position": current_jetbot_position,
                "orientation": current_jetbot_orientation,
                "goal_position": self._jetbot_goal_position,
            }
        }
        observations.update(self._pick_place_task.get_observations())
        return observations

    def get_params(self):
        pick_place_params = self._pick_place_task.get_params()
        params_representation = pick_place_params
        params_representation["jetbot_name"] = {"value": self._jetbot.name}
        params_representation["franka_name"] = pick_place_params["robot_name"]
        return params_representation

    def pre_step(self, control_index, simulation_time):
        if self._task_event == 0:
            current_jetbot_position, _ = self._jetbot.get_world_pose()
            if np.mean(np.abs(current_jetbot_position[:2] - self._jetbot_goal_position[:2])) < 4:
                self._task_event = 1
                self._cube_arrive_step_index = control_index
        elif self._task_event == 1:
            if control_index - self._cube_arrive_step_index == 200:
                self._task_event += 1
        return 

class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        world.add_task(RobotPlaying(name="awesome_task", offset=np.array([0, -100, 0])))
        VisualCuboid(
            prim_path="/new_cube_1",
            position=np.array([100, 0, 5]),
            size=np.array([10, 10, 10])
        )
        return 

    async def setup_post_load(self):
        self._world = self.get_world()
        task_params = self._world.get_task("awesome_task").get_params()
        self._jetbot = self._world.scene.get_object(task_params["jetbot_name"]["value"])
        self._franka = self._world.scene.get_object(task_params["franka_name"]["value"])
        self.cube_name = task_params["cube_name"]["value"]
        self._franka_controller = PickPlaceController(
            name="pick_place_controller",
            gripper_dof_indices=self._franka.gripper.dof_indices,
            robot_prim_path=self._franka.prim_path,
        )
        self._jetbot_controller = WheelBasePoseController(
            name="jetbot_controller",
            open_loop_wheel_controller=DifferentialController(name="open_loop_controller")
        )
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
                    goal_position=current_observations[self._jetbot.name]["goal_position"],
                )
            )
        elif current_observations["task_event"] == 1:
            self._jetbot.apply_wheel_actions(
                ArticulationAction(
                    joint_velocities=np.array([-10,-10])
                )
            )
        elif current_observations["task_event"] == 2:
            self._jetbot.apply_wheel_actions(
                ArticulationAction(
                    joint_velocities=np.array([0,0])
                )
            )
            action = self._franka_controller.forward(
                picking_position=current_observations["cube_name"]["position"],
                placing_position=current_observations["cube_name"]["target_position"],
                current_joint_positions=current_observations[self._franka.name]["joint_positions"]
            )
            self._franka.apply_action(actions)
        if self._franka_controller.is_done():
            self._world.pause()

        return
