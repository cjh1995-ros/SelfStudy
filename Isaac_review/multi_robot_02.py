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


class RobotsPlaying(BaseTask):
    # Adding offset to move the task objects with and the targets..etc
    def __init__(self, name, offset=None):
        super().__init__(name=name, offset=offset)
        self._task_event = 0
        self._jetbot_goal_position = np.array([np.random.uniform(120,160), 30, 0]) + self._offset #defined in the base task
        self._pick_place_task = PickPlace(cube_initial_position=np.array([10, 30, 5]),
                                        target_position=np.array([70, -30, 5.15 / 2.0]),
                                        offset=offset)
        return

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        #This will already translate the pick and place assets by the offset
        self._pick_place_task.set_up_scene(scene)
        # Find a unique scene name
        jetbot_name = find_unique_string_name(
            intitial_name="fancy_jetbot", is_unique_fn=lambda x: not self.scene.object_exists(x)
        )
        # Find a unique prim path
        jetbot_prim_path = find_unique_string_name(
            intitial_name="/World/Fancy_Jetbot", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        self._jetbot = scene.add(Jetbot(prim_path=jetbot_prim_path,
                                        name=jetbot_name,
                                        position=np.array([0, 30, 0])))
        # Add Jetbot to this task objects
        self._task_objects[self._jetbot.name] = self._jetbot
        pick_place_params = self._pick_place_task.get_params()
        self._franka = scene.get_object(pick_place_params["robot_name"]["value"])
        # translate the franka by 100 in the x direction
        current_position, _ = self._franka.get_world_pose()
        self._franka.set_world_pose(position=current_position + np.array([100, 0, 0]))
        self._franka.set_default_state(position=current_position + np.array([100, 0, 0]))
        # This will only translate the task_objects by the offset specified (defined in the BaseTask)
        # Note: PickPlace task objects were already translated when setting up its scene
        self._move_task_objects_to_their_frame()
        return

    def get_observations(self):
        current_jetbot_position, current_jetbot_orientation = self._jetbot.get_world_pose()
        observations= {
            self.name + "_event": self._task_event, # change task assets to make it unique
            self._jetbot.name: {
                "position": current_jetbot_position,
                "orientation": current_jetbot_orientation,
                "goal_position": self._jetbot_goal_position
            }
        }
        observations.update(self._pick_place_task.get_observations())
        return observations

    def get_params(self):
        pick_place_params = self._pick_place_task.get_params()
        params_representation = pick_place_params
        params_representation["jetbot_name"] = {"value": self._jetbot.name, "modifiable": False}
        params_representation["franka_name"] = pick_place_params["robot_name"]
        return params_representation


    def pre_step(self, control_index, simulation_time):
        if self._task_event == 0:
            current_jetbot_position, _ = self._jetbot.get_world_pose()
            if np.mean(np.abs(current_jetbot_position[:2] - self._jetbot_goal_position[:2])) < 4:
                self._task_event += 1
                self._cube_arrive_step_index = control_index
        elif self._task_event == 1:
            if control_index - self._cube_arrive_step_index == 200:
                self._task_event += 1
        return

    def post_reset(self):
        self._task_event = 0
        return


class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self._tasks = []
        self._num_of_tasks = 3
        self._franka_controllers = []
        self._jetbot_controllers = []
        self._jetbots = []
        self._frankas = []
        self._cube_names = []
        return

    def setup_scene(self):
        world = self.get_world()
        for i in range(self._num_of_tasks):
            world.add_task(RobotsPlaying(name="my_awesome_task_"+str(i), offset=100 * np.array([0, (i*2) - 3, 0])))
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        for i in range(self._num_of_tasks):
            self._tasks.append(self._world.get_task(name="my_awesome_task_"+str(i)))
            task_params = self._tasks[i].get_params()
            self._frankas.append(self._world.scene.get_object(task_params[i]["franka_name"]["value"]))
            self._jetbots.append(self._world.scene.get_object(task_params[i]["jetbot_name"]["value"]))
            self._cube_names.append(task_params[i]["cube_name"]["value"])
            self._franka_controllers.append(PickPlaceController(
                name="pick_place_controller",
                gripper_dof_indices=self._frankas[i].gripper.dof_indices,
                robot_prim_path=self._frankas[i].prim_path,
                events_dt=[0.008, 0.002, 0.5, 0.1, 0.05, 0.05, 0.0025, 1, 0.008, 0.08]
            ))
            self._jetbot_controllers.append(WheelBasePoseController(
                name="cool_controllers",
                open_loop_wheel_controller=DifferentialController(name="open_loop_controller")
            ))
            self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)

        await self._world.play_async()
        return

    async def setup_post_reset(self):
        for i in range(self._num_of_tasks):
            self._franka_controllers[i].reset()
            self._jetbot_controllers[i].reset()
        await self._world.play_async()
        return

    def physics_step(self, step_size):
        current_observations = self._world.get_observations()
        for i in range(len(self._tasks)):
            # Apply actions for each task
            if current_observations[self._tasks[i].name + "_event"] == 0:
                self._jetbots[i].apply_wheel_actions(
                    self._jetbot_controllers[i].forward(
                        start_position=current_observations[self._jetbots[i].name]["position"],
                        start_orientation=current_observations[self._jetbots[i].name]["orientation"],
                        goal_position=current_observations[self._jetbots[i].name]["goal_position"]))
            elif current_observations[self._tasks[i].name + "_event"] == 1:
                self._jetbots[i].apply_wheel_actions(ArticulationAction(joint_velocities=[-10, -10]))
            elif current_observations[self._tasks[i].name + "_event"] == 2:
                self._jetbots[i].apply_wheel_actions(ArticulationAction(joint_velocities=[0.0, 0.0]))
                actions = self._franka_controllers[i].forward(
                    picking_position=current_observations[self._cube_names[i]]["position"],
                    placing_position=current_observations[self._cube_names[i]]["target_position"],
                    current_joint_positions=current_observations[self._frankas[i].name]["joint_positions"])
                self._frankas[i].apply_action(actions)
        return

    def world_cleanup(self):
        self._tasks = []
        self._num_of_tasks = 3
        self._franka_controllers = []
        self._jetbot_controllers = []
        self._jetbots = []
        self._frankas = []
        self._cube_names = []

        return 
        
        
        
2022-02-11 11:52:54 [1,029,061ms] [Error] [asyncio] [/home/mint-lab/.local/share/ov/pkg/isaac_sim-2021.2.1/kit/python/lib/python3.7/asyncio/base_events.py:1619] Task exception was never retrieved
future: <Task finished coro=<BaseSampleExtension._on_load_world.<locals>._on_load_world_async() done, defined at /home/mint-lab/.local/share/ov/pkg/isaac_sim-2021.2.1/exts/omni.isaac.examples/omni/isaac/examples/base_sample/base_sample_extension.py:161> exception=KeyError(0)>
Traceback (most recent call last):
  File "/home/mint-lab/.local/share/ov/pkg/isaac_sim-2021.2.1/exts/omni.isaac.examples/omni/isaac/examples/base_sample/base_sample_extension.py", line 162, in _on_load_world_async
    await self._sample.load_world_async()
  File "/home/mint-lab/.local/share/ov/pkg/isaac_sim-2021.2.1/exts/omni.isaac.examples/omni/isaac/examples/base_sample/base_sample.py", line 50, in load_world_async
    await self.setup_post_load()
  File "/home/mint-lab/.local/share/ov/pkg/isaac_sim-2021.2.1/exts/omni.isaac.examples/omni/isaac/examples/user_examples/hello_world.py", line 115, in setup_post_load
    self._frankas.append(self._world.scene.get_object(task_params[i]["franka_name"]["value"]))
KeyError: 0

