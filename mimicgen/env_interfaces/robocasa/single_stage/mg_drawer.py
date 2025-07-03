from mimicgen.env_interfaces.robosuite import RobosuiteInterface

import numpy as np
from robosuite.utils.mjcf_utils import find_elements


class MG_OpenDrawer(RobosuiteInterface):

    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        return dict(
            handle=self.get_object_pose(obj_name=f"{self.env.drawer.name}_door_handle_reg_main", obj_type="geom"),
        )

    def get_subtask_term_signals(self):
        """
        Gets a dictionary of binary flags for each subtask in a task. The flag is 1
        when the subtask has been completed and 0 otherwise. MimicGen only uses this
        when parsing source demonstrations at the start of data generation, and it only
        uses the first 0 -> 1 transition in this signal to detect the end of a subtask.

        Returns:
            subtask_term_signals (dict): dictionary that maps subtask name to termination flag (0 or 1)
        """
        signals = dict()
        handle_body = find_elements(self.env.drawer.worldbody, tags="body", attribs={"name": f"{self.env.drawer.naming_prefix}door_handle_main"})
        drawer_handle_geoms = find_elements(handle_body, tags="geom", return_first=False)
        drawer_handle_geom_names = [e.get("name") for e in drawer_handle_geoms]
        contact_handle = self.env.check_contact(
            self.env.robots[0].gripper["right"],
            drawer_handle_geom_names
        )
        signals["stage_contact_handle"] = int(contact_handle)
        signals["success"] = int(self.env._check_success())
        return signals


class MG_CloseDrawer(RobosuiteInterface):

    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        return dict(
            handle=self.get_object_pose(obj_name=f"{self.env.drawer.name}_door_handle_reg_main", obj_type="geom"),
        )

    def get_subtask_term_signals(self):
        """
        Gets a dictionary of binary flags for each subtask in a task. The flag is 1
        when the subtask has been completed and 0 otherwise. MimicGen only uses this
        when parsing source demonstrations at the start of data generation, and it only
        uses the first 0 -> 1 transition in this signal to detect the end of a subtask.

        Returns:
            subtask_term_signals (dict): dictionary that maps subtask name to termination flag (0 or 1)
        """
        signals = dict()
        drawer_geoms = find_elements(self.env.drawer.worldbody, tags="geom", return_first=False)
        drawer_geom_names = [e.get("name") for e in drawer_geoms]
        contact_drawer = self.env.check_contact(
            self.env.robots[0].gripper["right"],
            drawer_geom_names,
        )
        signals["stage_contact_drawer"] = int(contact_drawer)
        signals["success"] = int(self.env._check_success())
        return signals