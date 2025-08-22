from mimicgen.env_interfaces.robosuite import RobosuiteInterface

import numpy as np
from robosuite.utils.mjcf_utils import find_elements
from robocasa.models.fixtures import Fixture


class MG_ManipulateBlenderLid(RobosuiteInterface):

    def get_lid_handle_name(self):
        return f"{self.env.blender.blender_lid.naming_prefix}handle_main"

    def get_object_poses(self):
        
        return dict(
            lid_handle=self.get_object_pose(obj_name=self.get_lid_handle_name(), obj_type="geom"),
        )

    def get_subtask_term_signals(self):
        signals = dict()
        contact_handle = self.env.check_contact(
            self.env.robots[0].gripper["right"],
            self.get_lid_handle_name(),
        )
        signals["stage_contact_lid_handle"] = int(contact_handle)
        signals["success"] = int(self.env._check_success())
        return signals

class MG_OpenBlenderLid(MG_ManipulateBlenderLid):
    pass

class MG_CloseBlenderLid(MG_ManipulateBlenderLid):
    pass

class MG_TurnOnBlender(RobosuiteInterface):

    def get_object_poses(self):
        return dict(
            button=self.get_object_pose(obj_name=self.env.blender.name + "_power_button_main", obj_type="geom"),
        )

    def get_subtask_term_signals(self):
        signals = dict()
        signals["success"] = int(self.env._check_success())
        return signals


