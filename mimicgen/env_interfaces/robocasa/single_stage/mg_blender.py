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
        lid_body = find_elements(
            self.env.model.worldbody,
            tags="body",
            attribs={"name": self.env.blender.blender_lid.naming_prefix + "main"},
            return_first=True,
        )
        lid_geoms = find_elements(lid_body, tags="geom", return_first=False)
        lid_geom_names = [e.get("name") for e in lid_geoms]
        contact_lid = self.env.check_contact(
            self.env.robots[0].gripper["right"],
            lid_geom_names,
        )
        signals["stage_contact_lid"] = int(contact_lid)
        signals["success"] = int(self.env._check_success())
        return signals

class MG_OpenBlenderLid(MG_ManipulateBlenderLid):
    pass

class MG_CloseBlenderLid(MG_ManipulateBlenderLid):
    def get_object_poses(self):
        signals = super().get_object_poses()
        signals["blender"] = self.get_object_pose(f"{self.env.blender.naming_prefix}reg_main", obj_type="geom")
        return signals

class MG_TurnOnBlender(RobosuiteInterface):

    def get_object_poses(self):
        return dict(
            button=self.get_object_pose(obj_name=self.env.blender.name + "_power_button_main", obj_type="geom"),
        )

    def get_subtask_term_signals(self):
        signals = dict()
        signals["success"] = int(self.env._check_success())
        return signals


