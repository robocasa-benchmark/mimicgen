from mimicgen.env_interfaces.robosuite import RobosuiteInterface
from robosuite.utils.mjcf_utils import find_elements

class MG_OpenElectricKettleLid(RobosuiteInterface):

    def get_object_poses(self):
        return dict(
            button=self.get_object_pose(obj_name=f"{self.env.electric_kettle.naming_prefix}button_lid_main", obj_type="geom"),
        )
    
    def get_subtask_term_signals(self):
        signals = dict()
        button_geom_name = f"{self.env.electric_kettle.naming_prefix}button_lid_main"
        button_contact = self.env.check_contact(
            self.env.robots[0].gripper["right"],
            button_geom_name,
        )
        signals["stage_contact_button"] = int(button_contact)
        signals["success"] = int(self.env._check_success())
        return signals

class MG_CloseElectricKettleLid(RobosuiteInterface):

    def get_object_poses(self):
        return dict(
            lid=self.get_object_pose(obj_name=f"{self.env.electric_kettle.naming_prefix}lid_main", obj_type="geom"),
        )
    
    def get_subtask_term_signals(self):
        signals = dict()
        lid_geom_name = f"{self.env.electric_kettle.naming_prefix}lid_main"
        lid_contact = self.env.check_contact(
            self.env.robots[0].gripper["right"],
            lid_geom_name,
        )
        signals["stage_contact_lid"] = int(lid_contact)
        signals["success"] = int(self.env._check_success())
        return signals

class MG_TurnOnElectricKettle(RobosuiteInterface):

    def get_object_poses(self):
        return dict(
            switch=self.get_object_pose(obj_name=f"{self.env.electric_kettle.naming_prefix}switch_main", obj_type="geom"),
        )

    def get_subtask_term_signals(self):
        signals = dict()
        switch_geom_name = f"{self.env.electric_kettle.naming_prefix}switch_main"
        switch_contact = self.env.check_contact(
            self.env.robots[0].gripper["right"],
            switch_geom_name,
        )
        signals["stage_contact_switch"] = int(switch_contact)
        signals["success"] = int(self.env._check_success())

        return signals
    
    