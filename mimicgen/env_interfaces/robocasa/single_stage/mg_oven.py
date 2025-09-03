from mimicgen.env_interfaces.robocasa.single_stage.mg_drawer import MG_SlideRack
from robosuite.utils.mjcf_utils import find_elements
from mimicgen.env_interfaces.robosuite import RobosuiteInterface


class MG_SlideOvenRack(MG_SlideRack):
    def get_rack_name(self):
        if self.env.oven.has_multiple_rack_levels():
            rack_level = self.env.rack_level
        else:
            rack_level = 0
        return self.env.oven.naming_prefix + f"rack{rack_level}"

class MG_PreheatOven(RobosuiteInterface):
    
    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        
        return dict(
            knob=self.get_object_pose(obj_name=self.env.oven.name + "_knob_temp_main", obj_type="geom"),
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
        knob_body = find_elements(
            self.env.oven.worldbody,
            tags="body",
            attribs={"name": "{}_knob_temp".format(self.env.oven.name)},
            return_first=True,
        )
        knob_geoms = find_elements(knob_body, tags="geom", return_first=False)
        knob_geom_names = [e.get("name") for e in knob_geoms]
        check_contact = self.env.check_contact(
            self.env.robots[0].gripper["right"],
            knob_geom_names,
        )
        signals["stage_contact_knob"] = int(check_contact)
        signals["success"] = int(self.env._check_success())

        return signals