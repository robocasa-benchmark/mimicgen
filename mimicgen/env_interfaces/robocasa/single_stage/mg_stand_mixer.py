from mimicgen.env_interfaces.robosuite import RobosuiteInterface
from robosuite.utils.mjcf_utils import find_elements

    
class MG_StandMixerHead(RobosuiteInterface):
    def get_object_poses(self):
            """
            Gets the pose of each object relevant to MimicGen data generation in the current scene.

            Returns:
                object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
            """
            return dict(
                head=self.get_object_pose(obj_name=f"{self.env.stand_mixer.naming_prefix}head", obj_type="body"),
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

        head_body = find_elements(self.env.stand_mixer.worldbody, "body", 
                                  attribs={"name": f"{self.env.stand_mixer.naming_prefix}head"},
                                  return_first=True)
        assert head_body is not None, "No head body found in stand mixer"
        head_geoms = find_elements(head_body, tags="geom", return_first=False)
        head_geom_names = [e.get("name") for e in head_geoms]
        contact_door = self.env.check_contact(
            self.env.robots[0].gripper["right"],
            head_geom_names
        )

        signals["stage_contact_head"] = int(contact_door)
        signals["success"] = int(self.env._check_success())

        return signals

class MG_CloseStandMixerHead(MG_StandMixerHead):
    pass

class MG_OpenStandMixerHead(MG_StandMixerHead):
    pass