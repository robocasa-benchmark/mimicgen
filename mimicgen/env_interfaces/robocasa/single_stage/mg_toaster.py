from mimicgen.env_interfaces.robosuite import RobosuiteInterface
from robosuite.utils.mjcf_utils import find_elements


class MG_TurnOnToaster(RobosuiteInterface):
    def _slot_pair_side(self, slot_pair: int) -> str:
        """
        Map a slot_pair index to a human-readable side label.        
        """
        if slot_pair not in self.env.toaster._slot_pairs:
            raise ValueError(f"slot_pair must be one of {self.env.toaster._slot_pairs}")

        # Look at the lever joint name we already stored
        jn = self.env.toaster._joint_names.get(f"lever_{slot_pair}", "")
        if "sideL" in jn:
            return "sideL_"
        if "sideR" in jn:
            return "sideR_"

        return ""
    
    def _get_lever_geom_name(self) -> str:
        """
        Get the lever geom name for the task.
        """
        toast_slot = 0
        for slot_pair in range(len(self.env.toaster.get_state(self).keys())):
            if self.env.toaster.check_slot_contact(self.env, "obj", slot_pair):
                toast_slot = slot_pair
                break
        side = self._slot_pair_side(toast_slot)
        return f"{self.env.toaster.name}_{side}lever_handle"
    
    
    def get_object_poses(self):
        lever_geom = self._get_lever_geom_name()
        return dict(
            lever=self.get_object_pose(obj_name=lever_geom, obj_type="geom"),
        )

    def get_subtask_term_signals(self):
        signals = dict()
        lever_geom = self._get_lever_geom_name()
        check_contact = self.env.check_contact(
            self.env.robots[0].gripper["right"],
            [lever_geom],
        )
        signals["stage_contact_lever"] = int(check_contact)
        signals["success"] = int(self.env._check_success())

        return signals
    