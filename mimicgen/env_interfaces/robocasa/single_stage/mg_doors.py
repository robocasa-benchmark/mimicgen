from mimicgen.env_interfaces.robosuite import RobosuiteInterface

import numpy as np
from robosuite.utils.mjcf_utils import find_elements


class MG_OpenSingleDoor(RobosuiteInterface):

    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        return dict(
            handle=self.get_object_pose(obj_name=f"{self.env.fxtr.naming_prefix}door_handle_main", obj_type="geom"),
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
        contact_handle = self.env.check_contact(
            self.env.robots[0].gripper["right"],
            f"{self.env.fxtr.naming_prefix}door_handle_main",
        )
        signals["stage_contact_handle"] = int(contact_handle)
        signals["success"] = int(self.env._check_success())
        return signals


class MG_OpenMultipleDoor(RobosuiteInterface):

    DYNAMIC_STAGE_INDS = set([0,1,2,3])

    def _get_handle_bodies(self):

        door_handle_bodies = []

        for body in self.env.fxtr.worldbody.findall(".//body"):
            name = body.attrib.get("name", "")
            if "door_handle_main" in name:
                door_handle_bodies.append(body)

        return door_handle_bodies
    
    def _get_handle_door_joint_names(self, handle_body):
        """
        Gets the door joint names associated with a handle body name.

        Args:
            handle_body (str): name of the handle body

        Returns:
            door_joint_names (list): list of door joint names associated with the handle body
        """
        def _find_parent(root, target):
            """
            Recursively find the parent of 'target' starting from 'root'.
            Returns the parent element if found, else None.
            """
            for child in root:
                if child is target:
                    return root
                parent = _find_parent(child, target)
                if parent is not None:
                    return parent
            return None
        door = _find_parent(self.env.fxtr.worldbody, _find_parent(self.env.fxtr.worldbody, handle_body))
        assert door is not None, "No door found in env for handle body {}".format(handle_body.get("name"))
        door_joint = find_elements(door, "joint")
        assert door_joint is not None, "No door joint found in env for handle body {}".format(door.get("name"))
        return door_joint.get("name")

    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        handle_names = self._get_handle_bodies()
        handle_names = [h.get("name") for h in handle_names]
        # TODO randomize order of task! by shuffling handle_names

        handle_1_pose = self.get_object_pose(obj_name=handle_names[0], obj_type="body") 
        handle_2_pose = self.get_object_pose(obj_name=handle_names[1], obj_type="body") if len(handle_names) > 1 else np.zeros_like(handle_1_pose)
        return dict(
            handle_1=handle_1_pose,
            handle_2=handle_2_pose,
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
        handle_bodies = self._get_handle_bodies()

        for door_num in [1, 2]:
            if door_num == 2 and len(handle_bodies) < 2:
                signals["stage_contact_handle_{}".format(door_num)] = -1
                signals["stage_open_door_{}".format(door_num)] = -1
                continue
            handle_geoms = find_elements(
                handle_bodies[door_num - 1],
                tags="geom",
                return_first=False
            )
            contact_handle = self.env.check_contact(
                self.env.robots[0].gripper["right"],
                [e.get("name") for e in handle_geoms]
            )

            door_open = self.env.fxtr.is_open(
                self.env,
                joint_names=[self._get_handle_door_joint_names(handle_bodies[door_num - 1])],
            )
            signals["stage_contact_handle_{}".format(door_num)] = int(contact_handle)
            signals["stage_open_door_{}".format(door_num)] = int(door_open) # and robot_cleared_door)
           
        signals["success"] = int(self.env._check_success())        
        return signals

    def skip_stage(self, stage_ind):
        if stage_ind not in self.DYNAMIC_STAGE_INDS:
            return False

        # Skip stage 0 if there is only one door
        if stage_ind >= 2 and len(self._get_handle_bodies()) < 2:
            return True
        
        return False
    

class MG_CloseSingleDoor(RobosuiteInterface):

    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        return dict(
            handle=self.get_object_pose(obj_name=f"{self.env.fxtr.naming_prefix}door_handle_main", obj_type="geom"),
        )
    
    def _get_single_door_body(self):

        def _find_parent(root, target):
            """
            Recursively find the parent of 'target' starting from 'root'.
            Returns the parent element if found, else None.
            """
            for child in root:
                if child is target:
                    return root
                parent = _find_parent(child, target)
                if parent is not None:
                    return parent
            return None
        
        door_joint_names = self.env.fxtr.door_joint_names
        assert len(door_joint_names) == 1, "task only supports door objects with one door"
        door_joint = find_elements(self.env.fxtr.worldbody, "joint", attribs={"name": door_joint_names[0]})
        door_body = _find_parent(self.env.fxtr.worldbody, door_joint)
        assert door_body is not None, "No door body found in env"
        return door_body

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

        single_door_body = self._get_single_door_body()
        door_geoms = find_elements(single_door_body, tags="geom", return_first=False)
        door_geom_names = [e.get("name") for e in door_geoms]
        contact_door = self.env.check_contact(
            self.env.robots[0].gripper["right"],
            door_geom_names
        )

        signals["stage_contact_door"] = int(contact_door)
        signals["success"] = int(self.env._check_success())

        return signals


class MG_CloseMultipleDoor(RobosuiteInterface):

    DYNAMIC_STAGE_INDS = set([0,1,2,3])

    def _get_door_bodies(self):

        def _find_parent(root, target):
            """
            Recursively find the parent of 'target' starting from 'root'.
            Returns the parent element if found, else None.
            """
            for child in root:
                if child is target:
                    return root
                parent = _find_parent(child, target)
                if parent is not None:
                    return parent
            return None
        
        door_joint_names = self.env.fxtr.door_joint_names
        door_joints = [find_elements(self.env.fxtr.worldbody, "joint", attribs={"name": name}) for name in door_joint_names]
        door_bodies = [_find_parent(self.env.fxtr.worldbody, joint) for joint in door_joints]
        return door_bodies

    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        door_bodies = self._get_door_bodies()
        door_1_pose = self.get_object_pose(door_bodies[0].get("name"), obj_type="body")
        door_2_pose = self.get_object_pose(door_bodies[1].get("name"), obj_type="body") if len(door_bodies) > 1 else np.zeros_like(door_1_pose)
        return dict(
            door_1=door_1_pose,
            door_2=door_2_pose,
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
        door_bodies = self._get_door_bodies()
        for door_num in [1, 2]:
            if door_num == 2 and len(door_bodies) < 2:
                signals["stage_contact_door_{}".format(door_num)] = -1
                signals["stage_close_door_{}".format(door_num)] = -1
                continue

            door_geoms = find_elements(
                door_bodies[door_num - 1],
                tags="geom",
                return_first=False
            )
            contact_door = self.env.check_contact(
                self.env.robots[0].gripper["right"],
                [e.get("name") for e in door_geoms]
            )

            door_closed = self.env.fxtr.is_closed(
                self.env,
                joint_names=[self.env.fxtr.door_joint_names[door_num - 1]],
            )

            signals["stage_contact_door_{}".format(door_num)] = int(contact_door)
            signals["stage_close_door_{}".format(door_num)] = int(door_closed)
        
        signals["success"] = int(self.env._check_success())

        return signals
    
    def skip_stage(self, stage_ind):
        if stage_ind not in self.DYNAMIC_STAGE_INDS:
            return False

        # Skip stage 0 if there is only one door
        if stage_ind >= 2 and len(self._get_door_bodies()) < 2:
            return True
        
        return False

class MG_OpenMicrowave(MG_OpenSingleDoor):
    pass

class MG_CloseMicrowave(MG_CloseSingleDoor):
    pass

class MG_OpenOven(MG_OpenSingleDoor):
    pass

class MG_CloseOven(MG_CloseSingleDoor):
    pass

class MG_OpenToasterOvenDoor(RobosuiteInterface):
    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        return dict(
            handle=self.get_object_pose(obj_name=f"{self.env.toaster_oven.naming_prefix}door_handle_main", obj_type="geom"),
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
        contact_handle = self.env.check_contact(
            self.env.robots[0].gripper["right"],
            f"{self.env.toaster_oven.naming_prefix}door_handle_main",
        )
        signals["stage_contact_handle"] = int(contact_handle)
        signals["success"] = int(self.env._check_success())
        return signals

class MG_CloseToasterOvenDoor(MG_CloseSingleDoor):

    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        return dict(
            handle=self.get_object_pose(obj_name=f"{self.env.toaster_oven.naming_prefix}door_handle_main", obj_type="geom"),
        )

    def _get_single_door_body(self):

        def _find_parent(root, target):
            """
            Recursively find the parent of 'target' starting from 'root'.
            Returns the parent element if found, else None.
            """
            for child in root:
                if child is target:
                    return root
                parent = _find_parent(child, target)
                if parent is not None:
                    return parent
            return None
        
        door_joint_names = self.env.toaster_oven.door_joint_names
        assert len(door_joint_names) == 1, "task only supports door objects with one door"
        door_joint = find_elements(self.env.toaster_oven.worldbody, "joint", attribs={"name": door_joint_names[0]})
        door_body = _find_parent(self.env.toaster_oven.worldbody, door_joint)
        assert door_body is not None, "No door body found in env"
        return door_body

class MG_OpenDishwasher(MG_OpenSingleDoor):
    pass

class MG_CloseDishwasher(MG_CloseSingleDoor):
    pass

class MG_OpenCabinet(MG_OpenMultipleDoor):
    pass
class MG_CloseCabinet(MG_CloseMultipleDoor):
    pass