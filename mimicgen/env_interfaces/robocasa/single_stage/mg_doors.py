from mimicgen.env_interfaces.robosuite import RobosuiteInterface

import numpy as np
from robosuite.utils.mjcf_utils import find_elements
from robocasa.models.fixtures import Fixture


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

    def _get_handles(self):

        door_handle_bodies = []

        for body in self.env.fxtr.worldbody.findall(".//body"):
            name = body.attrib.get("name", "")
            if "door_handle_main" in name:
                door_handle_bodies.append(body)

        return door_handle_bodies, "body"
    
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
        handle_elems, handle_type = self._get_handles()
        handle_names = [h.get("name") for h in handle_elems]
        # TODO randomize order of task! by shuffling handle_names

        handle_1_pose = self.get_object_pose(obj_name=handle_names[0], obj_type=handle_type) 
        handle_2_pose = self.get_object_pose(obj_name=handle_names[1], obj_type=handle_type) if len(handle_names) > 1 else np.zeros_like(handle_1_pose)
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
        handle_bodies, _ = self._get_handles()

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
        if stage_ind >= 2 and len(self._get_handles()) < 2:
            return True
        
        return False
    
    def _stage_order_swapped(self, all_datagen_info):
        """
        Returns True if, in the recorded trajectory, the robot made first contact
        with handle 2 before handle 1.  .
        """
        for term_sig in all_datagen_info["subtask_term_signals"]:
            if term_sig["stage_open_door_2"] == 1:
                return True          # handle‑2 came first
            elif term_sig["stage_open_door_1"] == 1:
                return False         # handle‑1 came first
        raise ValueError("Robot never made contact with handle")
    
    def _clean_contact_handle_2_signals(self, all_datagen_info):
        """
        When handle 2 is brushed accidentally while reaching for handle-1,
        `stage_contact_handle_2` may flip to 1 before door 1 is opened.  
        We blank those premature 1s.
        """
        term_list = all_datagen_info["subtask_term_signals"]

        for sig in term_list:
            # moment door 1 opens
            if sig["stage_open_door_1"] == 1:
                break

            if sig["stage_contact_handle_2"] != -1:
                sig["stage_contact_handle_2"] = 0

        return all_datagen_info


    
    def postprocess_datagen_info(self, all_datagen_info):
        """
        If the robot opened handle 2 first, swap datagen info
        """
        if self._stage_order_swapped(all_datagen_info):
            all_datagen_info = all_datagen_info.copy()

            new_obj_poses = []
            for poses in all_datagen_info["object_poses"]:
                new_obj_poses.append(
                    dict(
                        handle_1=poses["handle_2"].copy(),
                        handle_2=poses["handle_1"].copy(),
                    )
                )

            new_term_signals = []
            for info in all_datagen_info["subtask_term_signals"]:
                swapped = {}  
                swapped["stage_contact_handle_1"], swapped["stage_contact_handle_2"] = \
                    info["stage_contact_handle_2"], info["stage_contact_handle_1"]
                
                swapped["stage_open_door_1"],   swapped["stage_open_door_2"]   = \
                    info["stage_open_door_2"],   info["stage_open_door_1"]
                swapped["success"] = info["success"]
                new_term_signals.append(swapped)


            all_datagen_info["object_poses"] = new_obj_poses
            all_datagen_info["subtask_term_signals"] = new_term_signals
        
        all_datagen_info = self._clean_contact_handle_2_signals(all_datagen_info)
        
        return all_datagen_info
    

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

    def _stage_order_swapped(self, all_datagen_info):
        for term_signals in all_datagen_info["subtask_term_signals"]:
            if term_signals["stage_contact_door_2"] == 1:
                return True
            elif term_signals["stage_contact_door_1"] == 1:
                return False
        raise ValueError("Robot never made contact with door")

    def postprocess_datagen_info(self, all_datagen_info):
        if not self._stage_order_swapped(all_datagen_info):
            return all_datagen_info
        new_obj_poses = []
        new_term_signals = []

        for info in all_datagen_info["object_poses"]:
            # swap refs
            new_obj_poses.append(dict(door_1=info["door_2"].copy(), door_2=info["door_1"].copy()))
        
        for info in all_datagen_info["subtask_term_signals"]:
            signals = {}
            signals["stage_contact_door_1"] = info["stage_contact_door_2"]
            signals["stage_close_door_1"] = info["stage_close_door_2"]

            signals["stage_contact_door_2"] = info["stage_contact_door_1"]
            signals["stage_close_door_2"] = info["stage_close_door_1"]

            signals["success"] = info["success"]

            new_term_signals.append(signals)
                
        
        all_datagen_info["object_poses"] = new_obj_poses
        all_datagen_info["subtask_term_signals"] = new_term_signals
        return all_datagen_info

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

class MG_OpenFridge(MG_OpenMultipleDoor):
    def _get_handles(self):
        if len(self.env.fxtr._fridge_door_joint_names) > 1:
            assert len(self.env.fxtr._fridge_door_joint_names) == 2, "Expecting fridge with two handles"
            # arbitrarily get left then right
            names =  [f"{self.env.fxtr.naming_prefix}fridge_left_door_handle_main", f"{self.env.fxtr.naming_prefix}fridge_right_door_handle_main"]
        else:
            names =  [f"{self.env.fxtr.naming_prefix}fridge_door_handle_main"]
        return [find_elements(self.env.fxtr.worldbody, "geom", attribs={"name": name}) for name in names], "geom"
    
    def get_subtask_term_signals(self):
        signals = dict()
        handle_geoms_elems, _ = self._get_handles()
        handle_geom_names = [e.get("name") for e in handle_geoms_elems]

        for door_num in [1, 2]:
            if door_num == 2 and len(handle_geom_names) < 2:
                signals["stage_contact_handle_{}".format(door_num)] = -1
                signals["stage_open_door_{}".format(door_num)] = -1
                continue
            contact_handle = self.env.check_contact(
                self.env.robots[0].gripper["right"],
                [handle_geom_names[door_num - 1]]
            )

            if len(handle_geom_names) > 1:
                side = "left" if "left" in handle_geom_names[door_num - 1] else "right"
                door_joint_name = f"{self.env.fxtr.naming_prefix}fridge_{side}_door_joint"
            else:
                door_joint_name = f"{self.env.fxtr.naming_prefix}fridge_door_joint"

            # have to use Fixture class method because fridge overrident method does not allow
            # for custom joint names
            door_open =  Fixture.is_open(
                self.env.fxtr,
                self.env,
                joint_names=[door_joint_name],
            )
            signals["stage_contact_handle_{}".format(door_num)] = int(contact_handle)
            signals["stage_open_door_{}".format(door_num)] = int(door_open) # and robot_cleared_door)
           
        signals["success"] = int(self.env._check_success())        
        return signals
    
class MG_CloseFridge(MG_CloseMultipleDoor):
    def _get_door_bodies(self):
        if len(self.env.fxtr._fridge_door_joint_names) > 1:
            assert len(self.env.fxtr._fridge_door_joint_names) == 2, "Expecting fridge with two doors"
            # arbitrarily get left then right
            names =  [f"{self.env.fxtr.naming_prefix}fridge_left_door", f"{self.env.fxtr.naming_prefix}fridge_right_door"]
        else:
            names =  [f"{self.env.fxtr.naming_prefix}fridge_door"]
        return [find_elements(self.env.fxtr.worldbody, "body", attribs={"name": name}) for name in names]
    
    def get_subtask_term_signals(self):

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

            if len(door_bodies) > 1:
                side = "left" if "left" in door_bodies[door_num - 1].get("name") else "right"
                door_joint_name = f"{self.env.fxtr.naming_prefix}fridge_{side}_door_joint"
            else:
                door_joint_name = f"{self.env.fxtr.naming_prefix}fridge_door_joint"

            # have to use Fixture class method because fridge overrident method does not allow
            # for custom joint names
            door_closed =  Fixture.is_closed(
                self.env.fxtr,
                self.env,
                joint_names=[door_joint_name],
            )
            signals["stage_contact_door_{}".format(door_num)] = int(contact_door)
            signals["stage_close_door_{}".format(door_num)] = int(door_closed)
        
        signals["success"] = int(self.env._check_success())

        return signals