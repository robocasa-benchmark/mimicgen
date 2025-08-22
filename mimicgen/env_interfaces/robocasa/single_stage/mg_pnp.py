from mimicgen.env_interfaces.robosuite import RobosuiteInterface
from robocasa.models.fixtures import *

class MG_PnPObjectToContainer(RobosuiteInterface):

    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        return dict(
            obj=self.get_object_pose(obj_name=self.env.objects["obj"].root_body, obj_type="body"),
            container=self.get_object_pose(obj_name=self.env.objects["container"].root_body, obj_type="body"),
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
        contact_obj = self.env.check_contact(self.env.robots[0].gripper["right"], self.env.objects["obj"])
        signals["stage_contact_obj"] = int(contact_obj)
        signals["stage_place_obj"] = int(self.env._check_success())
        return signals

class MG_PnPCabinetToCounter(RobosuiteInterface):

    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        return dict(
            obj=self.get_object_pose(obj_name=self.env.objects["obj"].root_body, obj_type="body"),
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
        contact_obj = self.env.check_contact(self.env.robots[0].gripper["right"], self.env.objects["obj"])
        signals["stage_contact_obj"] = int(contact_obj)
        signals["stage_place_obj"] = int(self.env._check_success())
        return signals


class MG_PnPCounterToCabinet(RobosuiteInterface):

    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        if isinstance(self.env.cab, OpenCabinet):
            cab_ref = "{}_level0_shelf".format(self.env.cab.name)
        else:
            cab_ref = "{}_bottom".format(self.env.cab.name)

        return dict(
            obj=self.get_object_pose(obj_name=self.env.objects["obj"].root_body, obj_type="body"),
            cab=self.get_object_pose(obj_name=cab_ref, obj_type="geom"),
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
        contact_obj = self.env.check_contact(self.env.robots[0].gripper["right"], self.env.objects["obj"])
        signals["stage_contact_obj"] = int(contact_obj)
        signals["stage_place_obj"] = int(self.env._check_success())
        return signals


class MG_PnPCounterToSink(RobosuiteInterface):

    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        all_regions = list(self.env.sink.get_reset_regions().keys())
        chosen_region = self.env.rng.choice(all_regions)
        return dict(
            obj=self.get_object_pose(obj_name=self.env.objects["obj"].root_body, obj_type="body", obj_pos_offset="bottom"),
            sink=self.get_object_pose(obj_name="{}_reg_{}".format(self.env.sink.name, chosen_region), obj_type="geom"),
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
        contact_obj = self.env.check_contact(self.env.robots[0].gripper["right"], self.env.objects["obj"])
        signals["stage_contact_obj"] = int(contact_obj)
        signals["stage_place_obj"] = int(self.env._check_success())
        return signals


class MG_PnPSinkToCounter(RobosuiteInterface):

   def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        return dict(
            obj=self.get_object_pose(obj_name=self.env.objects["obj"].root_body, obj_type="body"),
            container=self.get_object_pose(obj_name=self.env.objects["container"].root_body, obj_type="body"),
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
        contact_obj = self.env.check_contact(self.env.robots[0].gripper["right"], self.env.objects["obj"])
        signals["stage_contact_obj"] = int(contact_obj)
        signals["stage_place_obj"] = int(self.env._check_success())
        return signals


class MG_PnPCounterToMicrowave(RobosuiteInterface):

    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        return dict(
            obj=self.get_object_pose(obj_name=self.env.objects["obj"].root_body, obj_type="body"),
            container=self.get_object_pose(obj_name=self.env.objects["container"].root_body, obj_type="body"),
            microwave=self.get_object_pose(obj_name="{}_{}".format(self.env.microwave.name, "tray"), obj_type="geom"),
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
        contact_obj = self.env.check_contact(self.env.robots[0].gripper["right"], self.env.objects["obj"])
        signals["stage_contact_obj"] = int(contact_obj)
        signals["stage_place_obj"] = int(self.env._check_success())
        return signals


class MG_PnPMicrowaveToCounter(RobosuiteInterface):

    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        return dict(
            obj=self.get_object_pose(obj_name=self.env.objects["obj"].root_body, obj_type="body"),
            container=self.get_object_pose(obj_name=self.env.objects["container"].root_body, obj_type="body"),
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
        contact_obj = self.env.check_contact(self.env.robots[0].gripper["right"], self.env.objects["obj"])
        signals["stage_contact_obj"] = int(contact_obj)
        signals["stage_place_obj"] = int(self.env._check_success())
        return signals


class MG_PnPCounterToStove(RobosuiteInterface):

    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        return dict(
            obj=self.get_object_pose(obj_name=self.env.objects["obj"].root_body, obj_type="body"),
            cookware=self.get_object_pose(obj_name=self.env.objects["container"].root_body, obj_type="body"),
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
        contact_obj = self.env.check_contact(self.env.robots[0].gripper["right"], self.env.objects["obj"])
        signals["stage_contact_obj"] = int(contact_obj)
        signals["stage_place_obj"] = int(self.env._check_success())
        return signals
    

class MG_PnPStoveToCounter(RobosuiteInterface):

    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        return dict(
            obj=self.get_object_pose(obj_name=self.env.objects["obj"].root_body, obj_type="body"),
            container=self.get_object_pose(obj_name=self.env.objects["container"].root_body, obj_type="body"),
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
        contact_obj = self.env.check_contact(self.env.robots[0].gripper["right"], self.env.objects["obj"])
        signals["stage_contact_obj"] = int(contact_obj)
        signals["stage_place_obj"] = int(self.env._check_success())
        return signals

class MG_PnPToasterOvenToCounter(MG_PnPObjectToContainer):
    pass

class MG_PnPCounterToToasterOven(RobosuiteInterface):

    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        return dict(
            obj=self.get_object_pose(obj_name=self.env.objects["obj"].root_body, obj_type="body"),
            rack=self.get_object_pose(obj_name=self.env.toaster_oven.naming_prefix + self.env.chosen_toaster_receptacle, obj_type="body"),
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
        contact_obj = self.env.check_contact(self.env.robots[0].gripper["right"], self.env.objects["obj"])
        signals["stage_contact_obj"] = int(contact_obj)
        signals["stage_place_obj"] = int(self.env._check_success())
        return signals
    
class MG_PnPCounterToStandMixer(RobosuiteInterface):
    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """

        return dict(
            obj=self.get_object_pose(obj_name=self.env.objects["obj"].root_body, obj_type="body"),
            bowl=self.get_object_pose(obj_name=f"{self.env.stand_mixer.naming_prefix}bowl", obj_type="body"),
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
        contact_obj = self.env.check_contact(self.env.robots[0].gripper["right"], self.env.objects["obj"])
        signals["stage_contact_obj"] = int(contact_obj)
        signals["stage_place_obj"] = int(self.env._check_success())
        return signals

class MG_PnPOvenToCounter(MG_PnPObjectToContainer):
    pass

class MG_PnPCounterToOven(RobosuiteInterface):

    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        return dict(
            obj=self.get_object_pose(obj_name=self.env.objects["obj"].root_body, obj_type="body"),
            oven_tray=self.get_object_pose(obj_name=self.env.objects["oven_tray"].root_body, obj_type="body"),
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
        contact_obj = self.env.check_contact(self.env.robots[0].gripper["right"], self.env.objects["obj"])
        signals["stage_contact_obj"] = int(contact_obj)
        signals["stage_place_obj"] = int(self.env._check_success())
        return signals

class MG_PnPToasterToCounter(MG_PnPObjectToContainer):
    def get_object_poses(self):
        return dict(
            obj=self.get_object_pose(obj_name=self.env.objects["obj"].root_body, obj_type="body"),
            container=self.get_object_pose(obj_name=self.env.objects["plate"].root_body, obj_type="body"),
        )

class MG_PnPDrawerToCounter(MG_PnPCabinetToCounter):
    pass

class MG_PnPCounterToDrawer(RobosuiteInterface):

    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        drawer_ref = f"{self.env.drawer.naming_prefix}reg_int"
        return dict(
            obj=self.get_object_pose(obj_name=self.env.objects["obj"].root_body, obj_type="body"),
            drawer=self.get_object_pose(obj_name=drawer_ref, obj_type="geom"),
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
        contact_obj = self.env.check_contact(self.env.robots[0].gripper["right"], self.env.objects["obj"])
        signals["stage_contact_obj"] = int(contact_obj)
        signals["stage_place_obj"] = int(self.env._check_success())
        return signals

class MG_PnPFridgeDrawerToShelf(RobosuiteInterface):

    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """

        # TODO: update to use the actual shelf from the demo, instead of assuming the middle one was used
        middle_shelf = list(self.env.fridge.get_reset_regions(env=self.env, compartment="fridge", reg_type="shelf", rack_index=-2).keys())[0]
        return dict(
            obj=self.get_object_pose(obj_name=self.env.objects["obj"].root_body, obj_type="body"),
            shelf=self.get_object_pose(obj_name=f"{self.env.fridge.naming_prefix}reg_{middle_shelf}", obj_type="geom"),
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
        contact_obj = self.env.check_contact(self.env.robots[0].gripper["right"], self.env.objects["obj"])
        signals["stage_contact_obj"] = int(contact_obj)
        signals["stage_place_obj"] = int(self.env._check_success())
        return signals

class MG_PnPFridgeShelfToDrawer(RobosuiteInterface):
    def get_drawer_reg_int(self):
        num_fridge_drawers = len(self.env.fridge._get_drawer_joints(compartment="fridge"))
        assert num_fridge_drawers > 0, "No fridge drawers found in the environment."
        # task always chooses the highest up drawer which corresponds to the drawer with the highest index
        drawer_num = num_fridge_drawers - 1 
        return f"{self.env.fridge.naming_prefix}reg_fridge_drawer{drawer_num}"
    
    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        drawer_reg_int = self.get_drawer_reg_int()
        return dict(
            obj=self.get_object_pose(obj_name=self.env.objects["obj"].root_body, obj_type="body"),
            drawer=self.get_object_pose(obj_name=drawer_reg_int, obj_type="geom"),
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
        contact_obj = self.env.check_contact(self.env.robots[0].gripper["right"], self.env.objects["obj"])
        signals["stage_contact_obj"] = int(contact_obj)
        signals["stage_place_obj"] = int(self.env._check_success())
        return signals

class MG_PnPCounterToBlender(RobosuiteInterface):

    def get_object_poses(self):
        """
        Gets the pose of each object relevant to MimicGen data generation in the current scene.

        Returns:
            object_poses (dict): dictionary that maps object name (str) to object pose matrix (4x4 np.array)
        """
        blender_ref = f"{self.env.blender.naming_prefix}reg_int"
        return dict(
            obj=self.get_object_pose(obj_name=self.env.objects["obj"].root_body, obj_type="body"),
            blender=self.get_object_pose(obj_name=blender_ref, obj_type="geom"),
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
        contact_obj = self.env.check_contact(self.env.robots[0].gripper["right"], self.env.objects["obj"])
        signals["stage_contact_obj"] = int(contact_obj)
        signals["stage_place_obj"] = int(self.env._check_success())
        return signals