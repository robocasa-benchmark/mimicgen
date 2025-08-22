from mimicgen.configs.config import MG_Config


class KitchenManipulateBlenderLid_Config(MG_Config):
    TYPE = "robosuite"

    def task_config(self):
        """
        This function populates the `config.task` attribute of the config, 
        which has task settings such as the task specification (the
        stages of each task, the amount of noise to apply during each stage, etc).
        """
        self.task.task_spec.stage_1 = dict(
            object_ref="lid_handle", 
            subtask_term_signal="stage_contact_lid", 
            subtask_term_offset_range=(5, 10),
            action_noise=0.05,
            num_interpolation_steps=5,
            selection_strategy="nearest_neighbor_interpolation",
            selection_strategy_kwargs=dict(nn_k=5),
        )
        self.task.task_spec.stage_2 = dict(
            object_ref="lid_handle", 
            subtask_term_signal=None, 
            subtask_term_offset_range=None, #(10, 20),
            action_noise=0.05,
            num_interpolation_steps=5,
            selection_strategy="nearest_neighbor_interpolation",
            selection_strategy_kwargs=dict(nn_k=5),
        )
        self.task.task_spec.do_not_lock_keys() # allow downstream code to completely replace the task spec

class KitchenOpenBlenderLid_Config(KitchenManipulateBlenderLid_Config):
    NAME = "OpenBlenderLid"

class KitchenCloseBlenderLid_Config(KitchenManipulateBlenderLid_Config):
    NAME = "CloseBlenderLid"

    def task_config(self):
        """
        This function populates the `config.task` attribute of the config, 
        which has task settings such as the task specification (the
        stages of each task, the amount of noise to apply during each stage, etc).
        """
        self.task.task_spec.stage_1 = dict(
            object_ref="lid_handle", 
            subtask_term_signal="stage_contact_lid", 
            subtask_term_offset_range=(0, 3),
            action_noise=0.0,
            num_interpolation_steps=5,
            selection_strategy="nearest_neighbor_interpolation",
            selection_strategy_kwargs=dict(nn_k=5),
        )
        self.task.task_spec.stage_2 = dict(
            object_ref="blender", 
            subtask_term_signal=None, 
            subtask_term_offset_range=None, #(10, 20),
            action_noise=0.0,
            num_interpolation_steps=5,
            selection_strategy="nearest_neighbor_interpolation",
            selection_strategy_kwargs=dict(nn_k=5),
        )
        self.task.task_spec.do_not_lock_keys() # allow downstream code to completely replace the task spec

class KitchenTurnOnBlender_Config(MG_Config):
    NAME = "TurnOnBlender"
    TYPE = "robosuite"

    def task_config(self):
        """
        This function populates the `config.task` attribute of the config, 
        which has task settings such as the task specification (the
        stages of each task, the amount of noise to apply during each stage, etc).
        """
        self.task.task_spec.stage_1 = dict(
            object_ref="button", 
            subtask_term_signal=None, 
            subtask_term_offset_range=None,
            action_noise=0.05,
            num_interpolation_steps=5,
            selection_strategy="nearest_neighbor_interpolation",
            selection_strategy_kwargs=dict(nn_k=5),
        )
        self.task.task_spec.do_not_lock_keys()