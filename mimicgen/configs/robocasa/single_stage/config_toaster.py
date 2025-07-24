from mimicgen.configs.config import MG_Config

class TurnOnToasterConfig(MG_Config):

    NAME = "TurnOnToaster"
    TYPE = "robosuite"

    def task_config(self):
        self.task.task_spec.stage_1 = dict(
            object_ref="lever", 
            subtask_term_signal="stage_contact_lever", 
            subtask_term_offset_range=(0, 5),
            action_noise=0.0,
            num_interpolation_steps=5,
            selection_strategy="nearest_neighbor_interpolation",
            selection_strategy_kwargs=dict(nn_k=5),
        )
        self.task.task_spec.stage_2 = dict(
            object_ref="lever", 
            subtask_term_signal=None, 
            subtask_term_offset_range=None,
            action_noise=0.0,
            num_interpolation_steps=5,
            selection_strategy="nearest_neighbor_interpolation",
            selection_strategy_kwargs=dict(nn_k=5),
        )
        self.task.task_spec.do_not_lock_keys() # allow downstream code to completely replace the task spec