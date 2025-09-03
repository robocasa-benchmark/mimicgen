from mimicgen.configs.robocasa.single_stage.config_drawer import KitchenSlideRack_Config
from mimicgen.configs.robocasa.single_stage.config_stove import ManipulateKnob_Config

class SlideOvenRack_Config(KitchenSlideRack_Config):
    NAME = "SlideOvenRack"

class PreheatOven_Config(ManipulateKnob_Config):
    NAME = "PreheatOven"