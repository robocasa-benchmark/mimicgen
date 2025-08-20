from mimicgen.configs.config import MG_Config
from mimicgen.configs.robocasa.single_stage.config_stove import ManipulateKnob_Config
from mimicgen.configs.robocasa.single_stage.config_drawer import KitchenSlideRack_Config

class TurnOnToasterOven_Config(ManipulateKnob_Config):
    NAME = "TurnOnToasterOven"

class AdjustToasterOvenTemperature_Config(ManipulateKnob_Config):
    NAME = "AdjustToasterOvenTemperature"

class SlideToasterOvenRack_Config(KitchenSlideRack_Config):
    NAME = "SlideToasterOvenRack"