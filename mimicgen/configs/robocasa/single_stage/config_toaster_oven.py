from mimicgen.configs.config import MG_Config
from mimicgen.configs.robocasa.single_stage.config_stove import ManipulateKnob_Config

class TurnOnToasterOven_Config(ManipulateKnob_Config):
    NAME = "TurnOnToasterOven"

class AdjustToasterOvenTemperature_Config(ManipulateKnob_Config):
    NAME = "AdjustToasterOvenTemperature"