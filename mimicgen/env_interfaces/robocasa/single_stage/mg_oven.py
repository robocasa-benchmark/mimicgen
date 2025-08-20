from mimicgen.env_interfaces.robocasa.single_stage.mg_drawer import MG_SlideRack

class MG_SlideOvenRack(MG_SlideRack):
    def get_rack_name(self):
        if self.env.oven.has_multiple_rack_levels():
            rack_level = self.env.rack_level
        else:
            rack_level = 0
        return self.env.oven.naming_prefix + f"rack{rack_level}"