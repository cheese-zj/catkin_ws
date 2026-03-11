# Import both classes at package-import time so that the
# @PreTrainedConfig.register_subclass("act_temporal") decorator fires and
# LeRobot's policy factory can resolve the "act_temporal" name.
from .configuration_act_temporal import ACTTemporalConfig  # noqa: F401
from .modeling_act_temporal import ACTTemporalPolicy  # noqa: F401
