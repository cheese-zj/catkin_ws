"""Pre/post-processor factory for ACTTemporalPolicy.

Delegates to the standard ACT processor.  Inputs/outputs share the same
structure as ACT; the temporal dimension is handled transparently by the
normaliser (which broadcasts over the leading T dimension).
"""

from lerobot.policies.act.processor_act import make_act_pre_post_processors


def make_act_temporal_pre_post_processors(config, dataset_stats=None):
    """Return standard ACT pre/post-processors for ACTTemporalPolicy."""
    return make_act_pre_post_processors(config=config, dataset_stats=dataset_stats)
