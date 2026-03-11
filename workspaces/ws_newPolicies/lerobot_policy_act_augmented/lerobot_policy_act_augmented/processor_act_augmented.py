from lerobot.policies.act.processor_act import make_act_pre_post_processors


def make_act_augmented_pre_post_processors(config, dataset_stats=None):
    """Return standard ACT pre/post-processors for ACTAugmentedPolicy."""
    return make_act_pre_post_processors(config=config, dataset_stats=dataset_stats)
