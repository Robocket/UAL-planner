"""Select per-instance source points from projected pixel labels."""

import numpy as np


def select_mask_groups(points, projected_ids, id_map, metadata, selection='largest',
                       target_id=0, target_class=''):
    if selection not in ('largest', 'all'):
        raise ValueError('instance_selection must be largest or all')
    if target_id < 0:
        raise ValueError('target_instance_id must be >= 0')
    if len(points) != len(projected_ids):
        raise ValueError('Projected IDs must correspond one-to-one with source points')
    labels, sizes = np.unique(id_map, return_counts=True)
    if any(value > 0 and value not in metadata for value in labels):
        raise ValueError('Foreground id-map labels are missing from instance metadata')
    if any(value <= 0 for value in metadata):
        raise ValueError('Instance metadata must use positive IDs')
    eligible = [
        value for value in np.unique(projected_ids)
        if value > 0 and value in metadata
        and (not target_id or value == target_id)
        and (not target_class or metadata[value] == target_class)
    ]
    if selection == 'largest' and eligible:
        area = dict(zip(labels, sizes))
        eligible = [max(eligible, key=lambda value: (area[value], -int(value)))]
    return {int(value): points[projected_ids == value] for value in eligible}
