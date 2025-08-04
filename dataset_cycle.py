import os
from nuscenes.nuscenes import NuScenes

# Load the nuScenes dataset (Ensure the path is correct)
nusc = NuScenes(version='v1.0-trainval', dataroot='/disk/ml/datasets/nuScenes/', verbose=True)

# Set to store unique bicycle rider instance tokens
unique_bicycle_riders = set()   #A set is used instead of a list because it automatically removes duplicates.It stores unique instance tokens of bicycles with riders.

# Iterate through all annotations
for annotation in nusc.sample_annotation:
    if annotation['category_name'] == 'vehicle.bicycle':
        # Check if the bicycle has the "cycle.with_rider" attribute
        attr_tokens = annotation.get('attribute_tokens', [])
        for attr_token in attr_tokens:
            attr = nusc.get('attribute', attr_token)
            if attr['name'] == 'cycle.with_rider':
                unique_bicycle_riders.add(annotation['instance_token'])

# Total number of unique bicycle riders
num_riders = len(unique_bicycle_riders)

print(f"Total number of unique bicycle riders in NuScenes: {num_riders}")
