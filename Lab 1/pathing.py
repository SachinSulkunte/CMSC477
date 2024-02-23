import aprilTagCp
import aprilTagPathing

path_to_goal = aprilTagPathing.compute_path()
if path_to_goal:
    print("Path to goal:", path_to_goal)
else:
    print("Goal not found.")
