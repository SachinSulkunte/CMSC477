import aprilTagPathing
from robomaster import robot

MAP_SCALE = 2
GRID_SIZE = 26.6/MAP_SCALE #cm

# takes in current (xy) and next (xy) and converts
# to x-distance and y-distance to move
def convert_coords_dist(curr_coords, next_coords):
    movement = [0, 0]
    sep_curr = curr_coords.split(' ')
    sep_next = next_coords.split(' ')
    movement[0] = (int(sep_next[0]) - int(sep_curr[0])) * GRID_SIZE / 100
    movement[1] = (int(sep_next[1]) - int(sep_curr[1])) * GRID_SIZE / 100
    print(movement)
    return movement

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis

    path = aprilTagPathing.compute_path()
    print("Path: " + str(path))

    x_val = 0.5
    y_val = 0.6
    z_val = 90

    for i in range(0, len(path)-1):
        movement = convert_coords_dist(path[i], path[i + 1])
        ep_chassis.move(x=movement[1], y=movement[0], z=0, xy_speed=0.7).wait_for_completed()
    
    ep_robot.close()
