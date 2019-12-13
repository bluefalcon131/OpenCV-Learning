import yaml

def get_coords():
    with open('../example/input_points.yaml') as input_points_file:
        input_points = yaml.load(input_points_file)
    room1_entrance = input_points['room1_entrance_xy']
    room1_centre = input_points['room1_centre_xy']
    room2_entrance = input_points['room2_entrance_xy']
    room2_centre = input_points['room2_centre_xy']
    return room1_entrance,room1_centre, room2_entrance,room2_centre
