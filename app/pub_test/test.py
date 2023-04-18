om = [0.28299999237060547, 0.26899999380111694, 0.37299999594688416, 0.0, 0.0, 0.0, 0.0, 0.0]

print(om)
# make the obstcle map become unit (0,1)
obstacle_map_unit = om
for i in range(8):
    if obstacle_map_unit[i] <= 0.0:
        obstacle_map_unit[i] = 0.0
    elif obstacle_map_unit[i] <= 0.3:
        obstacle_map_unit[i] = 1
    elif obstacle_map_unit[i] <= 0.5:
        obstacle_map_unit[i] = (float(obstacle_map_unit[i]) - 0.3) / (0.5 - 0.3)
    else:
        obstacle_map_unit[i] = 0.0

print('        ',obstacle_map_unit)

x_rep_matric = [0, 2**(-0.5), 1, 2**(-0.5), 0, -(2**(-0.5)), -1, -(2**(-0.5))]
y_rep_matric = [-1, -(2**(-0.5)), 0, 2**(-0.5), 1, 2**(-0.5), 0, -(2**(-0.5))]
x_rep_intensity_matric = [0, 2**(-0.5), 1, 2**(-0.5), 0, (2**(-0.5)), 1, (2**(-0.5))]
y_rep_intensity_matric = [1, (2**(-0.5)), 0, 2**(-0.5), 1, 2**(-0.5), 0, (2**(-0.5))]

print(x_rep_matric, y_rep_matric, x_rep_intensity_matric, y_rep_intensity_matric)