import matplotlib.pyplot as plt
import numpy as np
import csv

def velocity_profile(t, c1, c2, c3, c4, c5):
    return c1 + c2 * t + (1/2) * c3 * t**2 + (1/6) * c4 * t**3 + (1/24) * c5 * t**4

def space_over_time(t, c1, c2, c3, c4, c5):
    return c1 * t + (1/2) * c2 * t**2 + (1/6) * c3 * t**3 + (1/24) * c4 * t**4 + (1/120) * c5 * t**5

# plt.subplot(2, 2, 1)  # 2 rows, 1 column, first subplot
true_velocities = []
s = []
# CSV file reading
with open('path/to folder', newline='') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    next(reader)  # Skip header
    horizon = 50    
    first_primitive = True

    reader = csv.reader(csvfile, delimiter=',') 
    for row in reader:
        sf = float(row[1])
    
        s.append(sf)
        true_velocities.append(float(row[2]))
        c1, c2, c3, c4, c5 = map(float, row[3:8])
        time_values = np.linspace(0, 20, int(20/0.05))
        space_values = space_over_time(time_values, c1, c2, c3, c4, c5)
        offset = 160 - sf
        offsetted_space_values = [value + offset for value in space_values]
        velocity_values = velocity_profile(time_values, c1, c2, c3, c4, c5)

        plt.plot(offsetted_space_values, velocity_values, color='#add8e6', alpha=0.5)

true_velocities_space = np.linspace(0, 160, len(true_velocities))
plt.plot(s, true_velocities)
# Creating a subplot for the first plot
plt.xlim(0, 200)
plt.ylim(0, 20)
plt.xlabel('Space (m)')
plt.ylabel('Velocity (m/s)')
plt.title('Velocity Profiles with Receding Horizon')
plt.axis('equal')
plt.grid(True)

# # Read and plot data from the second CSV file
# with open('/home/bbhardy/Documents/IntelligentVehicles/bin/log_internal/control_log.csv', newline='') as csvfile:
#     reader = csv.reader(csvfile, delimiter=',')
#     next(reader)  # Skip header
#     horizon = 50
#     e = []
#     e_int = []
#     pedal_request = []
#     for row in reader:
#         e.append(float(row[0]))
#         e_int.append(float(row[1]))
#         pedal_request.append(float(row[2]))

# # Creating a subplot for the second plot
# e_space = np.linspace(0, 50, len(e))
# # Subplot for 'e'
# plt.subplot(2, 2, 2)  # 3 rows, 1 column, first subplot
# plt.plot(e_space, e, label='e')
# plt.xlabel('Space (m)')
# plt.ylabel('e')
# plt.title('e over Space')
# plt.legend()
# plt.grid(True)

# # Subplot for 'e_int'
# plt.subplot(2, 2, 3)  # 3 rows, 1 column, second subplot
# plt.plot(e_space, e_int, label='e_int')
# plt.xlabel('Space (m)')
# plt.ylabel('e_int')
# plt.title('e_int over Space')
# plt.legend()
# plt.grid(True)

# # Subplot for 'pedal_request'
# plt.subplot(2, 2, 4)  # 3 rows, 1 column, third subplot
# plt.plot(e_space, pedal_request, label='pedal_request')
# plt.xlabel('Space (m)')
# plt.ylabel('Pedal Request')
# plt.title('Pedal Request over Space')
# plt.legend()
# plt.grid(True)

# Display the figure with both subplots
# plt.tight_layout()  # Adjusts the subplots to fit into the figure area.
plt.show()
