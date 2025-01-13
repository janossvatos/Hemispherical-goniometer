import os
import time
import threading
import numpy as np
import pandas as pd
import thorlabs_apt as apt
from seabreeze.spectrometers import Spectrometer
from robot_control import move_robot, check_boundaries
from sensor_control import home_motor, move_sensor, adjust_position, adjust_current_position, check_and_move_to_zero
 
def main():
    #Robot arm settings
    # Arm link lengths and parameters
    a1 = 244
    a2 = 213
    a3 = 83
    b = -170  # Distance of the sample from the Wrist 3 center
    phi_offset = np.radians(90)
    # Input coordinates
    X = 4
    Y = 410  # Distance of the measured sample from the shoulder
    # Control parameters, acceleration, velocity, time, radius
    a = 0.5
    v = 1.5
    t = 0 # Not used
    r = 0 # Not used
    #IP and port
    ip = ""
    port = 30003
    # Belt tensioner adjustment
    z_beltadjustment = 98
 
    # Motorized sensor stage settings
    motor = apt.Motor(40839252)
    homing_velocity = 10.0
    acceleration = 20.0  # Acceleration of the sensor movement
    max_velocity = 20.0  # Max velocity of the sensor movement
    motor.set_velocity_parameters(0.0, acceleration, max_velocity)

    #Spectrometer settings
    spec = Spectrometer.from_serial_number('QEB0413')
    spec.integration_time_micros(100000)  # Set integration time to 100ms
 
    # Enable dark counts and nonlinearity correction
    spec.correct_dark_counts = True
    spec.correct_nonlinearity = True

    def extract_suffix(filename):
        parts = filename.split('_')
        if len(parts) >= 4:
            return f"{parts[2]}_{parts[3]}"
        else:
            raise ValueError("Filename does not have enough parts to extract suffix")

    scanning_path_filename = 'dense_filtered_10_0.csv'

    # Read the CSV file
    scanning_path = pd.read_csv(os.path.join('/Users/jansvatos/Desktop/DTU/Master theisis/Code/My Programs/rotations/sampling_dense', scanning_path_filename))

    base_folder = '/Users/jansvatos/Desktop/DTU/Master theisis/Code/My Programs/rotations/measurements'

    folder_suffix = extract_suffix(scanning_path_filename)

    # Save the sorted dataframe with the new filename
    new_foldername = f'BRDF_dense_{folder_suffix}'

    # Create a folder based on the current date and time
    current_time = time.strftime("%Y%m%d_%H%M%S")
    final_folder_path = os.path.join(base_folder, f"{new_foldername}_{current_time}")
    os.makedirs(final_folder_path, exist_ok=True)

    # Home the sensor
    home_motor(motor, homing_velocity)
 
    # Control loop
    total_rows = len(scanning_path)
    for index, row in scanning_path.iterrows():
        sensor_position = (row['alpha'])
        x = -np.radians(row['beta_x'])
        y = np.radians(row['beta_y'])
        z = np.radians(row['beta_z']) + np.radians(z_beltadjustment)

        theta_i = row['theta_i']
        phi_i = row['phi_i']
        theta_r = row['theta_r']
        phi_r = row['phi_r']

        print(f"θi: {theta_i}, φi: {phi_i}, θr: {theta_r},  φr: {phi_r}")

        # Calculate and print progress
        progress = (index + 1) / total_rows * 100
        print(f"Progress: {progress:.2f}%")

        # Check boundaries
        if not check_boundaries(x, y, z, z_beltadjustment):
            break

        # Get sensor positions
        sensor_position_adj = adjust_position(sensor_position)
        current_position = adjust_current_position(motor.position)

        # Check if the motor needs to move to 0
        check_and_move_to_zero(motor, current_position, sensor_position)

        # Create threads for moving to position and getting waypoints
        sensor_thread = threading.Thread(target=move_sensor, args=(sensor_position_adj,))
        robot_thread = threading.Thread(target=move_robot, args=(y, ip, port, x, z, a1, a2, a3, b, phi_offset, X, Y, a, v, t, r))

        # Start threads
        sensor_thread.start()
        robot_thread.start()

        # Wait for threads to complete
        sensor_thread.join()
        robot_thread.join()

        time.sleep(1)

        # Collect 5 measurements
        wavelengths = spec.wavelengths()
        photon_counts = []
 
        for i in range(5):
            photon_counts.append(spec.intensities())
 
        # Calculate the average photon counts
        average_photon_counts = np.mean(photon_counts, axis=0)

        # Create a dataframe and save the wavelengths and average_photon_counts
        df = pd.DataFrame({
            'Wavelengths': wavelengths,
            'Photon Counts': average_photon_counts
        })
        # Save the dataframe in the created folder
        df.to_csv(os.path.join(final_folder_path, f'ti_{theta_i:.0f}_pi_{phi_i:.0f}_tr_{theta_r:.0f}_pr_{phi_r:.0f}.csv'), index=False)


if __name__ == "__main__":
    main()