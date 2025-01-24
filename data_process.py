# data_processing.py

import csv
import numpy as np
import statistics

def read_vertical_distances(csv_path):
    """
    Reads a CSV file containing vertical distances (one per line or row).
    Returns a NumPy array of floats.
    """
    distances = []
    with open(csv_path, 'r', newline='') as f:
        reader = csv.reader(f)
        for row in reader:
            # Adapt if your file has more columns or different structure
            distances.append(float(row[0]))
    return np.array(distances)

def convert_samples(verti_distances):
    """
    Convert vertical distances into 'energy samples' using your custom formula:
        miles = sample * 1.15078
        time_req = miles / 60
        math_output = time_req * 307.6898
    """
    energy_samples = []
    for sample in verti_distances:
        miles = sample * 1.15078
        time_req = miles / 60
        math_output = time_req * 307.6898
        energy_samples.append(math_output)
    return np.array(energy_samples)

def calculate_stats(energy_samples):
    """
    Calculates mean and standard deviation of the energy samples.
    By default, 'statistics.stdev' is sample standard deviation;
    use 'statistics.pstdev' for population if desired.
    """
    average = np.mean(energy_samples)
    std_deviation = statistics.stdev(energy_samples) 
    return average, std_deviation
