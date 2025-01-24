import os
import pandas as pd
import matplotlib.pyplot as plt

class SensitivityAnalyzer:
    """
    A class to handle reading/averaging CSV energy consumption data for all mission
    parameters (climb angle, cruise altitude, vertical-climb height)
    and then plotting senstivity plots.
    """
    
    def __init__(self, mission_params):
        self.mission_params = mission_params

    def _process_csv_directory(self, directory, rows_of_interest, column_name='battery_energy_kw_hr'):
        """
        Reads and processes *all* .csv files in the given directory,
        extracting the specified 'column_name' at the specified row indices,
        returning the *average* of those values across all CSV files.
        """
        csv_files = [f for f in os.listdir(directory) if f.endswith(".csv")]
        all_values = []

        for csv_file in csv_files:
            file_path = os.path.join(directory, csv_file)
            df = pd.read_csv(file_path)
            
            # Extract the values for the requested rows
            row_vals = [df.at[r, column_name] for r in rows_of_interest]
            row_avg = sum(row_vals) / len(row_vals)
            all_values.append(row_avg)
        
        # Average across all CSV files
        if len(all_values) == 0:
            return None  # or raise an exception if no CSVs found
        
        return sum(all_values) / len(all_values)

    def get_param_data(self, param_name):
        """
        For a given parameter (e.g. 'climb_descent'), reads in all relevant
        CSV folders, processes them, and returns:
        
            x_values, { 'Label1': avgValue1, 'Label2': avgValue2, ... }
        
        This dictionary can be used for plotting or further analysis.
        """
        param_info = self.mission_params[param_name]
        x_values = param_info["x_values"]
        rows = param_info["rows_of_interest"]
        directories = param_info["directories"]
        
        results_dict = {}
        for label, directory in directories.items():
            avg_val = self._process_csv_directory(directory, rows)
            results_dict[label] = avg_val
        
        return x_values, results_dict

    def plot_param(self, param_name):
        """
        Plots a single figure for the given parameter, showing the
        average battery‐energy consumption vs. the parameter's x_values,
        for each flight‐distance dataset.
        
        If you have multiple x-values and a *single* average from a
        directory, we simply replicate that average at each x-value,
        as a quick way to visualize on one line. Adjust as needed if
        your CSV organization differs.
        """
        x_values, results_dict = self.get_param_data(param_name)

        plt.figure(figsize=(7, 5))
        
        # For each flight distance (label), we only have one numeric average.
        # We'll just replicate that average across all x_values for the sake
        # of demonstration.  If each CSV directory actually has multiple 
        # CSVs (one per x-value), you'd store them in a list and plot them 
        # properly:  plt.plot(x_values, your_list_of_averages).
        for label, val in results_dict.items():
            if val is None:
                continue  # skip if no data was found
            y_values = [val] * len(x_values)
            plt.plot(x_values, y_values, '-o', label=label)
        
        plt.xlabel(param_name)
        plt.ylabel("Average battery energy [kWh]")
        plt.title(f"Sensitivity for '{param_name}'")
        plt.legend()
        plt.show()
