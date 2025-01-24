# main.py

import numpy as np
import pandas as pd

from data_processing import (
    read_vertical_distances,
    convert_samples,
    calculate_stats
)
from risk_calculations import (
    run_multiple_seeds,
    simulate_risk
)
from plotting import (
    plot_histogram_with_markers,
    plot_lines
)

def main():
    # ----------------------------------------------------------------------
    # 1) Read vertical-distances data -> convert to energy samples -> stats
    # ----------------------------------------------------------------------
    verti_distances_path = "data/vertical_distances.csv"  # Adjust to your path
    distances = read_vertical_distances(verti_distances_path)
    energy_samples = convert_samples(distances)
    mean_energy, std_energy = calculate_stats(energy_samples)

    print("Energy Samples Mean:", mean_energy)
    print("Energy Samples Std Dev:", std_energy)

    # ----------------------------------------------------------------------
    # 2) Set up parameters for the risk simulation
    # ----------------------------------------------------------------------
    initial_battery = 239.43661967
    seeds = [101, 269, 268, 70, 111, 201, 469, 277, 20, 495]
    num_iterations = 1000
    alpha = 0.95

    # ----------------------------------------------------------------------
    # 3) Loop over a list of angles, automatically computing aggregated risk
    # ----------------------------------------------------------------------
    angles_list = [9.5, 15, 20, 25, 30, 35, 40]  # or read from a CSV/file
    results_for_all_angles = {
        "Average Mean": [],
        "Average VaR (alpha=0.95)": [],
        "Average CVaR (alpha=0.95)": []
    }

    for angle in angles_list:
        # Example: if each angle has its own route_data file:
        route_data_path = f"data/route_data_{angle}.csv"
        data_array = np.loadtxt(route_data_path, delimiter=",")

        # Now get the aggregated risk metrics for this angle
        avg_mean, avg_var, avg_cvar = run_multiple_seeds(
            seeds=seeds,
            num_iterations=num_iterations,
            initial_battery=initial_battery,
            data_array=data_array,
            mean_energy=mean_energy,
            std_energy=std_energy,
            alpha=alpha
        )

        # Append to dictionary
        results_for_all_angles["Average Mean"].append(avg_mean)
        results_for_all_angles["Average VaR (alpha=0.95)"].append(avg_var)
        results_for_all_angles["Average CVaR (alpha=0.95)"].append(avg_cvar)

    # Create a DataFrame from all the angles' results
    df_values = pd.DataFrame(results_for_all_angles)
    angles_df = pd.DataFrame({"angles": angles_list})

    print("\n--- ANGLE-BASED RISK RESULTS ---")
    print(pd.concat([angles_df, df_values], axis=1))

    # ----------------------------------------------------------------------
    # 4) Plot lines: Mean / VaR / CVaR vs. angle
    # ----------------------------------------------------------------------
    plot_lines(
        angles_df=angles_df,
        df_values=df_values,
        title="Risk vs Climb/Descent Angles",
        filename="risk_vs_angles.png"
    )

    # ----------------------------------------------------------------------
    # 5) (Optional) Demonstrate a single-seed distribution histogram
    # ----------------------------------------------------------------------
    seed_index = 0  # pick the first seed from your seed list
    chosen_seed = seeds[seed_index]

    # Suppose we just pick one angle to demonstrate a histogram
    single_angle = 20
    single_data_path = f"data/route_data_{single_angle}.csv"
    data_array_for_hist = np.loadtxt(single_data_path, delimiter=",")

    mean_vals, var_vals, cvar_vals = simulate_risk(
        seed=chosen_seed,
        num_iterations=num_iterations,
        initial_battery=initial_battery,
        data_array=data_array_for_hist,
        mean_value=mean_energy,
        std_dev=std_energy,
        alpha=alpha
    )
    # mean_vals etc. each have length = num_iterations
    # Let's plot distribution of mean_vals as an example
    distribution = mean_vals

    plot_histogram_with_markers(
        data_series=distribution,
        mean_val=np.mean(distribution),
        var_val=np.quantile(distribution, alpha),
        cvar_val=np.mean(distribution[distribution >= np.quantile(distribution, alpha)]),
        title=f"Histogram of Mean Risk Distribution (Angle={single_angle}, Seed={chosen_seed})",
        filename="risk_histogram.png"
    )

if __name__ == "__main__":
    main()
