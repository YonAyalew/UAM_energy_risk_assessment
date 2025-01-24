# risk_calculations.py

import numpy as np
from scipy.stats import truncnorm

def risk_profile(reserve_energy_values, initial_battery, data_array):
    """
    Example placeholder function for your risk formula, based on 
    the snippet in your code.
    Adapt to reflect your actual 'risk' computation logic.
    """
    # For demonstration:
    #   constant = np.divide(np.log10(reserve_energy_values), np.log10(np.e))
    #   risk_values = np.divide(constant, np.maximum(initial_battery - data_array, reserve_energy_values)) - 1
    # Here we just do a simple placeholder:
    constant = np.log10(reserve_energy_values + 1.0)  # +1 to avoid log(0)
    risk_values = (constant / (initial_battery - data_array + 1.0)) - 1
    return risk_values

def calculate_var(series, alpha):
    """Computes the VaR at given alpha for a 1D array-like 'series'."""
    return np.quantile(series, alpha)

def calculate_cvar(series, alpha):
    """Computes the CVaR at given alpha for a 1D array-like 'series'."""
    var_threshold = calculate_var(series, alpha)
    # CVaR = average of all values >= var_threshold
    return np.mean(series[series >= var_threshold])

def simulate_risk(
    seed,
    num_iterations,
    initial_battery,
    data_array,
    mean_value,
    std_dev,
    alpha=0.95
):
    """
    Generates random draws (truncated normal), applies 'risk_profile',
    and returns arrays of Mean, VaR, CVaR across 'num_iterations'.
    """
    np.random.seed(seed)
    lower_bound = mean_value - 4 * std_dev
    upper_bound = mean_value + 4 * std_dev

    energy_sims = []
    for _ in range(num_iterations):
        # truncated normal draws the same length as 'data_array'
        samples = truncnorm(
            (lower_bound - mean_value) / std_dev,
            (upper_bound - mean_value) / std_dev,
            loc=mean_value,
            scale=std_dev
        ).rvs(len(data_array))

        # Example "risk_profile" usage: pass in 'samples' as "reserve_energy_values"
        risk_vals = risk_profile(samples, initial_battery, data_array)
        energy_sims.append(risk_vals)

    df_sims = np.array(energy_sims)  # shape: (num_iterations, len(data_array))

    # For each iteration (row), get the mean risk, VaR, and CVaR
    mean_values = df_sims.mean(axis=1)
    var_values = np.apply_along_axis(lambda row: calculate_var(row, alpha), 1, df_sims)
    cvar_values = np.apply_along_axis(lambda row: calculate_cvar(row, alpha), 1, df_sims)

    return mean_values, var_values, cvar_values

def run_multiple_seeds(
    seeds,
    num_iterations,
    initial_battery,
    data_array,
    mean_energy,
    std_energy,
    alpha=0.95
):
    """
    Loops over multiple seeds, calls simulate_risk for each, 
    and returns the AVERAGE (over seeds) of the means, VaRs, CVaRs.
    """
    all_means = []
    all_vars = []
    all_cvars = []

    for seed in seeds:
        mean_vals, var_vals, cvar_vals = simulate_risk(
            seed,
            num_iterations,
            initial_battery,
            data_array,
            mean_energy,
            std_energy,
            alpha
        )
        # Each of these is length = num_iterations. We can average across iterations:
        all_means.append(np.mean(mean_vals))
        all_vars.append(np.mean(var_vals))
        all_cvars.append(np.mean(cvar_vals))

    # Return the overall average across all seeds
    return (
        np.mean(all_means),
        np.mean(all_vars),
        np.mean(all_cvars)
    )
