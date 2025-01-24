# plotting.py

import matplotlib.pyplot as plt
import seaborn as sns

def plot_histogram_with_markers(data_series, mean_val, var_val, cvar_val, title="", filename=None):
    """
    Plots a histogram of 'data_series' with vertical lines for Mean, VaR, and CVaR.
    """
    fig, ax = plt.subplots(figsize=(8, 6))
    sns.histplot(data_series, bins=40, color='salmon', edgecolor='black', kde=False, ax=ax)
    ax.set_xlabel("Risk Value")
    ax.set_ylabel("Frequency")
    ax.set_title(title)

    # Add vertical lines
    ax.axvline(mean_val, color='green', linestyle='--', label='Mean')
    ax.axvline(var_val, color='blue', linestyle='--', label='VaR')
    ax.axvline(cvar_val, color='red', linestyle='--', label='CVaR')

    ax.legend()
    plt.tight_layout()

    if filename:
        plt.savefig(filename)
    plt.show()

def plot_lines(angles_df, df_values, title="", filename=None):
    """
    Given angles (angles_df['angles']) and a DataFrame (df_values)
    with columns:
       "Average Mean"
       "Average VaR (alpha=0.95)"
       "Average CVaR (alpha=0.95)"
    plots them vs. angle in one figure.
    """
    fig, ax = plt.subplots(figsize=(10, 6))

    ax.plot(
        angles_df['angles'],
        df_values["Average Mean"],
        marker='o',
        label="Mean"
    )
    ax.plot(
        angles_df['angles'],
        df_values["Average VaR (alpha=0.95)"],
        marker='<',
        label="VaR (alpha=0.95)"
    )
    ax.plot(
        angles_df['angles'],
        df_values["Average CVaR (alpha=0.95)"],
        marker='s',
        label="CVaR (alpha=0.95)"
    )

    ax.set_xlabel("Climb/Descent Angles")
    ax.set_ylabel("Risk Values")
    ax.set_title(title)
    ax.grid(True)
    ax.legend()

    if filename:
        plt.savefig(filename)
    plt.show()
