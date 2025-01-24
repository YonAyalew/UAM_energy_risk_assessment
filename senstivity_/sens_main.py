from senst import SensitivityAnalyzer

def main():
    """
    Create the mission_parameters dictionary for your three
    sensitivity analyses: climb/descent, cruise altitude,
    and vertical climb. Update the paths and rows as needed.
    """
    
    mission_parameters = {
        "climb_descent": {
            "x_values": [9.5, 15, 20, 25, 30, 35, 40],
            "directories": {
                "GA11-GA66 (6nm)": r"C:\Users\...\6nm\GA11-GA66",
                "GABS-FTY (7nm)":  r"C:\Users\...\7nm\GABS-FTY",
                "FTY-GA54 (7.7nm)": r"C:\Users\...\7.7nm\FTY-GA54",
                "GA54-PDK (8.45nm)": r"C:\Users\...\8.45nm\GA54-PDK"
            },
            "rows_of_interest": [20, 44, 65, 89, 113]
        },
        "cruise_altitude": {
            "x_values": [2000, 3000, 4000, 5000, 6000],
            "directories": {
                "GA11-GA66 (6nm)": r"C:\Users\...\6nm\cruise_alt\GA11-GA66",
                "GABS-FTY (7nm)":  r"C:\Users\...\7nm\cruise_alt\GABS-FTY",
                "FTY-GA54 (7.7nm)": r"C:\Users\...\7.7nm\cruise_alt\FTY-GA54",
                "GA54-PDK (8.45nm)": r"C:\Users\...\8.45nm\cruise_alt\GA54-PDK"
            },
            "rows_of_interest": [5, 10, 15]  # example
        },
        "vertical_climb": {
            "x_values": [1000, 1500, 2000, 2500],
            "directories": {
                "GA11-GA66 (6nm)": r"C:\Users\...\6nm\vert_climb\GA11-GA66",
                "GABS-FTY (7nm)":  r"C:\Users\...\7nm\vert_climb\GABS-FTY",
                "FTY-GA54 (7.7nm)": r"C:\Users\...\7.7nm\vert_climb\FTY-GA54",
                "GA54-PDK (8.45nm)": r"C:\Users\...\8.45nm\vert_climb\GA54-PDK"
            },
            "rows_of_interest": [10, 11, 12]  # example
        }
    }
    
    # Instantiate our analyzer with the big dictionary:
    analyzer = SensitivityAnalyzer(mission_parameters)
    
    # Now, simply call the plot method for each parameter:
    analyzer.plot_param("climb_descent")
    analyzer.plot_param("cruise_altitude")
    analyzer.plot_param("vertical_climb")

if __name__ == "__main__":
    main()
