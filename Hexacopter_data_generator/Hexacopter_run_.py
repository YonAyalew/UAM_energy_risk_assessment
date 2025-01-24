 # test_Multicopter.py
#
# Created: Feb 2020, M. Clarke
#          Sep 2020, M. Clarke

""" setup file for a mission with an Electic Multicopter
"""

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units
from SUAVE.Plots.Performance.Mission_Plots import *
from SUAVE.Plots.Geometry  import * 
import numpy as np
import sys
import pandas as pd
import time
import os
import threading

#sys.path.append('.Vehicle')
# the analysis functions

from Electric_Multicopter_5  import vehicle_setup

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    # ------------------------------------------------------------------------------------------------------------------
    # Electric Multicopter
    # ------------------------------------------------------------------------------------------------------------------
    start_time = time.time()
    #Trajectory input  and use of it
    input_path = './data/trajectories//ATL/FTY-GA54/'
    counts = 0 
    start_time_tj = time.time()
    trajectory_data = pd.read_csv(input_path+'ATL_7.7nm_10_1.csv')
    tj = trajectory_data.values
    tj = compress_trajectory(tj)
  
    trajectory_data_1 = pd.read_csv(input_path+'ATL_7.7nm_20_4.csv')
    tj1 = trajectory_data_1.values
    tj1 = compress_trajectory(tj1)
    
    
    trajectory_data_2 = pd.read_csv(input_path+'ATL_7.7nm_40_6.csv')
    tj2 = trajectory_data_2.values
    tj2 = compress_trajectory(tj2)
    
    
    trajectory_data_3 = pd.read_csv(input_path+'ATL_7.7nm_20_5.csv')
    tj3 = trajectory_data_3.values
    tj3 = compress_trajectory(tj3)
    
    
    trajectory_data_4 = pd.read_csv(input_path+'ATL_7.7nm_20_6.csv')
    tj4 = trajectory_data_4.values
    tj4 = compress_trajectory(tj4)
    

    # build the vehicle, configs, and analyses
    configs, analyses = full_setup(tj,tj1,tj2,tj3,tj4)
    analyses.finalize()

    # Print weight properties of vehicle
    print(configs.base.weight_breakdown)
    print(configs.base.mass_properties.center_of_gravity)

    mission      = analyses.missions.base
    results      = mission.evaluate()
    agent_id = 1
    # plot results
    plot_mission(results)
    save_results(results, profile_id=agent_id)
    print(time.time()-start_time_tj)

    # save, load and plot old results
    #save_multicopter_results(results)
    old_results = load_multicopter_results()
    plot_mission(old_results,'k-')
    # plt.show(block=True)

    return


# ----------------------------------------------------------------------
#  Trajectory appending
# ----------------------------------------------------------------------

def compress_trajectory(tj):
    N = tj.shape[0]
    prev_speed = np.linalg.norm(tj[0,2:4])
    new_tj = []
    new_tj.append(tj[0,:])
    for i in range(1,N):
        if tj[i-1,4] > tj[i,4]:
            break
        cur_speed = np.linalg.norm(tj[i,2:4])
        err_speed = abs(cur_speed - prev_speed)
    
        if i+1 != N and err_speed < 1:
            pass
        else:
            new_tj.append(tj[i,:])
        prev_speed = cur_speed
    return np.array(new_tj)



def compress_trajectory(tj1):
    N = tj1.shape[0]
    prev_speed = np.linalg.norm(tj1[0,2:4])
    new_tj1 = []
    new_tj1.append(tj1[0,:])
    for i in range(1,N):
        if tj1[i-1,4] > tj1[i,4]:
            break
    
        cur_speed = np.linalg.norm(tj1[i,2:4])
        err_speed = abs(cur_speed - prev_speed)
        if i+1 != N and err_speed < 1:
            pass
        else:
            new_tj1.append(tj1[i,:])
        prev_speed = cur_speed
    
    return np.array(new_tj1)


def compress_trajectory(tj2):
    N = tj2.shape[0]
    prev_speed = np.linalg.norm(tj2[0,2:4])
    new_tj2 = []
    new_tj2.append(tj2[0,:])
    for i in range(1,N):
        if tj2[i-1,4] > tj2[i,4]:
            break
        cur_speed = np.linalg.norm(tj2[i,2:4])
        err_speed = abs(cur_speed - prev_speed)

        if i+1 != N and err_speed < 1:
            pass
        else:
            new_tj2.append(tj2[i,:])

        prev_speed = cur_speed
    return np.array(new_tj2)



def compress_trajectory(tj3):
    N = tj3.shape[0]
    prev_speed = np.linalg.norm(tj3[0,2:4])
    new_tj3 = []
    new_tj3.append(tj3[0,:])
    for i in range(1,N):
        if tj3[i-1,4] > tj3[i,4]:
            break
        cur_speed = np.linalg.norm(tj3[i,2:4])
        err_speed = abs(cur_speed - prev_speed)
        if i+1 != N and err_speed < 1:
            pass
        else:
            new_tj3.append(tj3[i,:])
        prev_speed = cur_speed
    return np.array(new_tj3)



def compress_trajectory(tj4):
    N = tj4.shape[0]
    prev_speed = np.linalg.norm(tj4[0,2:4])
    new_tj4 = []
    new_tj4.append(tj4[0,:])
    for i in range(1,N):
        if tj4[i-1,4] > tj4[i,4]:
            break
        cur_speed = np.linalg.norm(tj4[i,2:4])
        err_speed = abs(cur_speed - prev_speed)
        if i+1 != N and err_speed < 1:
            pass
        else:
            new_tj4.append(tj4[i,:])
        prev_speed = cur_speed
    return np.array(new_tj4)


# ----------------------------------------------------------------------
#   Setup
# ----------------------------------------------------------------------
def full_setup(tj,tj1,tj2,tj3,tj4):

    # vehicle data
    vehicle  = vehicle_setup()
    configs  = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs)

    # mission analyses
    mission  = mission_setup(configs_analyses,vehicle,tj,tj1,tj2,tj3,tj4)
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses

    return configs, analyses

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):
    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses


# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)

    return configs

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------
    analyses = SUAVE.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Basic Geometry Relations
    sizing = SUAVE.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)

    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights_eVTOL()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Energy
    energy= SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.networks
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)

    return analyses


def mission_setup(analyses,vehicle,tj,tj1,tj2,tj3,tj4):
    
    segment_type = []
    climb_rate   = []
    descend_rate = []
    start_altitude=[]
    end_altitude = []
    climb_angle =  []
    descent_angle =[]
    speed_spec =   []
    time_required =[]

    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission     = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # airport
    airport            = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()
    mission.airport    = airport

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment                                             = Segments.Segment()
    ones_row                                                 = base_segment.state.ones_row
    # base_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.state.numerics.number_control_points        = 3
    base_segment.process.initialize.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.finalize.post_process.update_battery_state_of_health = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_state_of_health  
    base_segment.process.iterate.conditions.planet_position  = SUAVE.Methods.skip 

    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                               = Segments.Hover.Climb(base_segment)
    segment.tag                                           = "Hover_Climb"
    segment.analyses.extend( analyses.base)
    segment.altitude_start                                = 0.0  * Units.ft
    segment.altitude_end                                  = 500.  * Units.ft
    segment.climb_rate                                    = 100. * Units['ft/min']
    segment.battery_energy                                = vehicle.networks.battery_propeller.battery.max_energy
    segment.state.unknowns.throttle                       = 0.6 * ones_row(1)
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,initial_power_coefficient=0.02)

    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(segment.climb_rate)
    descend_rate.append(np.nan)
    start_altitude.append(segment.altitude_start)
    end_altitude.append(segment.altitude_end)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(np.nan)
    time_required.append(np.nan)


    
    # ------------------------------------------------------------------
    #   Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                                 = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag                                             = "Accel_Climb"
    segment.analyses.extend( analyses.base)
    segment.air_speed                                       = 29.484364 * Units.knots
    segment.altitude_end                                    = 2000. * Units.ft
    segment.climb_angle                                     = 15 * Units.degrees
    segment.climb_rate                                      = 800. * Units['ft/min']
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(segment.climb_rate)
    descend_rate.append(np.nan)
    start_altitude.append(segment.altitude_start)
    end_altitude.append(segment.altitude_end)
    climb_angle.append(segment.climb_angle)
    descent_angle.append(np.nan)
    speed_spec.append(segment.air_speed)
    time_required.append(np.nan)
    
    
    # ------------------------------------------------------------------
    #   Hover Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                                 = Segments.Hover.Hover(base_segment)
    segment.tag                                             = "Hover"
    segment.analyses.extend( analyses.base)
    segment.altitude                                        = 2000.  * Units.ft
    segment.time                                            = (1/3)*60
    segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability         = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics         = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(np.nan)
    start_altitude.append(np.nan)
    end_altitude.append(np.nan)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(np.nan)
    time_required.append(np.nan)
    
    
    
    # # ------------------------------------------------------------------
    # #   Cruise: Constant Speed, Constant Altitude
    # # ------------------------------------------------------------------
    # segment                                                 = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    # segment.tag                                             = "Cruise"
    # segment.analyses.extend( analyses.base)
    # segment.air_speed                                       = 53.8765  * Units.knots
    # segment.distance                                        = 9.7246* Units.nautical_miles
    # segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    # segment.process.finalize.post_process.stability         = SUAVE.Methods.skip
    # segment.process.iterate.conditions.aerodynamics         = SUAVE.Methods.skip 
    # segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)
    
    # # ------------------------------------------------------------------
    # #   Cruise: Constant Speed, Constant Altitude
    # # ------------------------------------------------------------------
    
    total_t = 0
    speed = 0
    for i in range(tj.shape[0]-1):
        total_t = tj[i+1,4] - tj[i,4]
        speed = np.linalg.norm(tj[i,2:4])
        
        segment                                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
        segment.tag                                      = "cruise_"+str(i)
        segment.analyses.extend( analyses.base ) 
        segment.altitude                                 = 2000.0 * Units.ft
        segment.air_speed                                = speed * Units['m/s'] #110.   * Units['mph']
        segment.distance                                 = total_t * speed * Units['m'] #50.    * Units.miles
        segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
        segment.process.finalize.post_process.stability         = SUAVE.Methods.skip
        segment.process.iterate.conditions.aerodynamics         = SUAVE.Methods.skip 
        segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(np.nan)
    start_altitude.append(segment.altitude)
    end_altitude.append(segment.altitude)
    # start_altitude.append(np.nan)
    # end_altitude.append(np.nan)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(segment.air_speed)
    # time_required.append(np.nan)
    time_required.append(total_t)
    
    
    
    # ------------------------------------------------------------------
    #   Hover Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                                 = Segments.Hover.Hover(base_segment)
    segment.tag                                             = "Hover_2"
    segment.analyses.extend( analyses.base)
    segment.altitude                                        = 2000.  * Units.ft
    segment.time                                            = (1/3)*60
    segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability         = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics         = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(np.nan)
    start_altitude.append(np.nan)
    end_altitude.append(np.nan)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(np.nan)
    time_required.append(np.nan)
    
    
    
    # ------------------------------------------------------------------
    #   Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                                 = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag                                             = "Accel_descent"
    segment.analyses.extend( analyses.base)
    segment.air_speed                                       = 29.484364 * Units.knots
    segment.altitude_end                                    = 500. * Units.ft
    segment.descent_angle                                   = 15 * Units.degrees
    segment.descent_rate                                    = 800. * Units['ft/min']
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(segment.descent_rate)
    start_altitude.append(segment.altitude_start)
    end_altitude.append(segment.altitude_end)
    climb_angle.append(np.nan)
    descent_angle.append(segment.descent_angle)
    speed_spec.append(segment.air_speed)
    time_required.append(np.nan)
    
    
    
    
    
    # ------------------------------------------------------------------
    #   last descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                               = Segments.Hover.Descent(base_segment)
    segment.tag                                           = "Hover_Descent"
    segment.analyses.extend( analyses.base)
    segment.altitude_start                                = 500.  * Units.ft
    segment.altitude_end                                  = 0.  * Units.ft
    segment.descent_rate                                  = 100. * Units['ft/min']
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,initial_power_coefficient=0.02)

    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(segment.descent_rate)
    start_altitude.append(segment.altitude_start)
    end_altitude.append(segment.altitude_end)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(np.nan)
    time_required.append(np.nan)
    
    
    
    # ------------------------------------------------------------------
    #  Charge Segment: 
    # ------------------------------------------------------------------  
    # Charge Model 
    segment                                                  = Segments.Ground.Battery_Charge_Discharge(base_segment)     
    segment.tag                                              = 'Charge'
    segment.analyses.extend(analyses.base)           
    segment.battery_discharge                                = False    
    segment.increment_battery_cycle_day                      = True 
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip        
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)    

    # add to misison
    mission.append_segment(segment) 
    
    
    ###################### Second mission ###########################################################################
    #----------------------------------------------------------------------------------------------------------------
    #----------------------------------------------------------------------------------------------------------------
    #----------------------------------------------------------------------------------------------------------------
    #----------------------------------------------------------------------------------------------------------------
    
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                               = Segments.Hover.Climb(base_segment)
    segment.tag                                           = "Hover_Climb_1"
    segment.analyses.extend( analyses.base)
    segment.altitude_start                                = 0.0  * Units.ft
    segment.altitude_end                                  = 500.  * Units.ft
    segment.climb_rate                                    = 100. * Units['ft/min']
    segment.battery_energy                                = vehicle.networks.battery_propeller.battery.max_energy
    segment.state.unknowns.throttle                       = 0.6 * ones_row(1)
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,initial_power_coefficient=0.02)

    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(segment.climb_rate)
    descend_rate.append(np.nan)
    start_altitude.append(segment.altitude_start)
    end_altitude.append(segment.altitude_end)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(np.nan)
    time_required.append(np.nan)


    
    # ------------------------------------------------------------------
    #   Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                                 = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag                                             = "Accel_Climb_1"
    segment.analyses.extend( analyses.base)
    segment.air_speed                                       = 29.484364 * Units.knots
    segment.altitude_end                                    = 2000. * Units.ft
    segment.climb_angle                                     = 15 * Units.degrees
    segment.climb_rate                                      = 800. * Units['ft/min']
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(segment.climb_rate)
    descend_rate.append(np.nan)
    start_altitude.append(segment.altitude_start)
    end_altitude.append(segment.altitude_end)
    climb_angle.append(segment.climb_angle)
    descent_angle.append(np.nan)
    speed_spec.append(segment.air_speed)
    time_required.append(np.nan)
    
    
    # ------------------------------------------------------------------
    #   Hover Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                                 = Segments.Hover.Hover(base_segment)
    segment.tag                                             = "Hover_1"
    segment.analyses.extend( analyses.base)
    segment.altitude                                        = 2000.  * Units.ft
    segment.time                                            = (1/3)*60
    segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability         = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics         = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(np.nan)
    start_altitude.append(np.nan)
    end_altitude.append(np.nan)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(np.nan)
    time_required.append(np.nan)
    
    
    
    # # ------------------------------------------------------------------
    # #   Cruise: Constant Speed, Constant Altitude
    # # ------------------------------------------------------------------
    # segment                                                 = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    # segment.tag                                             = "Cruise"
    # segment.analyses.extend( analyses.base)
    # segment.air_speed                                       = 53.8765  * Units.knots
    # segment.distance                                        = 9.7246* Units.nautical_miles
    # segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    # segment.process.finalize.post_process.stability         = SUAVE.Methods.skip
    # segment.process.iterate.conditions.aerodynamics         = SUAVE.Methods.skip 
    # segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)
    
    # # ------------------------------------------------------------------
    # #   Cruise: Constant Speed, Constant Altitude
    # # ------------------------------------------------------------------
    
    total_t = 0
    speed = 0
    for i in range(tj1.shape[0]-1):
        total_t = tj1[i+1,4] - tj1[i,4]
        speed = np.linalg.norm(tj1[i,2:4])
        
        segment                                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
        segment.tag                                      = "cruise_1"+str(i)
        segment.analyses.extend( analyses.base ) 
        segment.altitude                                 = 2000.0 * Units.ft
        segment.air_speed                                = speed * Units['m/s'] #110.   * Units['mph']
        segment.distance                                 = total_t * speed * Units['m'] #50.    * Units.miles
        segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
        segment.process.finalize.post_process.stability         = SUAVE.Methods.skip
        segment.process.iterate.conditions.aerodynamics         = SUAVE.Methods.skip 
        segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(np.nan)
    start_altitude.append(segment.altitude)
    end_altitude.append(segment.altitude)
    # start_altitude.append(np.nan)
    # end_altitude.append(np.nan)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(segment.air_speed)
    # time_required.append(np.nan)
    time_required.append(total_t)
    
    
    
    # ------------------------------------------------------------------
    #   Hover Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                                 = Segments.Hover.Hover(base_segment)
    segment.tag                                             = "Hover_2_1"
    segment.analyses.extend( analyses.base)
    segment.altitude                                        = 2000.  * Units.ft
    segment.time                                            = (1/3)*60
    segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability         = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics         = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(np.nan)
    start_altitude.append(np.nan)
    end_altitude.append(np.nan)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(np.nan)
    time_required.append(np.nan)
    
    
    
    # ------------------------------------------------------------------
    #   Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                                 = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag                                             = "Accel_descent_1"
    segment.analyses.extend( analyses.base)
    segment.air_speed                                       = 29.484364 * Units.knots
    segment.altitude_end                                    = 500. * Units.ft
    segment.descent_angle                                   = 15 * Units.degrees
    segment.descent_rate                                    = 800. * Units['ft/min']
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(segment.descent_rate)
    start_altitude.append(segment.altitude_start)
    end_altitude.append(segment.altitude_end)
    climb_angle.append(np.nan)
    descent_angle.append(segment.descent_angle)
    speed_spec.append(segment.air_speed)
    time_required.append(np.nan)
    
    
    
    
    # ------------------------------------------------------------------
    #   last descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                               = Segments.Hover.Descent(base_segment)
    segment.tag                                           = "Hover_Descent_1"
    segment.analyses.extend( analyses.base)
    segment.altitude_start                                = 500.  * Units.ft
    segment.altitude_end                                  = 0.  * Units.ft
    segment.descent_rate                                  = 100. * Units['ft/min']
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,initial_power_coefficient=0.02)

    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(segment.descent_rate)
    start_altitude.append(segment.altitude_start)
    end_altitude.append(segment.altitude_end)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(np.nan)
    time_required.append(np.nan)
    
    
    
    
    # ------------------------------------------------------------------
    #  Charge Segment: 
    # ------------------------------------------------------------------  
    # Charge Model 
    segment                                                  = Segments.Ground.Battery_Charge_Discharge(base_segment)     
    segment.tag                                              = 'Charge_1'
    segment.analyses.extend(analyses.base)           
    segment.battery_discharge                                = False    
    segment.increment_battery_cycle_day                      = True 
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip        
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)    

    # add to misison
    mission.append_segment(segment) 
    
    
    # ###################### Third mission ###########################################################################
    # #----------------------------------------------------------------------------------------------------------------
    # #----------------------------------------------------------------------------------------------------------------
    # #----------------------------------------------------------------------------------------------------------------
    # #----------------------------------------------------------------------------------------------------------------
    
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                               = Segments.Hover.Climb(base_segment)
    segment.tag                                           = "Hover_Climb_2"
    segment.analyses.extend( analyses.base)
    segment.altitude_start                                = 0.0  * Units.ft
    segment.altitude_end                                  = 500.  * Units.ft
    segment.climb_rate                                    = 100. * Units['ft/min']
    segment.battery_energy                                = vehicle.networks.battery_propeller.battery.max_energy
    segment.state.unknowns.throttle                       = 0.6 * ones_row(1)
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,initial_power_coefficient=0.02)

    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(segment.climb_rate)
    descend_rate.append(np.nan)
    start_altitude.append(segment.altitude_start)
    end_altitude.append(segment.altitude_end)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(np.nan)
    time_required.append(np.nan)


    
    # ------------------------------------------------------------------
    #   Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                                 = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag                                             = "Accel_Climb_2"
    segment.analyses.extend( analyses.base)
    segment.air_speed                                       = 29.484364 * Units.knots
    segment.altitude_end                                    = 2000. * Units.ft
    segment.climb_angle                                     = 15 * Units.degrees
    segment.climb_rate                                      = 800. * Units['ft/min']
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(segment.climb_rate)
    descend_rate.append(np.nan)
    start_altitude.append(segment.altitude_start)
    end_altitude.append(segment.altitude_end)
    climb_angle.append(segment.climb_angle)
    descent_angle.append(np.nan)
    speed_spec.append(segment.air_speed)
    time_required.append(np.nan)
    
    
    # ------------------------------------------------------------------
    #   Hover Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                                 = Segments.Hover.Hover(base_segment)
    segment.tag                                             = "Hover_2"
    segment.analyses.extend( analyses.base)
    segment.altitude                                        = 2000.  * Units.ft
    segment.time                                            = (1/3)*60
    segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability         = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics         = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(np.nan)
    start_altitude.append(np.nan)
    end_altitude.append(np.nan)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(np.nan)
    time_required.append(np.nan)
    
    
    
    # # ------------------------------------------------------------------
    # #   Cruise: Constant Speed, Constant Altitude
    # # ------------------------------------------------------------------
    # segment                                                 = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    # segment.tag                                             = "Cruise"
    # segment.analyses.extend( analyses.base)
    # segment.air_speed                                       = 53.8765  * Units.knots
    # segment.distance                                        = 9.7246* Units.nautical_miles
    # segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    # segment.process.finalize.post_process.stability         = SUAVE.Methods.skip
    # segment.process.iterate.conditions.aerodynamics         = SUAVE.Methods.skip 
    # segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)
    
    # # ------------------------------------------------------------------
    # #   Cruise: Constant Speed, Constant Altitude
    # # ------------------------------------------------------------------
    
    total_t = 0
    speed = 0
    for i in range(tj2.shape[0]-1):
        total_t = tj2[i+1,4] - tj2[i,4]
        speed = np.linalg.norm(tj2[i,2:4])
        
        segment                                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
        segment.tag                                      = "cruise_2"+str(i)
        segment.analyses.extend( analyses.base ) 
        segment.altitude                                 = 2000.0 * Units.ft
        segment.air_speed                                = speed * Units['m/s'] #110.   * Units['mph']
        segment.distance                                 = total_t * speed * Units['m'] #50.    * Units.miles
        segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
        segment.process.finalize.post_process.stability         = SUAVE.Methods.skip
        segment.process.iterate.conditions.aerodynamics         = SUAVE.Methods.skip 
        segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(np.nan)
    start_altitude.append(segment.altitude)
    end_altitude.append(segment.altitude)
    # start_altitude.append(np.nan)
    # end_altitude.append(np.nan)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(segment.air_speed)
    # time_required.append(np.nan)
    time_required.append(total_t)
    
    
    
    # ------------------------------------------------------------------
    #   Hover Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                                 = Segments.Hover.Hover(base_segment)
    segment.tag                                             = "Hover_2_2"
    segment.analyses.extend( analyses.base)
    segment.altitude                                        = 2000.  * Units.ft
    segment.time                                            = (1/3)*60
    segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability         = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics         = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(np.nan)
    start_altitude.append(np.nan)
    end_altitude.append(np.nan)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(np.nan)
    time_required.append(np.nan)
    
    
    
    # ------------------------------------------------------------------
    #   Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                                 = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag                                             = "Accel_descent_2"
    segment.analyses.extend( analyses.base)
    segment.air_speed                                       = 29.484364 * Units.knots
    segment.altitude_end                                    = 500. * Units.ft
    segment.descent_angle                                   = 15 * Units.degrees
    segment.descent_rate                                    = 800. * Units['ft/min']
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(segment.descent_rate)
    start_altitude.append(segment.altitude_start)
    end_altitude.append(segment.altitude_end)
    climb_angle.append(np.nan)
    descent_angle.append(segment.descent_angle)
    speed_spec.append(segment.air_speed)
    time_required.append(np.nan)
    
    
    
    
    # ------------------------------------------------------------------
    #   last descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                               = Segments.Hover.Descent(base_segment)
    segment.tag                                           = "Hover_Descent_2"
    segment.analyses.extend( analyses.base)
    segment.altitude_start                                = 500.  * Units.ft
    segment.altitude_end                                  = 0.  * Units.ft
    segment.descent_rate                                  = 100. * Units['ft/min']
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,initial_power_coefficient=0.02)

    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(segment.descent_rate)
    start_altitude.append(segment.altitude_start)
    end_altitude.append(segment.altitude_end)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(np.nan)
    time_required.append(np.nan)
    
    
    
    
    
    # ------------------------------------------------------------------
    #  Charge Segment: 
    # ------------------------------------------------------------------  
    # Charge Model 
    segment                                                  = Segments.Ground.Battery_Charge_Discharge(base_segment)     
    segment.tag                                              = 'Charge_2'
    segment.analyses.extend(analyses.base)           
    segment.battery_discharge                                = False    
    segment.increment_battery_cycle_day                      = True 
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip        
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)    

    # add to misison
    mission.append_segment(segment) 
    
    
    # ###################### Fourth mission ###########################################################################
    # #----------------------------------------------------------------------------------------------------------------
    # #----------------------------------------------------------------------------------------------------------------
    # #----------------------------------------------------------------------------------------------------------------
    # #----------------------------------------------------------------------------------------------------------------
    
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                               = Segments.Hover.Climb(base_segment)
    segment.tag                                           = "Hover_Climb_3"
    segment.analyses.extend( analyses.base)
    segment.altitude_start                                = 0.0  * Units.ft
    segment.altitude_end                                  = 500.  * Units.ft
    segment.climb_rate                                    = 100. * Units['ft/min']
    segment.battery_energy                                = vehicle.networks.battery_propeller.battery.max_energy
    segment.state.unknowns.throttle                       = 0.6 * ones_row(1)
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,initial_power_coefficient=0.02)

    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(segment.climb_rate)
    descend_rate.append(np.nan)
    start_altitude.append(segment.altitude_start)
    end_altitude.append(segment.altitude_end)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(np.nan)
    time_required.append(np.nan)


    
    # ------------------------------------------------------------------
    #   Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                                 = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag                                             = "Accel_Climb_3"
    segment.analyses.extend( analyses.base)
    segment.air_speed                                       = 29.484364 * Units.knots
    segment.altitude_end                                    = 2000. * Units.ft
    segment.climb_angle                                     = 15 * Units.degrees
    segment.climb_rate                                      = 800. * Units['ft/min']
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(segment.climb_rate)
    descend_rate.append(np.nan)
    start_altitude.append(segment.altitude_start)
    end_altitude.append(segment.altitude_end)
    climb_angle.append(segment.climb_angle)
    descent_angle.append(np.nan)
    speed_spec.append(segment.air_speed)
    time_required.append(np.nan)
    
    
    # ------------------------------------------------------------------
    #   Hover Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                                 = Segments.Hover.Hover(base_segment)
    segment.tag                                             = "Hover_3"
    segment.analyses.extend( analyses.base)
    segment.altitude                                        = 2000.  * Units.ft
    segment.time                                            = (1/3)*60
    segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability         = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics         = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(np.nan)
    start_altitude.append(np.nan)
    end_altitude.append(np.nan)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(np.nan)
    time_required.append(np.nan)
    
    
    
    # # ------------------------------------------------------------------
    # #   Cruise: Constant Speed, Constant Altitude
    # # ------------------------------------------------------------------
    # segment                                                 = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    # segment.tag                                             = "Cruise"
    # segment.analyses.extend( analyses.base)
    # segment.air_speed                                       = 53.8765  * Units.knots
    # segment.distance                                        = 9.7246* Units.nautical_miles
    # segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    # segment.process.finalize.post_process.stability         = SUAVE.Methods.skip
    # segment.process.iterate.conditions.aerodynamics         = SUAVE.Methods.skip 
    # segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)
    
    # # ------------------------------------------------------------------
    # #   Cruise: Constant Speed, Constant Altitude
    # # ------------------------------------------------------------------
    
    total_t = 0
    speed = 0
    for i in range(tj3.shape[0]-1):
        total_t = tj3[i+1,4] - tj3[i,4]
        speed = np.linalg.norm(tj3[i,2:4])
        
        segment                                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
        segment.tag                                      = "cruise_3"+str(i)
        segment.analyses.extend( analyses.base ) 
        segment.altitude                                 = 2000.0 * Units.ft
        segment.air_speed                                = speed * Units['m/s'] #110.   * Units['mph']
        segment.distance                                 = total_t * speed * Units['m'] #50.    * Units.miles
        segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
        segment.process.finalize.post_process.stability         = SUAVE.Methods.skip
        segment.process.iterate.conditions.aerodynamics         = SUAVE.Methods.skip 
        segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(np.nan)
    start_altitude.append(segment.altitude)
    end_altitude.append(segment.altitude)
    # start_altitude.append(np.nan)
    # end_altitude.append(np.nan)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(segment.air_speed)
    # time_required.append(np.nan)
    time_required.append(total_t)
    
    
    
    # ------------------------------------------------------------------
    #   Hover Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                                 = Segments.Hover.Hover(base_segment)
    segment.tag                                             = "Hover_2_3"
    segment.analyses.extend( analyses.base)
    segment.altitude                                        = 2000.  * Units.ft
    segment.time                                            = (1/3)*60
    segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability         = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics         = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(np.nan)
    start_altitude.append(np.nan)
    end_altitude.append(np.nan)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(np.nan)
    time_required.append(np.nan)
    
    
    
    # ------------------------------------------------------------------
    #   Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                                 = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag                                             = "Accel_descent_3"
    segment.analyses.extend( analyses.base)
    segment.air_speed                                       = 29.484364 * Units.knots
    segment.altitude_end                                    = 500. * Units.ft
    segment.descent_angle                                   = 15 * Units.degrees
    segment.descent_rate                                    = 800. * Units['ft/min']
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(segment.descent_rate)
    start_altitude.append(segment.altitude_start)
    end_altitude.append(segment.altitude_end)
    climb_angle.append(np.nan)
    descent_angle.append(segment.descent_angle)
    speed_spec.append(segment.air_speed)
    time_required.append(np.nan)
    
    
    
    
    
    # ------------------------------------------------------------------
    #   last descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                               = Segments.Hover.Descent(base_segment)
    segment.tag                                           = "Hover_Descent_3"
    segment.analyses.extend( analyses.base)
    segment.altitude_start                                = 500.  * Units.ft
    segment.altitude_end                                  = 0.  * Units.ft
    segment.descent_rate                                  = 100. * Units['ft/min']
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,initial_power_coefficient=0.02)

    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(segment.descent_rate)
    start_altitude.append(segment.altitude_start)
    end_altitude.append(segment.altitude_end)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(np.nan)
    time_required.append(np.nan)
    
    
    
    
    
    # ------------------------------------------------------------------
    #  Charge Segment: 
    # ------------------------------------------------------------------  
    # Charge Model 
    segment                                                  = Segments.Ground.Battery_Charge_Discharge(base_segment)     
    segment.tag                                              = 'Charge_3'
    segment.analyses.extend(analyses.base)           
    segment.battery_discharge                                = False    
    segment.increment_battery_cycle_day                      = True 
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip        
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)    

    # add to misison
    mission.append_segment(segment) 
    
    
    # ###################### Fifth mission ###########################################################################
    # #----------------------------------------------------------------------------------------------------------------
    # #----------------------------------------------------------------------------------------------------------------
    # #----------------------------------------------------------------------------------------------------------------
    # #----------------------------------------------------------------------------------------------------------------
    
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                               = Segments.Hover.Climb(base_segment)
    segment.tag                                           = "Hover_Climb_4"
    segment.analyses.extend( analyses.base)
    segment.altitude_start                                = 0.0  * Units.ft
    segment.altitude_end                                  = 500.  * Units.ft
    segment.climb_rate                                    = 100. * Units['ft/min']
    segment.battery_energy                                = vehicle.networks.battery_propeller.battery.max_energy
    segment.state.unknowns.throttle                       = 0.6 * ones_row(1)
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,initial_power_coefficient=0.02)

    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(segment.climb_rate)
    descend_rate.append(np.nan)
    start_altitude.append(segment.altitude_start)
    end_altitude.append(segment.altitude_end)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(np.nan)
    time_required.append(np.nan)


    
    # ------------------------------------------------------------------
    #   Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                                 = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag                                             = "Accel_Climb_4"
    segment.analyses.extend( analyses.base)
    segment.air_speed                                       = 29.484364 * Units.knots
    segment.altitude_end                                    = 2000. * Units.ft
    segment.climb_angle                                     = 15 * Units.degrees
    segment.climb_rate                                      = 800. * Units['ft/min']
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(segment.climb_rate)
    descend_rate.append(np.nan)
    start_altitude.append(segment.altitude_start)
    end_altitude.append(segment.altitude_end)
    climb_angle.append(segment.climb_angle)
    descent_angle.append(np.nan)
    speed_spec.append(segment.air_speed)
    time_required.append(np.nan)
    
    
    # ------------------------------------------------------------------
    #   Hover Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                                 = Segments.Hover.Hover(base_segment)
    segment.tag                                             = "Hover_4"
    segment.analyses.extend( analyses.base)
    segment.altitude                                        = 2000.  * Units.ft
    segment.time                                            = (1/3)*60
    segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability         = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics         = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(np.nan)
    start_altitude.append(np.nan)
    end_altitude.append(np.nan)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(np.nan)
    time_required.append(np.nan)
    
    
    
    # # ------------------------------------------------------------------
    # #   Cruise: Constant Speed, Constant Altitude
    # # ------------------------------------------------------------------
    # segment                                                 = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    # segment.tag                                             = "Cruise"
    # segment.analyses.extend( analyses.base)
    # segment.air_speed                                       = 53.8765  * Units.knots
    # segment.distance                                        = 9.7246* Units.nautical_miles
    # segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    # segment.process.finalize.post_process.stability         = SUAVE.Methods.skip
    # segment.process.iterate.conditions.aerodynamics         = SUAVE.Methods.skip 
    # segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)
    
    # # ------------------------------------------------------------------
    # #   Cruise: Constant Speed, Constant Altitude
    # # ------------------------------------------------------------------
    
    total_t = 0
    speed = 0
    for i in range(tj4.shape[0]-1):
        total_t = tj4[i+1,4] - tj4[i,4]
        speed = np.linalg.norm(tj4[i,2:4])
        
        segment                                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
        segment.tag                                      = "cruise_4"+str(i)
        segment.analyses.extend( analyses.base ) 
        segment.altitude                                 = 2000.0 * Units.ft
        segment.air_speed                                = speed * Units['m/s'] #110.   * Units['mph']
        segment.distance                                 = total_t * speed * Units['m'] #50.    * Units.miles
        segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
        segment.process.finalize.post_process.stability         = SUAVE.Methods.skip
        segment.process.iterate.conditions.aerodynamics         = SUAVE.Methods.skip 
        segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(np.nan)
    start_altitude.append(segment.altitude)
    end_altitude.append(segment.altitude)
    # start_altitude.append(np.nan)
    # end_altitude.append(np.nan)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(segment.air_speed)
    # time_required.append(np.nan)
    time_required.append(total_t)
    
    
    
    # ------------------------------------------------------------------
    #   Hover Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                                 = Segments.Hover.Hover(base_segment)
    segment.tag                                             = "Hover_2_4"
    segment.analyses.extend( analyses.base)
    segment.altitude                                        = 2000.  * Units.ft
    segment.time                                            = (1/3)*60
    segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability         = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics         = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(np.nan)
    start_altitude.append(np.nan)
    end_altitude.append(np.nan)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(np.nan)
    time_required.append(np.nan)
    
    
    
    # ------------------------------------------------------------------
    #   Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                                 = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag                                             = "Accel_descent_4"
    segment.analyses.extend( analyses.base)
    segment.air_speed                                       = 29.484364 * Units.knots
    segment.altitude_end                                    = 500. * Units.ft
    segment.descent_angle                                   = 15 * Units.degrees
    segment.descent_rate                                    = 800. * Units['ft/min']
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)


    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(segment.descent_rate)
    start_altitude.append(segment.altitude_start)
    end_altitude.append(segment.altitude_end)
    climb_angle.append(np.nan)
    descent_angle.append(segment.descent_angle)
    speed_spec.append(segment.air_speed)
    time_required.append(np.nan)
    
    
    
    
    
    # ------------------------------------------------------------------
    #   last descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                               = Segments.Hover.Descent(base_segment)
    segment.tag                                           = "Hover_Descent_4"
    segment.analyses.extend( analyses.base)
    segment.altitude_start                                = 500.  * Units.ft
    segment.altitude_end                                  = 0.  * Units.ft
    segment.descent_rate                                  = 100. * Units['ft/min']
    segment.process.iterate.conditions.stability          = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability       = SUAVE.Methods.skip
    segment.process.iterate.conditions.aerodynamics       = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,initial_power_coefficient=0.02)

    # add to misison
    mission.append_segment(segment)
    
    segment_type.append(segment.tag)
    climb_rate.append(np.nan)
    descend_rate.append(segment.descent_rate)
    start_altitude.append(segment.altitude_start)
    end_altitude.append(segment.altitude_end)
    climb_angle.append(np.nan)
    descent_angle.append(np.nan)
    speed_spec.append(np.nan)
    time_required.append(np.nan)
    
    
    
    profile_spec_df = pd.DataFrame(data={'segment_type':segment_type,
                                    'climb_rate':climb_rate,
                                    'descend_rate':descend_rate,
                                    'start_altitude':start_altitude,
                                    'end_altitude':end_altitude,
                                    'climb_angle':climb_angle,
                                    'descent_angle':descent_angle,
                                    'speed':speed_spec,
                                    'time_required':time_required
                                    })
    
    # base_path = "./logs/profiles_eval/"
    # base_path = ".SUAVE-develop/prof/"
    # profile_spec_df.to_csv(base_path+"data"'.csv', index=False)
    profile_spec_df.to_csv(r'path\profile.csv', index=False)
    

    return mission


def missions_setup(base_mission):

    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    missions.base = base_mission


    # done!
    return missions


# ----------------------------------------------------------------------
#   Plot Results
# ----------------------------------------------------------------------
def plot_mission(results,line_style='bo-'):

    # Plot Flight Conditions
    plot_flight_conditions(results, line_style)

    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results, line_style)

    # Plot Aircraft Flight Speed
    plot_aircraft_velocities(results, line_style)

    # Plot Aircraft Electronics
    plot_battery_pack_conditions(results, line_style)

    # Plot Propeller Conditions
    plot_propeller_conditions(results, line_style)

    # Plot Electric Motor and Propeller Efficiencies
    plot_eMotor_Prop_efficiencies(results, line_style)

    # Plot propeller Disc and Power Loading
    plot_disc_power_loading(results, line_style)

    return



def load_multicopter_results():
    return SUAVE.Input_Output.SUAVE.load('results_multicopter.res')

def save_multicopter_results(results):
    SUAVE.Input_Output.SUAVE.archive(results,'results_multicopter.res')
    return

####################################### Data Generation ################################################
####################################### Data Generation ################################################
def save_results(results, profile_id=0):
    base_path = "./data/performance/"
    #flight conditions
    eVTOL_type = 'Multicopter'
    altitudes = []
    airspeeds = []
    densities = []
    # tempera = []
    thetas = []
    #climb_angles = []
    ranges = []
    times = []
    eVTOL_type_profile = []
    mission_segment_profile = []
    fesibility_profile = []
    for label, segment in zip(results.segments.keys(), results.segments.values()): 
        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        airspeed = segment.conditions.freestream.velocity[:,0] /   Units['mph'] 
        density  = segment.conditions.freestream.density[:,0]
        # temp     = segment.conditions.freestream.temperature[:,0]
        theta    = segment.conditions.frames.body.inertial_rotations[:,1,None] / Units.deg
        
        x        = segment.conditions.frames.inertial.position_vector[:,0]/ Units.nmi
        y        = segment.conditions.frames.inertial.position_vector[:,1]
        z        = segment.conditions.frames.inertial.position_vector[:,2]
        altitude = segment.conditions.freestream.altitude[:,0]/Units.feet
        
        altitudes += list(altitude)
        airspeeds += list(airspeed)
        densities += list(density)
        # tempera   += list(temp)
        thetas    += list(np.squeeze(theta)) 
       # climb_angles  += list(np.squeeze(climb_angle))
        ranges    += list(x)
        times     += list(time)
        eVTOL_type_profile += [eVTOL_type]*len(list(time))
        mission_segment_profile += [label]*len(list(time))
        fesibility_profile += [segment.converged] * len(list(time))
        
    profile_df = pd.DataFrame(data={'eVTOL_type':eVTOL_type_profile,
                                    'mission_segment':mission_segment_profile,
                                    'altitude_ft':altitudes,
                                    'air_speed_mph':airspeeds,
                                    'pitch_angle_deg':thetas,
                                    'Air_density':densities,
                                    # 'temperature_':tempera,
                                    'range_nmi':ranges,
                                    'time_min':times,
                                    'fesibility':fesibility_profile
                                    })
    
    
    profile_df.to_csv(base_path+"profile_flight_conditions_"+str(profile_id)+'.csv', index=False)
    
     #Aerodynamic Coefficients
    CLs = []
    CDs = []
    AOAs = []
    L_Ds = []
    times = []
    eVTOL_type_profile = []
    mission_segment_profile = []
    fesibility_profile = []
    for label, segment in zip(results.segments.keys(), results.segments.values()): 
       time     = segment.conditions.frames.inertial.time[:,0] / Units.min
       cl   = segment.conditions.aerodynamics.lift_coefficient[:,0,None] 
       cd   = segment.conditions.aerodynamics.drag_coefficient[:,0,None] 
       aoa  = segment.conditions.aerodynamics.angle_of_attack[:,0] / Units.deg
       l_d  = cl/cd
       
       CLs     += list(np.squeeze(cl))
       CDs     += list(np.squeeze(cd))
       AOAs    += list(aoa) 
       L_Ds    += list(np.squeeze(l_d))
       times   += list(time)
       eVTOL_type_profile += [eVTOL_type]*len(list(time))
       mission_segment_profile += [label]*len(list(time))
       fesibility_profile += [segment.converged] * len(list(time))
       
    profile_df = pd.DataFrame(data={'eVTOL_type':eVTOL_type_profile,
                                   'mission_segment':mission_segment_profile,
                                   'CL':CLs,
                                   'CD':CDs,
                                   'AOA_deg':AOAs,
                                   'L_D':L_Ds,
                                   'time_min':times,
                                   'fesibility':fesibility_profile
                                   })
   
   
    profile_df.to_csv(base_path+"profile_aerodynamic_coefficients_"+str(profile_id)+'.csv', index=False)
    
    # Aircraft Electronics
    socs    = []
    energys = []
    powers  = []
    voltages = []
    voltages_oc = []
    currents = []
    cratings_instant = []
    cratings_nominal = []
    specific_powers = []
    times = []
    eVTOL_type_profile = []
    mission_segment_profile = []
    fesibility_profile = []
    for label, segment in zip(results.segments.keys(), results.segments.values()): 
       time     = segment.conditions.frames.inertial.time[:,0] / Units.min
       
       pack_power          = segment.conditions.propulsion.battery_power_draw[:,0] 
       pack_energy         = segment.conditions.propulsion.battery_energy[:,0] 
       pack_volts          = segment.conditions.propulsion.battery_voltage_under_load[:,0] 
       pack_volts_oc       = segment.conditions.propulsion.battery_voltage_open_circuit[:,0]     
       pack_current        = segment.conditions.propulsion.battery_current[:,0]   
       pack_SOC            = segment.conditions.propulsion.battery_state_of_charge[:,0]  
       specific_power      = segment.conditions.propulsion.battery_specfic_power[:,0]  
       
       
       pack_battery_amp_hr = (pack_energy/ Units.Wh )/pack_volts  
       pack_C_instant      = pack_current/pack_battery_amp_hr
       pack_C_nominal      = pack_current/np.max(pack_battery_amp_hr)
       
   
       socs             += list(pack_SOC)  
       energys          += list((pack_energy/Units.Wh)/1000)     
       powers           += list(-pack_power/1000)        
       voltages         += list(pack_volts) 
       voltages_oc      += list(pack_volts_oc)
       currents         += list(pack_current)
       cratings_instant += list(pack_C_instant)
       cratings_nominal += list(pack_C_nominal)
       specific_powers  += list(specific_power)
       
       times         += list(time)
       eVTOL_type_profile += [eVTOL_type]*len(list(time))
       mission_segment_profile += [label]*len(list(time))
       fesibility_profile += [segment.converged] * len(list(time))
       
    profile_df = pd.DataFrame(data={'eVTOL_type':eVTOL_type_profile,
                                   'mission_segment':mission_segment_profile,
                                   'SOC':socs,
                                   'battery_energy_kw_h':energys,
                                   'battery_power_kw':powers,
                                   'voltage_v':voltages,
                                   'voltage_oc_v':voltages_oc,
                                   'current_a':currents,
                                   'C_Rating_instant':cratings_instant,
                                   'C_Rating_nominal':cratings_nominal,
                                   'specific_power':specific_powers,
                                   'time_min':times,
                                   'fesibility':fesibility_profile
                                   })
   
   
    profile_df.to_csv(base_path+"profile_aircraft_electronics_"+str(profile_id)+'.csv', index=False)
    
    

if __name__ == '__main__':
    main()
    plt.show(block=True)