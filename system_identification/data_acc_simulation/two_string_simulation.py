from pyswmm import Simulation, Nodes, Links
from datetime import datetime
import random
import pandas as pd
import matplotlib.pyplot as plt

columns = ['time',
           'tank1_depth',
           'pipe1_depth',
           'pipe2_depth',
           'pipe3_depth',
           'pipe4_depth',
           'tank2_inflow',
           'tank2_depth',
           'pump1_flow',
           'pump2_flow',
           'tank_area',
           'simulation_time']

network_df = pd.DataFrame(columns=columns)

with Simulation(r'../epa_network/two_string_simple_network.inp') as sim:
    time_step = 1  # number of ROUTING_STEP's to be simulated before returning to python
    sim.step_advance(time_step)

    # Network layout
    tank1 = Nodes(sim)["B2334099PSA256"]
    pump1 = Links(sim)["P1"]
    tank1_area = 6.539
    pipe1 = Links(sim)["L2747"]
    pipe2 = Links(sim)["L2746"]
    pipe3 = Links(sim)["L2745"]
    pipe4 = Links(sim)["L2921"]
    pipe5 = Links(sim)["L2797"]
    pipe6 = Links(sim)["L2798"]
    pipe7 = Links(sim)["L2799"]
    pipe8 = Links(sim)["L2793"]
    pipe9 = Links(sim)["L2792"]
    pipe10 = Links(sim)["L2791"]
    pipe11 = Links(sim)["L2379"]
    pipe12 = Links(sim)["L2378"]
    pipe13 = Links(sim)["L2377"]
    pipe14 = Links(sim)["L2374"]
    psudo_pipe = Nodes(sim)["B2323037O398"]
    psudo_pipe_area = 7.011
    pipe15 = Links(sim)["L2373"]
    pipe16 = Links(sim)["L6497"]
    pipe17 = Links(sim)["L2390"]
    pipe18 = Links(sim)["L2354"]
    pipe19 = Links(sim)["L2418"]
    tank2 = Nodes(sim)["B2323069"]
    tank2_area = 29.9432
    pump2 = Links(sim)["P2"]
    tank_offset = 1.1


    # Init sim
    count = 1
    on_time = 0
    total_count = 0
    pump_reference_flow = 1                     # [m^3/s]
    tank1.generated_inflow(2)
    for idx, step in enumerate(sim):
        # Make sure the system is always supplied with water

        # System identification setup:
        #

        # Outflow pump simple control in operating range:

        if tank2.depth > tank_offset+1.5:
            pump2.target_setting = 1*pump_reference_flow
        elif tank2.depth > tank_offset+1.8:
            pump2.target_setting = 1.3 * pump_reference_flow
        elif tank2.depth < tank_offset+0.2:
            pump2.target_setting = 0

        # Create simple random controller
        if count > on_time:
            count = 0
            on_time = random.randint(50, 100)
            pump1.target_setting = random.randint(0, 1) * pump_reference_flow
            # pump1.target_setting = random.uniform(0, 1)*pump_reference_flow
        # if tank2.depth > 0.9:
        #     pump1.target_setting = 0
        else:
            count += 1

        # Add info to dataframe
        total_outflow_tank2 = pump2.flow + tank2.flooding
        elapsed_time = total_count

        time_now = sim.current_time
        duration = time_now - sim.report_start
        duration_in_s = duration.total_seconds()
        network_df = network_df.append(pd.Series([elapsed_time, tank1.depth, pipe2.depth, pipe5.depth, pipe10.depth,
                                                  pipe18.depth, tank2.total_inflow, tank2.depth, pump1.flow, total_outflow_tank2,
                                                  tank2_area, duration_in_s], index=network_df.columns), ignore_index=True)
        total_count += 1
        print(f"Progress {int(sim.percent_complete * 100)}%", end="\r")

    fig, axes = plt.subplots(nrows=3, ncols=1)

    network_df.plot(x='time', y='pipe1_depth', ax=axes[0])
    network_df.plot(x='time', y='pipe2_depth', ax=axes[0])
    network_df.plot(x='time', y='pipe3_depth', ax=axes[0])
    network_df.plot(x='time', y='pipe4_depth', ax=axes[0])

    network_df.plot(x='time', y='pump1_flow', ax=axes[1])
    # network_df.plot(x='time', y='tank2_inflow', ax=axes[1])
    network_df.plot(x='time', y='pump2_flow', ax=axes[1])
    network_df.plot(x='time', y='tank2_depth', ax=axes[2])
    plt.show()
    network_df.to_csv(r'gen_data_output/nonlinear_tank_2.csv', index=False, header=True)
