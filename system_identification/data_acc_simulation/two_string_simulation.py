from pyswmm import Simulation, Nodes, Links
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
           'tank_area']

network_df = pd.DataFrame(columns=columns)

with Simulation(r'../epa_network/toSt.inp') as sim:

    time_step = 1  # number of ROUTING_STEP's to be simulated before returning to python
    sim.step_advance(time_step)

    # Control input
    pump1 = Links(sim)["FP1"]
    pump2 = Links(sim)["FP2"]
    pump3 = Links(sim)["FP3"]
    # States
    tank1 = Nodes(sim)["T1"]
    pipe1 = Links(sim)["P1"]
    pipe2 = Links(sim)["P2"]
    pipe3 = Links(sim)["P3"]
    pipe4 = Links(sim)["P4"]
    tank2 = Nodes(sim)["T2"]

    # Init sim
    tank_area = 200;
    count = 1
    on_time = 0
    total_count = 0
    pump_reference_flow = 1/5
    for idx, step in enumerate(sim):
        # Make sure the system is always supplied with water
        pump3.target_setting = 5

        # System identification setup:
        #
        pump2.target_setting = 0.3*pump_reference_flow
        # pump1.target_setting = 1*pump_reference_flow

        # Create simple random controller
        if count > on_time:
            count = 0
            on_time = random.randint(5, 15)
            pump1.target_setting = random.uniform(0, 1)*pump_reference_flow
        if tank2.depth > 0.9:
            pump1.target_setting = 0
        else:
            count += 1

        # Add info to dataframe
        total_outflow_tank2 = pump2.flow + tank2.flooding
        elapsed_time = total_count
        network_df = network_df.append(pd.Series([elapsed_time, tank1.depth, pipe1.depth, pipe2.depth, pipe3.depth,
                                                  pipe4.depth, tank2.total_inflow, tank2.depth, pump1.flow, total_outflow_tank2,
                                                  tank_area], index=network_df.columns), ignore_index=True)
        total_count += 1
        print(f"Progress {int(sim.percent_complete * 100)}%", end="\r")

    fig, axes = plt.subplots(nrows=3, ncols=1)

    network_df.plot(x='time', y='pipe1_depth', ax=axes[0])
    network_df.plot(x='time', y='pipe2_depth', ax=axes[0])
    network_df.plot(x='time', y='pipe3_depth', ax=axes[0])
    network_df.plot(x='time', y='pipe4_depth', ax=axes[0])

    network_df.plot(x='time', y='pump1_flow', ax=axes[1])
    network_df.plot(x='time', y='tank2_inflow', ax=axes[1])
    network_df.plot(x='time', y='pump2_flow', ax=axes[1])
    network_df.plot(x='time', y='tank2_depth', ax=axes[2])
    plt.show()
    network_df.to_csv(r'gen_data_output/new_data2.csv', index=False, header=True)
