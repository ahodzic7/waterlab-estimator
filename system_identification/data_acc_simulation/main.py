from pyswmm import Simulation, Nodes, Links
import random
import pandas as pd
import matplotlib.pyplot as plt

columns = ['tank1_depth',
           'pipe1_depth',
           'pipe2_depth',
           'pipe3_depth',
           'pipe4_depth',
           'pump1_flow',
           'pump3_flow']

network_df = pd.DataFrame(columns=columns)

print(network_df)

with Simulation('C:\Git\waterlab-estimator\system_identification\epa_network\project_network.inp') as sim:
    time_step = 30  # sec
    sim.step_advance(time_step)

    # Control input
    pump1 = Links(sim)["FP1"]
    pump3 = Links(sim)["FP3"]
    # States
    tank1 = Nodes(sim)["T1"]
    pipe1 = Links(sim)["P1"]
    pipe2 = Links(sim)["P2"]
    pipe3 = Links(sim)["P3"]
    pipe4 = Links(sim)["P4"]
    tank2 = Nodes(sim)["T2"]

    # Init sim
    count = 1
    on_time = 0
    for idx, step in enumerate(sim):
        # Make sure the system is always supplied with water
        pump3.target_setting = 5
        # pump1.target_setting = 1

        # Create simple random controller
        if count > on_time:
            count = 0
            on_time = random.randint(5,15)
            pump1.target_setting = random.uniform(0, 1.5)
        else:
            count += 1
        network_df = network_df.append(pd.Series([tank1.depth, pipe1.depth, pipe2.depth, pipe3.depth,
                                                   pipe4.depth, pump1.flow,pump3.flow], index=network_df.columns),
                                                ignore_index=True)
        print(f"Progress {int(sim.percent_complete * 100)}%", end="\r")

    fig, axes = plt.subplots(nrows=2, ncols=1)

    network_df.plot(y='pipe1_depth', ax=axes[0])
    network_df.plot(y='pipe2_depth', ax=axes[0])
    network_df.plot(y='pipe3_depth', ax=axes[0])
    network_df.plot(y='pipe4_depth', ax=axes[0])
    network_df.plot(y='pump1_flow', ax=axes[1])

    plt.show()