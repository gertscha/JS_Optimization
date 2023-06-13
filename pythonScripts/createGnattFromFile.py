import sys
import itertools
import math
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from os.path import dirname, abspath

# Get the absolute path of the current script
current_folder = dirname(abspath(__file__))
# Add the current folder to the system path
sys.path.append(current_folder)

import colorGenerator as ColGen


# Define the function that creates the Gantt chart
def create_gantt_chart(data):
    # variables to hold the data for the bars
    ids = [] # defines color
    indices = [] # only needed to label it
    machines = [] # defines row
    start_times = [] # defines postion
    duration = [] # defines length and used for labeling
    max_t = 0 # maximum value (x-axis)

    # parse basic parameters
    name = data.pop(0)
    sizes = data.pop(0).split()

    # Parse the data for the bars from the file
    for line in data:
        _,r = line.split(',', 1)
        steps = r.split(',')
        for s in steps:
            s.strip()
            if not s:
                continue
            t = list(map(int, s.split()))
            ids.append(t[0])
            indices.append(t[1])
            machines.append(t[2])
            duration.append(t[3])
            start_times.append(t[4])
            if t[4] + t[3] > max_t:
                max_t = t[4] + t[3]

    # create color for each task (see colorGenerator.py, source: https://stackoverflow.com/a/13781114)
    colorsGen = list(itertools.islice(ColGen.rgbs(), int(sizes[0])))

    # Create the horizontal bar chart
    fig, ax = plt.subplots(1,1, figsize=(12, 8))
    bars = ax.barh(y=machines, width=duration, left=start_times, height=0.5,
                   color=[colorsGen[i] for i in ids], edgecolor='black')

    # prepare legend data
    legend_nextId = 0
    legend_handles = []
    legend_labels = []
    # fill out legend handles and labels
    while legend_nextId < int(sizes[0]):
        for i, bar in enumerate(bars):
            if ids[i] == legend_nextId:
                legend_handles.append(bar)
                legend_labels.append(f'Task {ids[i]}')
                legend_nextId += 1
    # create legend
    legend = plt.legend(legend_handles, legend_labels, bbox_to_anchor=(1.01, 1))
    legend.set_draggable(True)

    # Add labels inside each bar
    for i, bar in enumerate(bars):
        ax.text(bar.get_width() / 2 + bar.get_x(), bar.get_y() + bar.get_height() / 2,
                '({},{})\n{}'.format(ids[i], indices[i], duration[i]),
                ha='center', va='center', color='white', fontsize=8)


    # Set the chart title and axis labels
    #filename = file_path.split('/')[-1]
    #plt.title(f'Gantt Chart for "{name}" in file "{filename}"')
    plt.title(f'Gantt Chart for "{name}"')
    
    # configure the x-axis
    ax.set_xlabel("Time [arb. unit]")
    ax.minorticks_on()
    ax.xaxis.set_major_locator(plt.AutoLocator())
    ax.xaxis.set_minor_locator(ticker.AutoMinorLocator())
    # add makespan label
    auto_xtick_labels = ax.get_xticklabels()
    auto_xticks_val = ax.get_xticks().tolist()
    max_label_val_diff = abs(int(auto_xticks_val[-1]) - max_t) / 2.0
    new_ticks = auto_xticks_val
    new_labels = [label.get_text() for label in auto_xtick_labels]
    # filter ticks that are too big out
    filtered_tick = [pos for pos in new_ticks if pos < (max_t - max_label_val_diff)] + [float(max_t)]
    filtered_labels = [label for label, pos in zip(new_labels, new_ticks) if pos < (max_t - max_label_val_diff)] + [str(max_t)]
    # set the new labels
    ax.set_xticks(filtered_tick)
    ax.set_xticklabels(filtered_labels)

    ax.format_coord = lambda x, y: 'x={:g}, y={:g}'.format(math.floor(x + 0.5), math.floor(y + 0.5))

    # configure y-axix
    ax.yaxis.set_tick_params(which='minor', bottom=False)
    ax.yaxis.set_ticks(list(range(0, int(sizes[1]), 1)))
    ax.set_ylabel("Machine")
    ax.invert_yaxis()
    # different format, disable ylabel 
    #ax.yaxis.set_ticks(list(range(int(sizes[1]))), ['Machine ' + str(id) for id in list(range(int(sizes[1])))])
    
    # create background grid
    ax.set_axisbelow(True)
    ax.xaxis.grid(color='silver', which='both')
    
    # Display the chart
    plt.show()

# Get the file path argument from the command line
file_path = sys.argv[1]

# Read the data from the file, ignore comment lines
with open(file_path, 'r') as f:
    data = [line.strip() for line in f if not line.startswith('#')]

# Create the Gantt chart
create_gantt_chart(data)

