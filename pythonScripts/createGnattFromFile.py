import sys
import itertools
import matplotlib.pyplot as plt
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
    legend = plt.legend(legend_handles, legend_labels, bbox_to_anchor=(1.04, 1))
    legend.set_draggable(True)

    # Add labels inside each bar
    for i, bar in enumerate(bars):
        ax.text(bar.get_width() / 2 + bar.get_x(), bar.get_y() + bar.get_height() / 2,
                '{}\n{}'.format(indices[i], duration[i]),
                ha='center', va='center', color='white', fontsize=10)


    # Set the chart title and axis labels
    #filename = file_path.split('/')[-1]
    #plt.title(f'Gantt Chart for "{name}" in file "{filename}"')
    plt.title(f'Gantt Chart for "{name}"')

    # configure the x-axis
    ax.set_xlabel("Time")
    # tick labels, on three scales
    xMultipleLocator = 5
    if max_t > 200:
        if max_t > 1000:
            xMultipleLocator = 100
        else:
            xMultipleLocator = 50
    ax.minorticks_on()
    ax.xaxis.set_minor_locator(plt.MultipleLocator(xMultipleLocator))
    ax.xaxis.set_ticks([0,xMultipleLocator] + list(range(2*xMultipleLocator, max_t, 2*xMultipleLocator)) + [max_t])
    ax.yaxis.set_tick_params(which='minor', bottom=False)
    
    # configure y-axix
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
