import sys
import itertools
import matplotlib.pyplot as plt
import colorGenerator as ColGen

# Define the function that creates the Gantt chart
def create_gantt_chart(data):
    # Parse the data from the file   
    ids = []
    machines = []
    start_times = []
    duration = []
    max_t = 0

    name = data.pop(0)
    sizes = data.pop(0).split()

    for line in data:
        _,r = line.split('-', 1)
        steps = r.split(',')
        for s in steps:
            s.strip()
            if not s:
                continue
            t = list(map(int, s.split()))
            ids.append(t[0])
            machines.append(t[2])
            duration.append(t[3])
            start_times.append(t[4])
            if t[4] + t[3] > max_t:
                max_t = t[4] + t[3]

    # create color for each task
    colorsGen = list(itertools.islice(ColGen.rgbs(), int(sizes[0])))
    
    # Create the horizontal bar chart
    fig, ax = plt.subplots()
    bars = ax.barh(y=machines, width=duration, left=start_times, height=0.5,
                   color=[colorsGen[i] for i in ids], edgecolor='black')

    # Add labels inside each bar
    for i, bar in enumerate(bars):
        ax.text(bar.get_width() / 2 + bar.get_x(), bar.get_y() + bar.get_height() / 2,
                '{} ; {}'.format(ids[i], duration[i]),
                ha='center', va='center', color='white', fontsize=10)

    # Set the chart title and axis labels
    #filename = file_path.split('/')[-1]
    #plt.title(f'Gantt Chart for "{name}" in file "{filename}"')
    plt.title(f'Gantt Chart for "{name}"')

    # Set the x-axis limits
    plt.xlim(0, max_t)
    # Get x-axis minor ticks (only)
    ax.minorticks_on()
    ax.yaxis.set_tick_params(which='minor', bottom=False)
    ax.xaxis.set_minor_locator(plt.MultipleLocator(5))
    # create y-axix labels
    ax.invert_yaxis()
    yLabels = ['Machine ' + str(id) for id in machines]
    ax.set_yticklabels(yLabels)

    # Display the chart
    plt.show()

# Get the file path argument from the command line
file_path = sys.argv[1]

# Read the data from the file, ignore comment lines
with open(file_path, 'r') as f:
    data = [line.strip() for line in f if not line.startswith('#')]

# Create the Gantt chart
create_gantt_chart(data)

