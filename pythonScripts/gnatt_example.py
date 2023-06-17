import matplotlib.pyplot as plt
import numpy as np

# Create some data
y = ['A', 'B', 'C']
x = np.array([5, 3, 8])

# Create a horizontal bar chart
fig, ax = plt.subplots()
bars = ax.barh(y=y, width=x)

# Add labels inside each bar
for i, bar in enumerate(bars):
    ax.text(bar.get_width() / 2, bar.get_y() + bar.get_height() / 2,
            '{}, {}'.format(0, i),
            ha='center', va='center', color='white', fontsize=10)

# Display the chart
plt.show()
