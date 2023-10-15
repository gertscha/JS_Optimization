# helper to call the Google_OR_Tools.py script on many Problems
import os
import subprocess
import sys
from glob import glob


def capture_output(command):
  """Captures all the output from a command and returns it as a string."""
  output = subprocess.check_output(command, shell=True)
  return output

def write_output_to_file(output, filename):
  """Writes the output to a file."""
  with open(filename, "ba") as f:
    f.write(output)

# adjust this such that it calls the desired python installation
pyversion = 'python310'
call = ' Google_OR_Tools.py -r '
timeout = ' -t 240'

for filename in glob('../JobShopProblems/Instances/**/*.txt', recursive=True):
    command = pyversion + call + filename + timeout
    print(f'running: {command}')
    output = capture_output(command)
    write_output_to_file(output, "output.txt")

#os.system(command)
