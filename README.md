# Optimizer
Project to Implement different Simulated Annealing Optimizers

# Dependencies/Libraries
The project uses python to visualize the solutions

## logeru
The logging library loguru is also used
https://github.com/emilk/loguru

it is provided in this repository and requires no installation

# File formats
Problems and Solutions are stored and read in from text files.

During the parsing of the files the first continous block of lines starting with '#' will be ignored as comments.
## Problem description
A problem consists of Tasks (arbitrarily many).

A Task consists of Steps (arbitrarily many), each Step runs on a specific machine and takes a set amount of time.

This is an example of a Problem description:
```
3 5
6, 0 8, 3 6, 4 10, 0 3, 1 9, 2 5
4, 3 4, 0 12, 4 16, 3 8
7, 4 19, 2 8, 0 6, 1 10, 3 7, 1 5, 2 9
```
The first line contains "'task_count' 'machine_count'".

Each line afterwards corresponds to a **Task**. The first number specifies the number of Steps the Task has.

Separated by commas we have "'machine' 'duration'" pairs that define a Step. 'machine' has to be in the range [0,machine_count] and 'duration' must be >= 0.

The sequence of the steps on a line defines the precedence of the Steps.

All values must be integers. The comma and hyphen can be replaced by any character (only for readability).

## Solution description
This is a Solution for the Problem description given in the [above](#Problem-description):
```
SmallTestingSolution
3 5 66
4, 0 0 0 8 0 8, 1 1 0 12 8 20, 2 2 0 6 27 33, 0 3 0 3 33 36
3, 2 3 1 10 33 43, 0 4 1 9 43 52, 2 5 1 5 52 57
3, 2 1 2 8 19 27, 0 5 2 5 52 57, 2 6 2 9 57 66
4, 1 0 3 4 0 4, 0 1 3 6 8 14, 2 4 3 7 43 50, 1 3 3 8 50 58
3, 2 0 4 19 0 19, 0 2 4 10 19 29, 1 2 4 16 29 45
```
The first line is the name of the Solution/Problem. The Optimizer can set this value for clarity, the implemented optimizers will set it to "user_prefix-problem_filename-round-iteration".

The second line contains "'task_count' 'machine_count" the first two are the same as in the Problem description.

Each line afterwards corresponds to a **Machine**. The first number is the number of Steps the Machine executes.

Separated by commas we have SolutionStep tuples. The tuples have the format: "'taskId' 'StepIndex' 'machine' 'duration' 'startTime' 'endTime'". Again 'machine' has to be in the range [0,machine_count] and 'duration' must be >= 0.

Naturally the Steps and Tasks that can be constructed from the Solution must match the Problem it should solve. This can be checked with the Solution::validateSolution(Problem) function for arbitrary pairs of Solutions and Problems.

For solutions the comma and hyphen **cannot** be repalace because the the createGnatt.py script (used to visualize the solutions) relies on them to parse the file.

# Visualization
example:
python38 createGnatt.py "../JobShopSolutions/SmallTestingSolution.txt"