import collections
import sys
from os.path import dirname, abspath
import argparse
import multiprocessing
try:
    from ortools.sat.python import cp_model
except:
    print('make sure to have the Google OR Tools installed')
    print('use : "python -m pip install --upgrade --user ortools"')
    sys.exit()

# taken from https://developers.google.com/optimization/scheduling/job_shop?hl=en
def runORTools(jobs_data):

    machines_count = 1 + max(task[0] for job in jobs_data for task in job)
    all_machines = range(machines_count)
    # Computes horizon dynamically as the sum of all durations.
    horizon = sum(task[1] for job in jobs_data for task in job)

    # Create the model.
    model = cp_model.CpModel()

    # Named tuple to store information about created variables.
    task_type = collections.namedtuple('task_type', 'start end interval')
    # Named tuple to manipulate solution information.
    assigned_task_type = collections.namedtuple('assigned_task_type',
                                                'start job index duration')

    # Creates job intervals and add to the corresponding machine lists.
    all_tasks = {}
    machine_to_intervals = collections.defaultdict(list)

    for job_id, job in enumerate(jobs_data):
        for task_id, task in enumerate(job):
            machine = task[0]
            duration = task[1]
            suffix = '_%i_%i' % (job_id, task_id)
            start_var = model.NewIntVar(0, horizon, 'start' + suffix)
            end_var = model.NewIntVar(0, horizon, 'end' + suffix)
            interval_var = model.NewIntervalVar(start_var, duration, end_var,
                                                'interval' + suffix)
            all_tasks[job_id, task_id] = task_type(start=start_var,
                                                   end=end_var,
                                                   interval=interval_var)
            machine_to_intervals[machine].append(interval_var)

    # Create and add disjunctive constraints.
    for machine in all_machines:
        model.AddNoOverlap(machine_to_intervals[machine])

    # Precedences inside a job.
    for job_id, job in enumerate(jobs_data):
        for task_id in range(len(job) - 1):
            model.Add(all_tasks[job_id, task_id +
                                1].start >= all_tasks[job_id, task_id].end)

    # Makespan objective.
    obj_var = model.NewIntVar(0, horizon, 'makespan')
    model.AddMaxEquality(obj_var, [
        all_tasks[job_id, len(job) - 1].end
        for job_id, job in enumerate(jobs_data)
    ])
    model.Minimize(obj_var)

    # Creates the solver and solve.
    solver = cp_model.CpSolver()
    status = solver.Solve(model)

    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        print('Solution:')
        # Create one list of assigned tasks per machine.
        assigned_jobs = collections.defaultdict(list)
        for job_id, job in enumerate(jobs_data):
            for task_id, task in enumerate(job):
                machine = task[0]
                assigned_jobs[machine].append(
                    assigned_task_type(start=solver.Value(
                        all_tasks[job_id, task_id].start),
                                       job=job_id,
                                       index=task_id,
                                       duration=task[1]))

        # Create per machine output lines.
        output = ''
        for machine in all_machines:
            # Sort by starting time.
            assigned_jobs[machine].sort()
            sol_line_tasks = 'Machine ' + str(machine) + ': '
            sol_line = '           '

            for assigned_task in assigned_jobs[machine]:
                name = 'job_%i_task_%i' % (assigned_task.job,
                                           assigned_task.index)
                # Add spaces to output to align columns.
                sol_line_tasks += '%-15s' % name

                start = assigned_task.start
                duration = assigned_task.duration
                sol_tmp = '[%i,%i]' % (start, start + duration)
                # Add spaces to output to align columns.
                sol_line += '%-15s' % sol_tmp

            sol_line += '\n'
            sol_line_tasks += '\n'
            output += sol_line_tasks
            output += sol_line

        # Finally print the solution found.
        print(f'Optimal Schedule Length: {solver.ObjectiveValue()}')
        print(output)
    else:
        print('No solution found.')

    # Statistics.
    print('\nStatistics')
    print('  - conflicts: %i' % solver.NumConflicts())
    print('  - branches : %i' % solver.NumBranches())
    print('  - wall time: %f s' % solver.WallTime())


# add a time limit to the execution
def main(file_path, timeout_time):
    optima = 0
    task_count = 0
    machine_count = 0
    problem_description = []

    # Read the data from the file, ignore comment lines
    try:
        file = open(file_path, 'r')
    except:
        print(f'Error: Could not open: {file_path}')
        sys.exit()
    
    lines = file.readlines()
    file.close()

    for line in lines:
        values = line.strip().split('\t')
            
        if len(values) == 3:
            task_count = values[0]
            machine_count = values[1]
            optima = values[2]
            continue
            
        pairs = []
        for i in range(0, len(values), 2):
            pair = (int(values[i]), int(values[i+1]))
            pairs.append(pair)
            
        problem_description.append(pairs)
        
    print('-----------------------------------------------------')
    print(f"Theoretical Optimal Schedule Length is {optima}")
    print('-----------------------------------------------------')
    try:
        with multiprocessing.Pool(processes=1) as pool:
            arg1 = problem_description
            result = pool.apply_async(runORTools, args=(arg1,))
            try:
                # Get the result from the function call with the timeout
                result.get(timeout=timeout_time)
                print("runORTools completed successfully")
            except multiprocessing.TimeoutError:
                print("runORTools timed out")
    except Exception as e:
        print("An error occurred:", str(e))
    print('-----------------------------------------------------')


if __name__ == '__main__':
    # Get the absolute path of the current script
    current_folder = dirname(abspath(__file__))
    # Add the current folder to the system path
    sys.path.append(current_folder)

    # default timeout time
    timeout_time = 120

    # argument parsing
    parser = argparse.ArgumentParser(description='''Use Google OR Tools to solve a Job Shop problem,
                                                    only takes standard problem files as input (see JSLib docs)''')

    parser.add_argument('file_path', help='path to the problem file')
    group = parser.add_mutually_exclusive_group()
    group.add_argument("-r", '--rel', help='make the path relative to the "JobShopProblems" folder',
                        action="store_true")
    group.add_argument("-ri", '--rel_inst', help='make the path relative to the "JobShopProblems/Instances" folder',
                        action="store_true")
    parser.add_argument("-t", '--timeout', help=f'set a custom timeout duration in seconds (default: {timeout_time})',
                        metavar="SEC", dest="timeout", type=int, default=timeout_time)

    args = parser.parse_args()

    if args.rel:
        file_path = '../JobShopProblems/' + args.file_path
    elif args.rel_inst:
        file_path = '../JobShopProblems/Instances/' + args.file_path
    else:
        file_path = args.file_path

    main(file_path, args.timeout)
