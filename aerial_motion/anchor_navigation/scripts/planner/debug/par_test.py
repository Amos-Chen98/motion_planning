import multiprocessing
import time


class BSplinePlanner:
    def __init__(self, a, b):
        self.a = a
        self.b = b

    def plan(self, c, d):
        # Dummy planning logic
        time.sleep(1)
        return c + d


def worker(a, b):
    """Worker function that creates a planner and calls the plan method"""
    local_planner = BSplinePlanner(a, b)
    result = local_planner.plan(a, b)
    # Assume the result of plan is a list (e.g. trajectory points)
    # Create a sample list as the result
    trajectory = [result * i for i in range(3)]  # Generate three trajectory points as an example
    return a, b, trajectory


if __name__ == '__main__':  # This guard is required for multiprocessing
    num_list = [1, 2, 3, 4, 5, 6]
    
    # Prepare parallel planning tasks
    planning_tasks = []
    for i in range(len(num_list) - 1):
        a = num_list[i]
        b = num_list[i + 1]
        planning_tasks.append((a, b))
    
    # Start timing
    time_start = time.time()
    
    # Execute in parallel using a process pool
    with multiprocessing.Pool(processes=1) as pool:
        results = pool.starmap(worker, planning_tasks)
    
    # End timing
    time_end = time.time()
    
    # Concatenate results in order into a single list
    combined_trajectory = []
    for i, (a, b, trajectory) in enumerate(results):
        print(f"Planning from {a} to {b}: {trajectory}")
        
        if i == 0:
            # Add all points from the first segment
            combined_trajectory.extend(trajectory)
        else:
            # Skip the first point of subsequent segments to avoid duplicates
            combined_trajectory.extend(trajectory[1:])
    
    print(f"\nCombined trajectory: {combined_trajectory}")
    print(f"Total time: {time_end - time_start:.4f} seconds")
    print(f"Tasks: {len(planning_tasks)}, using 5 processes")
