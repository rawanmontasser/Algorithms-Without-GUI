
import heapq
import time
from collections import deque
from colorama import Fore, Style, init
import random

init(autoreset=True)

GOAL_STATE = [1, 2, 3, 4, 5, 6, 7, 8, 0]
GOAL_TUPLE = tuple(GOAL_STATE)

MOVES = [(-1, 0), (1, 0), (0, -1), (0, 1)]
MOVE_NAMES = ['UP', 'DOWN', 'LEFT', 'RIGHT']

def index_to_position(index):
    return (index // 3, index % 3)

def is_goal(state):
    return tuple(state) == GOAL_TUPLE

def get_neighbors(state):
    neighbors = []
    zero_index = state.index(0)
    zero_row, zero_col = index_to_position(zero_index)

    for dr, dc in MOVES:
        new_row, new_col = zero_row + dr, zero_col + dc
        if 0 <= new_row < 3 and 0 <= new_col < 3:
            new_index = new_row * 3 + new_col
            new_state = state[:]
            new_state[zero_index], new_state[new_index] = new_state[new_index], new_state[zero_index]
            neighbors.append((new_state, MOVE_NAMES[MOVES.index((dr, dc))]))

    return neighbors

def bfs(initial_state):
    visited = set()
    queue = deque([(initial_state, [], None)])

    while queue:
        current_state, path, last_move = queue.popleft()
        if is_goal(current_state):
            return path
        
        visited.add(tuple(current_state))
        for neighbor, move in get_neighbors(current_state):
            if tuple(neighbor) not in visited:
                queue.append((neighbor, path + [(neighbor, move)], move))
    
    return None

def dfs(initial_state, max_depth=50):
    stack = [(initial_state, [], None, 0)]  
    visited = set()
    visited_count = 0  

    start_time = time.time()

    while stack:
        current_state, path, last_move, depth = stack.pop()

        if is_goal(current_state):
            algorithm_execution_time = time.time() - start_time
            if algorithm_execution_time < 1:
                artificial_delay(1 - algorithm_execution_time)
            return path

        state_tuple = tuple(current_state)
        if state_tuple not in visited:
            visited.add(state_tuple)
            visited_count += 1  

            if depth < max_depth:  
                neighbors = get_neighbors(current_state)

                for neighbor, move in reversed(neighbors):
                    if tuple(neighbor) not in visited:
                        stack.append((neighbor, path + [(neighbor, move)], move, depth + 1))

    algorithm_execution_time = time.time() - start_time
    if algorithm_execution_time < 1:
        artificial_delay(1 - algorithm_execution_time)

    return None


def ucs(initial_state):
    visited = set()
    queue = []
    heapq.heappush(queue, (0, initial_state, [], None))

    while queue:
        cost, current_state, path, last_move = heapq.heappop(queue)
        if is_goal(current_state):
            return path
        
        visited.add(tuple(current_state))
        for neighbor, move in get_neighbors(current_state):
            if tuple(neighbor) not in visited:
                new_cost = cost + 1
                heapq.heappush(queue, (new_cost, neighbor, path + [(neighbor, move)], move))

    return None

def print_matrix(state):
    print("╔═══╦═══╦═══╗")
    for i in range(0, 9, 3):
        print(f"║ {state[i]} ║ {state[i+1]} ║ {state[i+2]} ║")
        if i < 6:
            print("╠═══╬═══╬═══╣")
    print("╚═══╩═══╩═══╝")

def artificial_delay(seconds):
    time.sleep(seconds)

def solve_puzzle():
    start_time = time.time()

    print(Fore.BLUE + "\nWelcome to the 8 puzzle solver!")
    initial_state = []
    for i in range(1, 4):
        row = list(map(int, input(f"Enter row {i} (3 integers separated by space): ").split()))
        if len(row) != 3:
            print(Fore.RED + "Error: Each row must have exactly 3 integers.")
            return
        initial_state.extend(row)

    print(Fore.CYAN + "\nInitial puzzle state:")
    print_matrix(initial_state)

    if len(initial_state) != 9 or sorted(initial_state) != list(range(9)):
        print(Fore.RED + "Invalid initial state. The state must contain numbers 0-8.")
        return

    print(Fore.GREEN + "\nSelect an algorithm:")
    print("1. BFS (Breadth-First Search)")
    print("2. DFS (Depth-First Search)")
    print("3. UCS (Uniform Cost Search)")

    choice = int(input("Enter your choice (1/2/3): "))

    solution_path = None
    algorithm_start_time = time.time()

    if choice == 1:
        print(Fore.YELLOW + "\nSolving using BFS...")
        solution_path = bfs(initial_state)

    elif choice == 2:
        print(Fore.YELLOW + "\nSolving using DFS...")
        solution_path = dfs(initial_state, max_depth=50)  

    elif choice == 3:
        print(Fore.YELLOW + "\nSolving using UCS...")
        solution_path = ucs(initial_state)

    else:
        print(Fore.RED + "Invalid choice.")
        return

    algorithm_end_time = time.time()
    algorithm_execution_time = algorithm_end_time - algorithm_start_time

    if solution_path is None:
        print(Fore.RED + "\nNo solution found.")
    else:
        print(Fore.GREEN + "\nSolution found!")
        for i, (state, move) in enumerate(solution_path):
            print(Fore.MAGENTA + f"\nStep {i + 1}: Move {move}")
            print_matrix(state)

    total_time = time.time() - start_time
    total_steps = len(solution_path) if solution_path else 0
    total_cost = total_steps + 1
    print(Fore.CYAN + f"\nTime Taken: {algorithm_execution_time:.4f} seconds")
    print(Fore.CYAN + f"Total Steps Taken: {total_steps}")
    print(Fore.CYAN + f"Total Cost: {total_cost}")

def continue_prompt():
    while True:
        user_input = input(Fore.GREEN + "\nDo you want to solve another puzzle? (y/n): ").lower()
        if user_input == 'y':
            solve_puzzle()
        elif user_input == 'n':
            print(Fore.RED + "Goodbye!")
            break
        else:
            print(Fore.RED + "Invalid input. Please enter 'y' or 'n'.")

if __name__ == "__main__":
    solve_puzzle()
    continue_prompt()

