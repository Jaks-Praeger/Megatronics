def generate_commands(path):
    if not path or len(path) < 2:
        return []

    # Initial direction (Assume starting facing North)
    directions = {(0, 1): "N", (1, 0): "E", (0, -1): "S", (-1, 0): "W"}
    current_direction = "W"  

    # Direction transitions (N->E, E->S, S->W, W->N)
    turn_right = {"N": "E", "E": "S", "S": "W", "W": "N"}
    turn_left = {v: k for k, v in turn_right.items()}

    commands = []
    move_count = 0  # Track consecutive moves

    for i in range(1, len(path)):
        x_old, y_old = path[i - 1]
        x_new, y_new = path[i]

        # Calculate movement direction
        dx, dy = x_new - x_old, y_new - y_old

        # Determine the new direction
        new_direction = directions.get((dx, dy))
        if new_direction is None:
            continue  # Ignore invalid movements

        # Adjust direction if needed
        if current_direction != new_direction:
            # If we were moving forward, append the move count before turning
            if move_count > 0:
                commands.append(f"moveForward({move_count})")
                move_count = 0  # Reset move counter

            # Turn until facing the correct direction
            while current_direction != new_direction:
                if turn_right[current_direction] == new_direction:
                    commands.append("turnRight()")
                    current_direction = turn_right[current_direction]
                else:
                    commands.append("turnLeft()")
                    current_direction = turn_left[current_direction]

        # Count consecutive moves
        move_count += 1

    # Append any remaining forward moves
    if move_count > 0:
        commands.append(f"moveForward({move_count})")

    return commands

# Example usage:
path = [(15, 1), (14, 1), (13, 1), (12, 1), (11, 1), (10, 1), (10, 2), (10, 3), (10, 4), (11, 4), (11, 5), (11, 6), (11, 7), (11, 8), (10, 8), (9, 8), (9, 9), (9, 10), (9, 11), (9, 12), (10, 12), (11, 12), (11, 13), (11, 14), (12, 14), (13, 14), (14, 14), (15, 14)]
commands = generate_commands(path)
print(commands)
