from machine import Pin
from ir_api import IR
import time


def moveForward(steps):
    print(f"Moving forward {steps} steps")
    # Add actual movement code here


def turnLeft():
    print("Turning left")
    # Add actual movement code here


def turnRight():
    print("Turning right")
    # Add actual movement code here


def generate_commands(path):
    if not path or len(path) < 2:
        return []

    directions = {(0, 1): "N", (1, 0): "E", (0, -1): "S", (-1, 0): "W"}
    current_direction = "W"
    turn_right = {"N": "E", "E": "S", "S": "W", "W": "N"}
    turn_left = {v: k for k, v in turn_right.items()}

    commands = []
    move_count = 0

    for i in range(1, len(path)):
        x_old, y_old = path[i - 1]
        x_new, y_new = path[i]
        dx, dy = x_new - x_old, y_new - y_old
        new_direction = directions.get((dx, dy))
        if new_direction is None:
            continue

        if current_direction != new_direction:
            if move_count > 0:
                commands.append(("moveForward", move_count))
                move_count = 0
            while current_direction != new_direction:
                if turn_right[current_direction] == new_direction:
                    commands.append(("turnRight",))
                    current_direction = turn_right[current_direction]
                else:
                    commands.append(("turnLeft",))
                    current_direction = turn_left[current_direction]
        move_count += 1

    if move_count > 0:
        commands.append(("moveForward", move_count))
    return commands


def execute_commands(commands):
    for command in commands:
        if command[0] == "moveForward":
            moveForward(command[1])
        elif command[0] == "turnLeft":
            turnLeft()
        elif command[0] == "turnRight":
            turnRight()
        time.sleep(0.5)  # Small delay for execution


PIN = 22
irm = IR(PIN)
print("IR Test Start...")
path = [
    (15, 1),
    (14, 1),
    (13, 1),
    (12, 1),
    (11, 1),
    (10, 1),
    (10, 2),
    (10, 3),
    (10, 4),
    (11, 4),
    (11, 5),
    (11, 6),
    (11, 7),
    (11, 8),
    (10, 8),
    (9, 8),
    (9, 9),
    (9, 10),
    (9, 11),
    (9, 12),
    (10, 12),
    (11, 12),
    (11, 13),
    (11, 14),
    (12, 14),
    (13, 14),
    (14, 14),
    (15, 14),
]
commands = generate_commands(path)

while True:
    IR_re = irm.scan()
    if IR_re[0] == True and IR_re[1] != None:
        print("command: %s " % IR_re[1])
        if IR_re[1] == "1":  # Pressing "1" on the IR remote starts movement
            print("Executing path...")
            execute_commands(commands)
        time.sleep(0.02)
    time.sleep(0.1)
