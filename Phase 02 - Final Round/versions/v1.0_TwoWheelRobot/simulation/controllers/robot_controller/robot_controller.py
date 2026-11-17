"""
PeraBots 2026 - Robot Controller
Nine Seals of Pera - Final Round Challenge

This controller provides basic autonomous navigation capabilities:
- Line following using ground sensors
- Obstacle avoidance using distance sensors
- Maze exploration with wall following
- Color tile detection

Teams should modify and extend this controller for their specific strategy.
"""

from controller import Robot, DistanceSensor, Motor, Camera
import math

# ============================================================
# CONSTANTS
# ============================================================
TIME_STEP = 16
MAX_SPEED = 6.28  # rad/s for e-puck motors

# Sensor thresholds
OBSTACLE_THRESHOLD = 80.0   # Distance sensor threshold for obstacles
LINE_THRESHOLD = 500        # Ground sensor threshold for black line

# Navigation states
STATE_START = 0
STATE_OBSTACLE_NAV = 1
STATE_MAZE_ENTRY = 2
STATE_MAZE_EXPLORE = 3
STATE_MAZE_EXIT = 4
STATE_BOARD_APPROACH = 5
STATE_BOARD_SOLVE = 6
STATE_FINISH = 7

class PeraBotsController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Initialize motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        # Initialize proximity sensors (e-puck has 8 IR sensors: ps0-ps7)
        self.prox_sensors = []
        for i in range(8):
            sensor_name = 'ps' + str(i)
            sensor = self.robot.getDevice(sensor_name)
            sensor.enable(self.timestep)
            self.prox_sensors.append(sensor)

        # Initialize ground sensors (for line following)
        self.ground_sensors = []
        for i in range(3):
            sensor_name = 'gs' + str(i)
            try:
                sensor = self.robot.getDevice(sensor_name)
                if sensor:
                    sensor.enable(self.timestep)
                    self.ground_sensors.append(sensor)
            except Exception:
                pass

        # Initialize camera (for tile color detection)
        try:
            self.camera = self.robot.getDevice('camera')
            if self.camera:
                self.camera.enable(self.timestep)
        except Exception:
            self.camera = None

        # State variables
        self.state = STATE_START
        self.blue_tile_count = 0
        self.green_tile_count = 0
        self.right_turns = 0
        self.left_turns = 0
        self.step_count = 0

        print("[PeraBots] Robot initialized")
        print("[PeraBots] Starting quest: The Nine Seals of Pera")

    def set_speed(self, left, right):
        """Set motor speeds (clamped to MAX_SPEED)."""
        left = max(-MAX_SPEED, min(MAX_SPEED, left))
        right = max(-MAX_SPEED, min(MAX_SPEED, right))
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)

    def get_prox_values(self):
        """Read all proximity sensor values."""
        return [s.getValue() for s in self.prox_sensors]

    def get_ground_values(self):
        """Read all ground sensor values."""
        if self.ground_sensors:
            return [s.getValue() for s in self.ground_sensors]
        return [1000, 1000, 1000]  # Default: no line detected

    def detect_obstacle(self, prox_values):
        """Check if an obstacle is detected in front."""
        # ps0 = front-right, ps7 = front-left
        front_left = prox_values[7]
        front_right = prox_values[0]
        return front_left > OBSTACLE_THRESHOLD or front_right > OBSTACLE_THRESHOLD

    def wall_follow_left(self, prox_values):
        """Left wall following for maze exploration."""
        # e-puck sensor layout:
        #   ps7 ps0  (front)
        #  ps6    ps1
        # ps5      ps2
        #  ps4 ps3  (back)

        left_wall = prox_values[5]     # left side sensor
        left_front = prox_values[6]    # left-front sensor
        front_left = prox_values[7]    # front-left sensor
        front_right = prox_values[0]   # front-right sensor

        base_speed = MAX_SPEED * 0.5

        # If no left wall, turn left
        if left_wall < OBSTACLE_THRESHOLD and left_front < OBSTACLE_THRESHOLD:
            self.set_speed(base_speed * 0.3, base_speed)
            return 'left_turn'

        # If front wall detected, turn right
        if front_left > OBSTACLE_THRESHOLD or front_right > OBSTACLE_THRESHOLD:
            self.set_speed(base_speed, -base_speed * 0.5)
            return 'right_turn'

        # Follow left wall - maintain distance
        if left_wall > OBSTACLE_THRESHOLD * 1.5:
            # Too close to left wall - veer right
            self.set_speed(base_speed, base_speed * 0.7)
        elif left_wall < OBSTACLE_THRESHOLD * 0.5:
            # Too far from left wall - veer left
            self.set_speed(base_speed * 0.7, base_speed)
        else:
            # Good distance - go straight
            self.set_speed(base_speed, base_speed)

        return 'straight'

    def obstacle_avoidance(self, prox_values):
        """Simple obstacle avoidance using Braitenberg-style logic."""
        # Braitenberg weights for obstacle avoidance
        left_weight = [150, 100, 80, -10, -20, -80, -100, -150]
        right_weight = [-150, -100, -80, -10, -20, 80, 100, 150]

        left_speed = 0
        right_speed = 0

        for i in range(8):
            left_speed += prox_values[i] * left_weight[i] / 2000.0
            right_speed += prox_values[i] * right_weight[i] / 2000.0

        left_speed = MAX_SPEED * 0.4 + left_speed
        right_speed = MAX_SPEED * 0.4 + right_speed

        self.set_speed(left_speed, right_speed)

    def line_follow(self, ground_values):
        """Follow a black line using ground sensors."""
        if len(ground_values) < 3:
            self.set_speed(MAX_SPEED * 0.3, MAX_SPEED * 0.3)
            return

        left_g = ground_values[0]
        center_g = ground_values[1]
        right_g = ground_values[2]

        base_speed = MAX_SPEED * 0.4

        if center_g < LINE_THRESHOLD:
            # On the line
            self.set_speed(base_speed, base_speed)
        elif left_g < LINE_THRESHOLD:
            # Line is to the left
            self.set_speed(base_speed * 0.2, base_speed)
        elif right_g < LINE_THRESHOLD:
            # Line is to the right
            self.set_speed(base_speed, base_speed * 0.2)
        else:
            # Lost the line - go forward slowly
            self.set_speed(base_speed * 0.5, base_speed * 0.5)

    def run(self):
        """Main control loop."""
        print("[PeraBots] Quest begins!")
        print("[PeraBots] State: Navigating obstacle field")

        while self.robot.step(self.timestep) != -1:
            prox_values = self.get_prox_values()
            ground_values = self.get_ground_values()
            self.step_count += 1

            if self.state == STATE_START:
                # Initial movement - go forward
                self.set_speed(MAX_SPEED * 0.4, MAX_SPEED * 0.4)
                if self.step_count > 50:
                    self.state = STATE_OBSTACLE_NAV
                    print("[PeraBots] Entering obstacle navigation")

            elif self.state == STATE_OBSTACLE_NAV:
                # Navigate through obstacle field using obstacle avoidance
                if self.detect_obstacle(prox_values):
                    self.obstacle_avoidance(prox_values)
                else:
                    # Try to move forward generally
                    self.set_speed(MAX_SPEED * 0.4, MAX_SPEED * 0.4)

            elif self.state == STATE_MAZE_EXPLORE:
                # Maze exploration using left wall following
                action = self.wall_follow_left(prox_values)

                # Count turns for clue gathering
                if action == 'left_turn':
                    self.left_turns += 1
                elif action == 'right_turn':
                    self.right_turns += 1

            elif self.state == STATE_BOARD_APPROACH:
                # Navigate to the game board
                self.set_speed(MAX_SPEED * 0.3, MAX_SPEED * 0.3)

            elif self.state == STATE_FINISH:
                # Stop the robot
                self.set_speed(0, 0)
                print("[PeraBots] Quest complete!")
                print(f"[PeraBots] Blue tiles: {self.blue_tile_count}")
                print(f"[PeraBots] Green tiles: {self.green_tile_count}")
                print(f"[PeraBots] Right turns: {self.right_turns}")
                print(f"[PeraBots] Left turns: {self.left_turns}")
                break

            # Periodic status print
            if self.step_count % 500 == 0:
                print(f"[PeraBots] Step {self.step_count}, State: {self.state}")

# ============================================================
# MAIN
# ============================================================
if __name__ == '__main__':
    controller = PeraBotsController()
    controller.run()
