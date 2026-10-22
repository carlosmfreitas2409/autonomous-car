"""Which stage of the competition the car is driving.

    Delivery - the drawn pickup and delivery letters: the mission picks the
               detours and stops at the letters, stop signs and traffic
               lights are obeyed, the sonar guard slows and stops the car.
    Speed    - one lap for time. Only the line: the path stays on Straight
               (it passes both forks), no mission, no signs or lights, no
               sonar. The sign camera is paused so the lane loop has the
               machine to itself, and the strip runs the police effect.

Each stage keeps its own PID and speed sliders. The stage is only changed
while e-stopped; the loop owns it and the GUI shows it.
"""

from enum import Enum


class Challenge(Enum):
    DELIVERY = "delivery"
    SPEED = "speed"
