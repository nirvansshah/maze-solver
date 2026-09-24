# Maze Solver

# Arduino Nano R4 Maze Robot

Autonomous maze-solving robot built with an **Arduino Nano R4**, **TB6612FNG motor driver**, wheel encoders, and five **VL53L0X ToF distance sensors**.

The robot uses a **left-hand maze-solving strategy** and encoder-based movement for consistent forward and 90° turns.

## Hardware

* Arduino Nano R4
* TB6612FNG motor driver
* 2 × DC geared motors with encoders
* 5 × VL53L0X distance sensors
* Robot chassis and battery

## Features

* Left-hand maze solving
* 5-direction distance sensing
* Encoder-based cell movement
* Encoder-based 90° turns
* Automatic VL53L0X address assignment
* Sensor filtering and basic error handling
* Adjustable motor speed, turn speed and thresholds

## Sensor Mapping

```text
        FRONT
          |
     L45  |  R45
       \  |  /
        LEFT RIGHT
```

The code maps the five sensors as:

```text
FRONT <- Sensor 2
LEFT  <- Sensor 4
RIGHT <- Sensor 0
L45   <- Sensor 3
R45   <- Sensor 1
```

## Maze Logic

The robot follows:

```text
LEFT → FORWARD → RIGHT → U-TURN
```

Movement is controlled using wheel encoder feedback rather than fixed delays wherever possible.

## Tuning

The main values to tune for a different robot are near the top of the sketch:

```cpp
FWD_PWM
TURN_PWM
TICKS_PER_CELL
TICKS_TURN_90
TH_LEFT_NEAR
TH_RIGHT_NEAR
TH_FRONT_BLOCK
```

These values depend on the motors, wheel size, sensor mounting and maze layout.

## Project Status

Built and tuned as a practical maze-solving robot. Hardware-specific tuning may be required when using the code on a different robot.

## License

MIT License
