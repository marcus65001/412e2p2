# CMPUT 412 Exercise 2 - Part 2
This repository contains all the code for Exercise 2 Part 2.

## Usage
1. Clone the repository.
2. Build `dts devel build -H $BOT`.
3. Run `led_emitter_node` form `dt-core`: 

```
dts duckiebot demo --demo_name led_emitter_node --duckiebot_name $BOT --package_name led_emitter --image duckietown/dt-core:daffy-arm64v8
```

4. Run `dts devel run -H $BOT`.

## Nodes
* led_node: Provides custom service `set_led`, changes LED pattern.
* odometry_node: Subscribes to the `velocity` topic from `kinematics_node`, records the intergrated `x, y, theta` info for both robot and world frame.
* task_p2_node: Executes the tasks specified in Part 2.


## License
[Duckietown](https://www.duckietown.org/about/sw-license)
