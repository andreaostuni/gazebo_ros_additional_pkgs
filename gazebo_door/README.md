# Gazebo Classic corridor + two rooms + automatic door

This example contains:

- a **Gazebo Classic world** with:
  - one corridor
  - two rooms
  - one hinged door between the corridor and the right room
- a **ModelPlugin** that opens the door when a named entity gets close

## Files

- `worlds/corridor_two_rooms_door.world`
- `plugins/proximity_door_plugin.cpp`
- `CMakeLists.txt`

## Build

```bash
cd /path/to/gazebo_door_example
mkdir -p build && cd build
cmake ..
make -j
```

## Run

From the project root:

```bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$(pwd)/build
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$(pwd)
gazebo worlds/corridor_two_rooms_door.world
```

## How it works

The world contains a model named `robot` that starts in the corridor.
The door plugin monitors the distance between:

- the target model: `robot`
- the doorway trigger point: `(8.0, 0.0)`

When the target gets within `trigger_distance`, the hinge joint `door_hinge`
is driven toward `open_angle`.

## Moving the trigger entity manually

Inside Gazebo Classic, you can use the translate tool and drag the `robot`
model close to the door. The door should open automatically.

## Notes

- The door is a real articulated body with **collision**, so when it opens the passage is actually cleared.
- The plugin uses `world->ModelByName(target_model)` so it watches models, not individual links.
- You can adapt it to a ROS 2 topic / service based door later if needed.
