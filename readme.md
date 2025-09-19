

````markdown
# ROS2 Turtlesim Quick Start

## Open VSCode in WSL
```bash
wsl
code .
````

## Run Turtlesim

In **Terminal 1**:

```bash
ros2 run turtlesim turtlesim_node
```

In **Terminal 2** (replace `file_name.py` with your script):

```bash
python3 file_name.py
```

## View ROS2 Graph

```bash
ros2 run rqt_graph rqt_graph
```

```
