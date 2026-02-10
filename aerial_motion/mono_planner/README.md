# mono_planner
Motion planner for monolithic (rigid-body) robots. Plans from point cloud input.

## Dependencies

```
sudo apt install ros-one-octomap* # change the ROS version according to your system
pip install octomap-python
```

If you encounter an error while installing octomap-python, run the following command instead:

```
CXXFLAGS="-std=c++11" pip install octomap-python
```

If problem still exists, refer to https://github.com/wkentaro/octomap-python for more solutions.

If it reports missing `libdynamicedt3d.so.1.8` while running, add the following line to `.bashrc`:

```
export LD_LIBRARY_PATH=~/.local/lib:$LD_LIBRARY_PATH
```