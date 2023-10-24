# ompl-ros-connector
This repository contains a move_base plugin for integrating OMPL (Open Motion Planning Library) planners with ROS (Robot Operating System). Through this integration, users can leverage the sophisticated motion planning algorithms provided by OMPL directly within the ROS ecosystem.

## Features
- **OMPL Integration**: Enables the use of various planners from the OMPL library directly within ROS / move_base.
- **Extensibility**: Provides an easy pathway to integrate new OMPL planners.

## Dependencies
- [OMPL](https://github.com/ompl/ompl) (Tested with version 1.6.0)
- [ROS](https://wiki.ros.org/noetic) (Tested with Noetic)
    - dynamic_reconfigure
    - costmap_2d
    - move_base

## Installation
1. Please first install OMPL by yourself. You can check the [wiki](https://ompl.kavrakilab.org/download.html) for installation instructions.
2. Install ros package dependencies
    ```bash
    sudo apt install ros-noetic-dynamic-reconfigure ros-noetic-costmap-2d ros-noetic-move-base
    ```
3. Clone the repository
    ```bash
    git clone https://github.com/esatgundogdu/ompl-ros-connector.git
    ``` 
4. Build package
    ```bash
    cd ~/catkin_ws/
    catkin_make
    ```

## Adding More Planners
**TODO**

## Known Issues / TODO

## Contributing
Contributions are what make the open-source community such an amazing place to learn, inspire, and create. Any contributions you make are greatly appreciated.

- Fork the Project
- Create your Feature Branch (**git checkout -b feature/AmazingFeature**)
- Commit your Changes (**git commit -m 'Add some AmazingFeature'**)
- Push to the Branch (**git push origin feature/AmazingFeature**)
- Open a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.