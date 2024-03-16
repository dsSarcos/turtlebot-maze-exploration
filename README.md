Maze Simulation

Authors
Diego Santamaria-sarcos
Marius Baden

Program Structure
Task Manager
    The task manager is a simple node that progresses until the end of its execution. While doing so, it calls for two services, 'exploration/explore_maze' and 'maze_manager/solve_maze' in order.
Exploration Manager

Maze Manager
    The maze manager, upon receiving a call from the task manager, begins by requesting an image of the map from the hector_compressed_map_transport node. This image is then converted into an OpenCV material, which is then processed to locate the two exits for the map. The exit furthest from the robot's initial position is selected and sent to the motion controller for execution.
    The processing pipeline structure from image to solution is as follows:
    map_to_image -> image_to_material -> blurring -> thresholding -> contouring -> hulling
    The CV material is blurred so that the thresholding only detects the borders of the map. This thresholded image is then processed through OpenCV's findCountours(). These contours are then used to construct hulls for the image.
    These hulls are then processed further to extract the two largest (that aren't the border); These two hulls are always triangles, so the larger diagonal is found and its enpoints are compared by distance to the initial position. The furthest point is transformed from our this image frame to the map frame and converted to a goal pose, which is sent to the motion controller.
Motion Controller
    This node handles two tasks. The first is publishing the robot's estimated pose, since gmapping doesn't do that automatically. The second is acting as an interface for us to easily interact with the move_base actions. To handle this, we implemented two services: the nonblocking one is used by the exploration manager, and the blocking one is used by the maze manager.

Usage
    To run our code, you can use our launchfile:

    $roslaunch maze_simulation_sarcos_baden maze_exploration_and_exit.launch map_number:={1 | 2}

    NOTE the map number is necessary in order to launch gazebo

Contribution
    Our tasks were relatively cleanly divided for this project. Furthermore, we worked on this project side-by-side (with some online calls) till the end with a few minor exceptions like minor bug-fixing and code cleanup on our own. 
    In our final implementation the following tasks were completed primarily by:
    Diego
        maze_manager
    Marius
        exploration_manager
    Both
        task_manager
        launchfile
        motion_controller
        README
        demo video
Sources
    Converting between ROS images and OpenCV images (Python) http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython 
    Convex Hull using OpenCV in Python and C++ https://learnopencv.com/convex-hull-using-opencv-in-python-and-c/ 
    hector_compressed_map_transport http://wiki.ros.org/hector_compressed_map_transport 
    How to get robot's pose estimation data from the SLAM algorithm? https://stackoverflow.com/questions/69389307/how-to-get-robots-pose-estimation-data-from-the-slam-algorithm