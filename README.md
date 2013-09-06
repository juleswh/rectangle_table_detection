Basic Use
=========

Compilation
-----------

Be sure to have a catkin workspace (http://wiki.ros.org/catkin/Tutorials/create_a_workspace), and you've sourced the devel/setup.bash of it.

Then clone in the src/ directory the git repository:

    git clone https://github.com/julesw/rectangle_table_detection.git

Then   `catkin_make` in your workspace.


Execution
---------

Then do the following:

    roscore
    roslaunch openni_launch openni.launch

optionnaly, you can add following arguments to openni.launch, if you already have a tf publisher that places your camera:

    camera:=<camera_name>
    publish_tf:="false"

See the `openni_launch` documentation for more info.

