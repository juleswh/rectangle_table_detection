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

See the `openni_launch` documentation for more info on this part. You also should take a look at the calibration
of your camera.

The program also uses tf to place the camera and the table. So be sure to have at least 2 frames,
including one which indicate the vertical reference of the world (commonly `/world` or `/map`, can
also be a frame fixed to your robot like `/base_link`,...) and one that correspond to the camera
(`<camera_name>_link` if you use openni.launch with `publish_tf` = true (default), otherwise you
need all of the frames `<camera_name>_rgb_optical`, `<camera_name>_depth_optical`, ...)

Now you are ready for launching our table detection.
We recommand to use a launch file, to manage the numerous parameter.
