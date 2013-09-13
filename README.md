Basic Use
=========

Compilation
-----------

(also find this documentation [here](http://julesw.github.io/rectangle_table_detection/rectangle_table_detection/html/basicuse.html))

Be sure to have a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace), and you've sourced the devel/setup.bash of it.

Then clone in the src/ directory the git repository:

    git clone https://github.com/julesw/rectangle_table_detection.git

Then do `catkin_make` in your workspace.


Execution
---------

Then do the following:

    roscore
    roslaunch openni_launch openni.launch

optionnaly, you can add following arguments to `openni.launch`, if you already have a tf publisher that places your camera:

    camera:=<camera_name>
    publish_tf:="false"

See the `openni_launch` documentation for more info on this part. You also should take a look at the calibration
of your camera.
If you never heard about calibration and simply want to see a result right now, do the following in a console:
    rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True
Change 'camera' to your camera name if you do not use the default `openni_launch` name.

The program also uses tf to place the camera and the table. So be sure to have at least 2 frames,
including one which indicate the vertical reference of the world (commonly `/world` or `/map`, can
also be a frame fixed to your robot like `/base_link`,...) and one that correspond to the camera
(`<camera_name>_link` if you use `openni.launch` with `publish_tf` = true (default), otherwise you
need all of the frames `<camera_name>_rgb_optical`, `<camera_name>_depth_optical`, ...)

Now you are ready for launching our table detection.
We recommand to use a launch file, to manage the numerous parameter. You can use the `detection.launch`
launch file as a basis, and modify some parameters.

Results
-------

To visualize the results, in a human friendly interface, use rviz and set it to show the input point cloud,
the tf frames, and the rviz marker (the node does not publish it by default, set `publish_marker_rviz` to 
see it).

You will certainly want to use the output data in another ros node, to do so, use tf and the published topic `table_dimensions`.


Troubleshooting
---------------

Well, if you are here, it's that you're not satisfied with the results.
First of all, check for your parameters to see if they are valid, and if
the node effectively receives an input cloud and the transform tree is valid.
See [the documentation main page](http://julesw.github.io/rectangle_table_detection/html/index.html) for more info on parameters and topics.
