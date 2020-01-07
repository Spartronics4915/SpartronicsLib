# SpartronicsLib

This is Spartronics FRC Team 4915's year-independent library. Features include:
 - JNI bindings for the Intel RealSense T265 camera which allow for seamless, plug-and-play V-SLAM on the roboRIO.
 - A pure Java RPLidar A1M8 driver that works on the roboRIO, along with a 2D LIDAR object detection algorithim that works on squares and circles. Also includes a LIDAR target tracker.
 - An A\* path finder that can generate the waypoints for a trajectory that avoids arbitrary axis-aligned obstacles.
 - A trajectory generator and followers fast enough to generate trajectories on the fly (<20 ms generation time), along with a variety of constraints to make trajectories driveable.
 - Motor wrappers to abstract the various FRC-legal motor controllers and to ease unit conversions.
 - Unit conversion, logging and other handy utilities.

## Demo Videos
Click on the below thumbnails to play the videos.

### LIDAR object detection on the 2020 Power Cell gamepieces
<a href="http://www.youtube.com/watch?feature=player_embedded&v=fzdhYzALs-o" target="_blank"><img src="http://img.youtube.com/vi/fzdhYzALs-o/0.jpg" alt="Thumbnail and link for a video demoing detecting power cells with LIDAR" width="800" height="450" border="10" /></a>

### Generating a trajectory on the fly to an object detected with LIDAR
_Odometry is provided by the V-SLAM camera. The robot drive to the center of the ball.__

<a href="http://www.youtube.com/watch?feature=player_embedded&v=4HTlq_ENjXw" target="_blank"><img src="http://img.youtube.com/vi/4HTlq_ENjXw/maxresdefault.jpg" alt="Thumbnail and link for a video demoing the robot driving to a ball detected with LIDAR, using V-SLAM for odometry during trajectory following." width="800" height="450" border="10" /></a>

## Using SpartronicsLib
First, add the JitPack repository to your `build.gradle`:
```groovy
repositories {
    maven { url 'https://jitpack.io' } // Add this line
}
```

Then, add SpartronicsLib as a dependency (note that this will use the latest version of SpartronicsLib on the master branch; to use something different you should change the `master-SNAPSHOT` bit):
```groovy
dependencies {
    implementation "com.github.Spartronics4915:SpartronicsLib:master-SNAPSHOT"
}
```

## Contributing to SpartronicsLib
When adding features to SpartronicsLib, it is reccomended that you include a unit test. The unit testing is very easy, and in many cases it means that you don't need to make another project or have a robot to test your feature.

If you *do* want to make a change to SpartronicsLib while it's included in an external project, then you should use a local copy of SpartronicsLib instead of the Jitpack one when building your external project. This is easy as passing `--include-build ../path/to/SpartronicsLib` to the Gradle wrapper when building or testing your external project.
