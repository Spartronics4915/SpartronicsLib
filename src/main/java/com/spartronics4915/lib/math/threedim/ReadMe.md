# Intro

This is a collection of season-agnostic math utilities which may be useful
in Vision solutions.  We recommend that you start exploring from the top-down.
Comments and doctests "say it all"!

* `CamToField2020` is an example transformation chain representing the coordinate system
 transformations for a camera attached to a rotating turret attached to the robot.
 It is located in the frc2020 directory.
* `CamToField` is the base class for any robot that receives periodic updates of the robot's pose.
* `Affine3` is our core representation of geometric transformations and coordinate systems.
* `Quaternion` is a compact/robust representation for rotations
* `Vec3` is a simple cover for 3d points and vectors.

# See Also

* There are lots of comments around the derivation of the current pipeline
  in the test subdir for threedim.

## For a deeper understanding of 4x4 matrices
* http://graphics.cs.cmu.edu/nsp/course/15-462/Spring04/slides/04-transform.pdf
*  Matrices and transformations. Ronald Goldman.
  In "Graphics Gems I", pp 472-475. Morgan Kaufmann, 1990.
*  More matrices and transformations: shear and pseudo-perspective. Ronald Goldman.
  In "Graphics Gems II", pp 320-323. Morgan Kaufmann, 1991.
* Decomposing a matrix into simple transformations. Spencer Thomas.
   In "Graphics Gems II", pp 320-323. Morgan Kaufmann, 1991.
