Create a `data` directory inside `/home/username/.ros/`. Copy the files included in the zip folder and paste them in
the `data` directory. Those files contain the information extracted during training and are mandatory for the detection
procedure.

It is worth mentioning that the technical specifications of the spring used for the training can be found in the [link](http://cz.rs-online.com/web/p/tlacne-pruziny/0751607/?searchTerm=751-607&relevancy-
data=636F3D3126696E3D4931384E525353746F636B4E756D6265724D504E266C753D656E266D6D3D6D617463
68616C6C26706D3D5E283F69292852537C5253207C52532D293F5C647B337D285C73293F5B5C732D2F255C2E
2C5D285C73293F5C647B332C347D2426706F3D313426736E3D592673743D52535F53544F434B5F4E554D4245
522677633D4E4F4E45267573743D3735312D363037267374613D3037353136303726)

Build the package using `catkin_make`.

Type `rosrun spring_detector test_detector` in the command window. It is the node which advertises the service for
spring detection and pose estimation.

In another terminal type `rosservice call /detect_spring`. It returns a ros custom message containing the convex hull
and the pose of each spring detected in the test image. Each convex hall is represented by a vector of x,y coordinates
(point.msg) of the image. The convex hull and the variables theta,phi(rotation of the spring over the Z and Y axis
respectively) form the hull.msg and they describe the position and the pose of a spring. A vector of hull.msg messages
(hullArray.msg) containing the convex halls and rotations of all springs detected in the image, is the final output of
the service.

The test image is read by a ros topic created for test purposes and it can be easily modified according to how the
image will be provided.

Running the code will produce also an image in the `/tmp` directory.

Pose estimation
The initial position of the object is shown below, where the origin of the object is set to the center of its mass.
The variables phi and theta denote the rotation of the spring over the Y and Z axis respectively, following the
sequence of XZY intrinsic rotations. Since the spring is symmetric with respect to X axis, no rotation is needed.

![alt text](https://gitlab.ciirc.cvut.cz/mitre/radioroso_certh/blob/5864d863e586478f0b2e9885c8d1e90a0e2e96b0/blender.png "reference")


