# Grasp Synergy ROS package

[![Build Status](https://travis-ci.org/felixduvallet/grasp-synergy.svg?branch=master)](https://travis-ci.org/felixduvallet/grasp-synergy)

This package implements grasp synergies (aka eigengrasps): low-dimensional
representation of hand configurations.

Very basically, grasp synergies are computed by doing PCA on a bunch of hand
configurations (joint angles) to infer a low-dimensional representation.

Points in this low-dimensional space can then be reprojected to the full
configuration space to control the hand.

This ROS package is independent of any hand configuration.
It consists of two components:
 - the GraspSynergy class: basically a wrapper around PCA.
 - the synergy_node: ROS magic to provide nice interfaces for controlling hands.

## grasp_synergy

Grasp Synergies (aka eigengrasps) are just low-dimensional representations of
the grasp learned using PCA.

The GraspSynergy class is just a convenience class for computing, storing, and
commanding grasp synergies. It is hand-agnostic.

Grasping data can be of any form:
 - A numpy matrix of joint values (N by D)
 - A list of sensor_msgs/JointState messages
 - A filepath to a rosbag file.

Here N are the number of collected grasps and D is the full dimensionality of
the hand. We extract all D principal components (synergies), but in practice we
will only use a smaller number.

To compute a grasp (in the original space), simply pass in a B-dimensional
vector of coefficients to `compute_grasp`. (Nominally, B < D.) The method will
automatically figure out how many components to use, and return the
corresponding grasp configuration in the original D-dimensional space.

## synergy_node

The synergy node enables you to load a grasp synergy space, create subscribers
for the synergy space, and publish desired joint state messages to control the
hand.

Each time the node receives a new low-dimensional (synergy-space) point, it
computes the hand configuration and publishes a message with the desired joint
states.

**Subscribers:**
The node can create a variable number of subscribers: one for the
fully-specified coefficient vector, and one per component (with singleton
values).

So for example, you might have the following subscribers:
```
/grasp_synergy
/grasp_synergy/syn_0
/grasp_synergy/syn_1
/grasp_synergy/syn_2
/grasp_synergy/syn_3
/grasp_synergy/syn_4
```

This enables you to use tools like `rqt_ez_publisher` to provide a nice slider
GUI interface to the synergy space:

![rqt_grasp](https://cloud.githubusercontent.com/assets/6153835/14283454/c8cb284a-fb43-11e5-995e-452cfa981145.png)

**Publisher:**
The desired joint state topic must be given.
