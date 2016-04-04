import rospy

import numpy as np
from sklearn.decomposition import PCA


class GraspSynergy(object):
    """
    A grasp synergy (aka eigengrasp) is a lower-dimensional representation of
    the hand configuration useful to simplify all manners of grasping things.

    Essentially this is just a utilitarian wrapper around PCA.
    """

    def __init__(self):
        # Fit *all* components. Select lower-dimensional subspace later.
        self._pca = PCA(n_components=None)
        self._N = 0
        self._D = 0

        # Store the transformed joint angles for all principal components. This
        # is used to compute the min/max values of the transformed
        # representation.
        self._transformed_joint_angles = None

    def fit_joint_values(self, joint_values):
        """
        Fit the principal components of the given joint values to compute the
        grasp synergies (aka eigengrasps).

        We compute and store *all* D principal components here. The
        dimensionality of the lower-dimensional subspace is determined at grasp
        computation time.

        :param joint_values A numpy array (N by D) with N datapoints of D
        dimensions.

        """
        self._N = joint_values.shape[0]
        self._D = joint_values.shape[1]
        self._transformed_joint_angles = self._pca.fit_transform(joint_values)

    def fit_joint_state_messages(self, joint_states_msgs):
        """
        Extract joint states from a list of ROS messages, then compute the grasp
        synergies.

        :param joint_states_msgs A list of ROS sensor_msgs/JointState.
        """

        joint_values = []
        for (idx, msg) in enumerate(joint_states_msgs):
            try:  # Catch any exceptions due to a wrong message type.
                joint_values.append(msg.position)
            except AttributeError:
                rospy.logwarn('Message is not a JointState, skipping.')
                pass
        # Create a numpy array of size NxD.
        joint_values = np.array(joint_values)

        return self.fit_joint_values(joint_values)

    def compute_grasp(self, alphas):
        if not hasattr(self._pca, 'components_'):
            rospy.logwarn('No grasp synergies, did you call fit_joint_*?')
            return None
        pass




    pass
