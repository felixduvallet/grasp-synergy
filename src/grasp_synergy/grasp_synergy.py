import numpy as np
from sklearn.decomposition import PCA

import rospy
import rosbag


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

    @property
    def trained(self):
        """
        :return True if the synergy space has been computed.
        """
        return hasattr(self._pca, 'components_')

    def fit_joint_values(self, joint_values):
        """
        Fit the principal components of the given joint values to compute the
        grasp synergies (aka eigengrasps).

        We compute and store *all* D principal components here. The
        dimensionality of the lower-dimensional subspace is determined at grasp
        computation time.

        :param joint_values A numpy array (N by D) with N datapoints of D
        dimensions.

        :return True if the synergies were properly fit.

        """

        assert isinstance(joint_values, np.ndarray), 'Must have np.array().'

        if joint_values.size <= 0:
            return False

        self._N = joint_values.shape[0]
        self._D = joint_values.shape[1]
        self._transformed_joint_angles = self._pca.fit_transform(joint_values)

        rospy.loginfo('Learned synergy space (from {}-dims) with {} points.'.
                      format(self._D, self._N))
        rospy.loginfo(' Explained variance ratio: {}\b... ]'.format(
            self._pca.explained_variance_ratio_[0:4]))

        return True

    def fit_joint_state_messages(self, joint_state_messages):
        """
        Extract joint state values from a list of ROS messages, then compute the
        grasp synergies.

        :param joint_state_messages A list of ROS sensor_msgs/JointState.
        """

        joint_values = []
        for (idx, msg) in enumerate(joint_state_messages):
            try:  # Catch any exceptions due to a wrong message type.
                joint_values.append(msg.position)
            except AttributeError:
                rospy.logwarn('Message is not a JointState, skipping.')
                pass
        # Create a numpy array of size NxD.
        joint_values = np.array(joint_values)

        return self.fit_joint_values(joint_values)

    def fit_bag_file(self, bag_filepath):
        """
        Extract *all* joint state messages from the given bag file, then compute
        the hand synergies.

        :param bag_filepath: Fully-qualified filepath of the bagfile.
        """

        try:
            bag = rosbag.Bag(bag_filepath)
        except IOError:
            rospy.logerr('Could not open file: {}'.format(bag_filepath))
            return False

        # Collect *all* messages of type JointState.
        joint_messages = []
        for (topic, msg, time) in bag.read_messages():
            if 'JointState' in str(type(msg)):
                joint_messages.append(msg)

        return self.fit_joint_state_messages(joint_messages)

    def compute_grasp(self, alphas):
        """
        Reconstruct a grasp given a combination of (low-dimensional) synergy
        coefficients to get a (full-dimensional) grasp configuration.

        The coefficients are a weighting vector of the various (ordered)
        principal grasp components.

        :return mean + sum_i alpha_i * coeff_i. If the synergy is not already
        computed this returns None.

        """

        if not hasattr(self._pca, 'components_'):
            rospy.logwarn('No grasp synergies, did you call fit_joint_*?')
            return None

        num_components = len(self._pca.components_)
        num_alpha = len(alphas)

        rospy.logdebug('Computing a grasp using {} principal components'.
                       format(min(num_alpha, num_components)))

        # Compute mean + dot<alphas, components>. Truncate both alphas and
        # components to ensure we have correct shapes. The resulting vector
        # is a D-dimensional vector.
        ret = self._pca.mean_ + np.dot(alphas[0:num_components],
                                       self._pca.components_[0:num_alpha])

        return ret

    def synergy_range(self, component_num):
        """
        Compute the range of values for the i'th component of the synergy, using
        the transformed original values used to train the PCA.

        If there are no synergies or the component number is invalid, returns
        (0, 0).

        :return (min, max) for transformed values
        """

        transformed_min, transformed_max = 0.0, 0.0

        # If there are no synergies, D = 0 so this does not get called.
        if 0 <= component_num < self._D:
            transformed_min = \
                np.min(self._transformed_joint_angles, axis=0)[component_num]
            transformed_max = \
                np.max(self._transformed_joint_angles, axis=0)[component_num]
        return transformed_min, transformed_max
