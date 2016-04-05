import numpy as np

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float32



class GraspSynergyNode(object):
    def __init__(self, joint_cmd_topic, synergy_input_topic='/grasp_synergy',
                 num_synergies=2):

        self._publisher = rospy.Publisher(
            joint_cmd_topic, JointState, queue_size=1)

        self._subscriber_main = None
        self._subscriber_components = []
        self._init_subscribers(synergy_input_topic, num_synergies)

        pass

    def command_synergy(self, synergy):
        rospy.loginfo('Commanding synergy: {}'.format(synergy))


        pass

    def _callback_main(self, data):
        """
        Main callback for a fully-specified coefficient vector.
        Commands the synergy.
        """
        rospy.loginfo('Received data! {}'.format(data))
        alpha = data.data
        self.command_synergy(alpha)

    def _callback_component(self, data, component_number):
        """
        Component callback (component number & value).

        Creates an alpha vector with all zeros except for the given component,
        then commands the synergy.
        """
        rospy.loginfo('Received data for component {} - {}'.format(
            component_number, data.data))
        alpha = np.zeros((component_number + 1,))
        alpha[component_number] = data.data
        self.command_synergy(alpha)

    def _init_subscribers(self, synergy_input_topic, num_synergies=0):
        """
        Create num_synergies+1 subscribers. The main (top-level) subscriber
        listens for an array of floats.

        Each component subscriber only listens to the float corresponding to
        its component's singular value. Each component subscriber is nested
        under the top-level one: /synergy_input_topic/

        :param synergy_input_topic The top-level topic.

        :param num_synergies Number of component subscribers to generate.

        """

        # Main callback for arrays.
        self._subscriber_main = rospy.Subscriber(
            synergy_input_topic,  Float32MultiArray, self._callback_main)
        rospy.loginfo('Created main subscriber {}'.format(synergy_input_topic))

        # Component subscriber, using individual (nested) topic names and a
        # simple Float.
        for idx in range(num_synergies):
            topic = '{}/syn_{}'.format(synergy_input_topic, idx)

            subscriber = rospy.Subscriber(
                topic, Float32, self._callback_component, idx)
            self._subscriber_components.append(subscriber)
            rospy.loginfo('  Created component subscriber {}'.format(topic))
        pass

def run():
    rospy.init_node('grasp_synergy')
    node = GraspSynergyNode(joint_cmd_topic='/allegro_cmd', num_synergies=5)
    rospy.spin()


if __name__ == '__main__':
    run()
