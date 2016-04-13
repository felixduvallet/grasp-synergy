#!/usr/bin/env python

import sys
import argparse
import numpy as np

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float32

from grasp_synergy import GraspSynergy


class GraspSynergyNode(object):
    """
    The grasp synergy node subscribes to low-dimensional synergies and publishes
    desired joint state.

    It is hand agnostic.
    """

    def __init__(self, joint_cmd_topic,
                 synergy_input_topic='grasp_synergy',
                 num_synergies=2):

        self._synergy = GraspSynergy()

        self._publisher = rospy.Publisher(
            joint_cmd_topic, JointState, queue_size=1)

        self._subscriber_main = None
        self._subscriber_components = []
        self._init_subscribers(synergy_input_topic, num_synergies)

    def fit_joint_values(self, joint_values):
        return self._synergy.fit_joint_values(joint_values)

    def fit_joint_state_messages(self, joint_state_messages):
        return self._synergy.fit_joint_state_messages(joint_state_messages)

    def fit_bag_file(self, bag_filepath):
        return self._synergy.fit_bag_file(bag_filepath)

    def command_synergy(self, synergy):

        if not self._synergy.trained:
            rospy.logwarn('Synergy is not initialized: did you provide '
                          'training data?')
            return

        grasp = self._synergy.compute_grasp(synergy)
        rospy.logdebug('Commanding synergy: {} -> grasp {}'.format(
            synergy, grasp))

        joint_state = JointState()
        joint_state.position = grasp
        self._publisher.publish(joint_state)

    def _callback_main(self, data):
        """
        Main callback for a fully-specified coefficient vector.
        Commands the synergy.
        """
        alpha = data.data
        self.command_synergy(alpha)

    def _callback_component(self, data, component_number):
        """
        Component callback (component number & value).

        Creates an alpha vector with all zeros except for the given component,
        then commands the synergy.
        """
        rospy.logdebug('Received data for component {} - {}'.format(
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
        under the top-level one: synergy_input_topic/

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


def run(arguments):
    parser = argparse.ArgumentParser(
        description='Command hands in a low-dimensional grasp synergy space.')
    parser.add_argument('--hand_control_topic', required=True,
                        help='Commanded joint state topic.')
    parser.add_argument('--bag_filename', required=True,
                        help='Filepath for bag file of grasp data')
    parser.add_argument('--num_synergies', default=4,
                        type=int,
                        help='Number of synergies.')
    args = parser.parse_args(rospy.myargv(arguments))

    rospy.init_node('grasp_synergy')

    hand_control_topic = args.hand_control_topic
    num_synergies = args.num_synergies
    fpath = args.bag_filename

    node = GraspSynergyNode(joint_cmd_topic=hand_control_topic,
                            num_synergies=num_synergies)
    if not node.fit_bag_file(fpath):
        return
    rospy.spin()


if __name__ == '__main__':
    arguments = sys.argv[1:]  # argv[0] is the program name.
    run(arguments)
