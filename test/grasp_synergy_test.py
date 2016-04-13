#!/usr/bin/env python
import rostest
import unittest
import rospy

class SynergySelfCheck(unittest.TestCase):
    """
    Very basic self-test capabilities.

    NOTE: currently these aren't run because of deadlock problems in creating
    the topic subscribers.

    """

    def setUp(self):
        # Wait for topics
        self.topic_dict = {}
        while '/test_synergy/grasp_synergy' not in self.topic_dict:
            _, _, topic_types = rospy.get_master().getTopicTypes()
            self.topic_dict = dict(topic_types)
            print self.topic_dict

            rospy.sleep(0.02)

    def test_topics(self):
        # Make sure we have the right number of topics in the appropriate
        # test_synergy namespace.
        self.assertIn('/test_synergy/grasp_synergy', self.topic_dict)
        self.assertIn('/test_synergy/grasp_synergy/syn_0', self.topic_dict)
        self.assertIn('/test_synergy/grasp_synergy/syn_1', self.topic_dict)
        self.assertIn('/test_synergy/grasp_synergy/syn_2', self.topic_dict)
        self.assertNotIn('/test_synergy/grasp_synergy/syn_3', self.topic_dict)


if __name__ == '__main__':
    import rostest
    rospy.init_node('grasp_synergy_test')
    rostest.rosrun('grasp_synergy', 'grasp_synergy_test', SynergySelfCheck)
