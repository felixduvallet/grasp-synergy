import unittest
import os
import rosbag
import numpy as np

from grasp_synergy.grasp_synergy import GraspSynergy


class TestCase(unittest.TestCase):

    def setUp(self):
        (_, self.messages, _) = zip(*self.data)
        self.synergy = GraspSynergy()
        pass

    @classmethod
    def setUpClass(cls):
        fpath = os.path.join(os.path.dirname(__file__), 'data', 'allegro.bag')
        bag = rosbag.Bag(fpath)
        topics = ['/allegroHand_0/joint_states']
        cls.data = list(bag.read_messages(topics))
        bag.close()
        pass

    def test_constructor(self):
        self.assertIsNotNone(self.synergy)
        self.assertEqual(0, self.synergy._D)
        self.assertEqual(0, self.synergy._N)

    def test_fit_joint_values(self):
        joints = np.random.random((25, 5))
        self.synergy.fit_joint_values(joints)
        self.assertEqual(5, self.synergy._D)
        self.assertEqual(25, self.synergy._N)
        self.assertEqual(5, len(self.synergy._pca.components_))

    def test_fit_joint_messages(self):
        self.synergy.fit_joint_state_messages(self.messages)
        self.assertEqual(16, self.synergy._D)
        self.assertEqual(82, self.synergy._N)
        self.assertEqual(16, len(self.synergy._pca.components_))

    def test_compute_grasp_no_data(self):
        ret = self.synergy.compute_grasp([0, 1, 3])
        self.assertIsNone(ret)




if __name__ == '__main__':
    unittest.main()
