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

    def test_trained_with_data(self):
        self.assertFalse(self.synergy.trained)
        joints = np.random.random((25, 5))
        self.synergy.fit_joint_values(joints)

        self.assertTrue(self.synergy.trained)

    def test_trained_no_data(self):
        joints = np.zeros((10, 0))
        self.synergy.fit_joint_values(joints)
        self.assertFalse(self.synergy.trained)

    def test_fit_joint_values(self):
        joints = np.random.random((25, 5))
        ret = self.synergy.fit_joint_values(joints)
        self.assertTrue(ret)
        self.assertEqual(5, self.synergy._D)
        self.assertEqual(25, self.synergy._N)
        self.assertEqual(5, len(self.synergy._pca.components_))

    def test_fit_joint_values_bad_type(self):
        joints = [[1, 2, 3], [4, 5, 6]]
        with self.assertRaisesRegexp(AssertionError, 'Must have'):
            self.synergy.fit_joint_values(joints)

    def test_fit_joint_values_empty(self):

        # Interesting fact about numpy arrays: len(joints) is 10 while
        # joints.size is 0.
        joints = np.zeros((10, 0))

        ret = self.synergy.fit_joint_values(joints)

        self.assertFalse(ret)
        self.assertEqual(0, self.synergy._D)
        self.assertEqual(0, self.synergy._N)
        self.assertFalse(self.synergy.trained)

    def test_fit_joint_messages(self):
        ret = self.synergy.fit_joint_state_messages(self.messages)
        self.assertTrue(ret)
        self.assertEqual(16, self.synergy._D)
        self.assertEqual(82, self.synergy._N)
        self.assertEqual(16, len(self.synergy._pca.components_))

    def test_fit_joint_messages_empty(self):
        messages = []  # List is okay.
        ret = self.synergy.fit_joint_state_messages(messages)
        self.assertFalse(ret)
        self.assertEqual(0, self.synergy._D)
        self.assertEqual(0, self.synergy._N)
        self.assertFalse(self.synergy.trained)

    def test_fit_bag_file(self):
        fpath = os.path.join(os.path.dirname(__file__), 'data', 'allegro.bag')
        ret = self.synergy.fit_bag_file(fpath)
        self.assertTrue(ret)
        self.assertEqual(16, self.synergy._D)
        self.assertEqual(82, self.synergy._N)
        self.assertEqual(16, len(self.synergy._pca.components_))

    def test_bag_file_nonexistent(self):
        ret = self.synergy.fit_bag_file('/not/a/file')
        self.assertFalse(ret)
        self.assertEqual(0, self.synergy._D)
        self.assertEqual(0, self.synergy._N)
        self.assertFalse(self.synergy.trained)

    def test_compute_grasp_no_data(self):
        ret = self.synergy.compute_grasp([0, 1, 3])
        self.assertIsNone(ret)

    def test_compute_grasp_shape(self):
        self.synergy.fit_joint_state_messages(self.messages)
        ret = self.synergy.compute_grasp([1.0])
        self.assertIsNotNone(ret)
        self.assertEqual((16,), ret.shape)

    def test_compute_grasp_zero_alphas(self):
        self.synergy.fit_joint_state_messages(self.messages)
        ret = self.synergy.compute_grasp([])
        ref = self.synergy._pca.mean_
        np.testing.assert_array_almost_equal(ref, ret)

    def test_compute_grasp_sum_alphas(self):
        self.synergy.fit_joint_state_messages(self.messages)
        ret = self.synergy.compute_grasp([1.0, 1.0])
        ref = (self.synergy._pca.components_[0] +
               self.synergy._pca.components_[1] + self.synergy._pca.mean_)
        np.testing.assert_array_almost_equal(ref, ret)

    def test_compute_grasp_many_alphas(self):
        # Make sure we can pass in a large vector of coefficients without
        # failing.
        self.synergy.fit_joint_state_messages(self.messages)
        alphas = np.ones((100,))
        ret = self.synergy.compute_grasp(alphas)
        ref = (np.sum(self.synergy._pca.components_, axis=0) +
               self.synergy._pca.mean_)
        np.testing.assert_array_almost_equal(ref, ret)

    def test_component_ranges_0(self):
        self.synergy.fit_joint_state_messages(self.messages)
        (ret_min, ret_max) = self.synergy.synergy_range(0)
        self.assertAlmostEqual(-1.18130, ret_min, places=4)
        self.assertAlmostEqual(1.12507406, ret_max, places=4)

    def test_component_ranges_1(self):
        self.synergy.fit_joint_state_messages(self.messages)
        (ret_min, ret_max) = self.synergy.synergy_range(1)
        self.assertAlmostEqual(-0.41370870, ret_min, places=4)
        self.assertAlmostEqual(0.4547809556, ret_max, places=4)

    def test_component_ranges_untrained(self):  # try before training.
        (ret_min, ret_max) = self.synergy.synergy_range(1)
        self.assertEqual(0, ret_min)
        self.assertEqual(0, ret_max)

    def test_component_ranges_negative(self):
        (ret_min, ret_max) = self.synergy.synergy_range(-1)
        self.assertEqual(0, ret_min)
        self.assertEqual(0, ret_max)

    def test_component_ranges_too_big(self):
        (ret_min, ret_max) = self.synergy.synergy_range(100)
        self.assertEqual(0, ret_min)
        self.assertAlmostEqual(0, ret_max)


if __name__ == '__main__':
    unittest.main()
