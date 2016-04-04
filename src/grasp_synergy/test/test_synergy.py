import unittest

from grasp_synergy.grasp_synergy import GraspSynergy


class TestCase(unittest.TestCase):

    def setUp(self):
        pass

    def test_constructor(self):
        gs = GraspSynergy()
        self.assertIsNotNone(gs)


if __name__ == '__main__':
    unittest.main()
