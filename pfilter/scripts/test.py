import plotutil as ptl

import unittest
import math

class TestTransformations(unittest.TestCase):

    test_values = (
                  ( (10, 10, math.pi / 6), (-100, 100, -math.pi / 6) ),
                  ( (0,0,0), (0,0,0) ),
                  )

    def test_stage_to_cv(self):
        tr = ptl.Transformer(50,50,500,500)

        for args, correct_rets in self.test_values:
            ret_tuple = tr.stage_to_cv(args)
            self.assertEqual(len(ret_tuple), len(correct_rets))

            for i in xrange(len(ret_tuple)):
                self.assertAlmostEqual(ret_tuple[i], correct_rets[i])

        self.assertEqual(type(tr.stage_to_cv((0,0,0))), tuple)

    def test_add_angle(self):
        self.assertAlmostEqual(ptl.add_angle( math.pi / 2, math.pi), - math.pi / 2)
        self.assertAlmostEqual(ptl.add_angle( .5, .5), 1.0)
        self.assertAlmostEqual(ptl.add_angle( - math.pi / 2, math.pi), math.pi / 2)

if __name__ == '__main__':
    unittest.main()
