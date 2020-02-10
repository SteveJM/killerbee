import unittest
import os, sys
_root_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, _root_dir)
from rangeparser import *

class RangeParserTest(unittest.TestCase):
    def setUp(self):
        self.parser = RangeParser()
    
    def test_basic_list(self):
        self.assertEqual(self.parser.parse('10,20,30'), [10, 20, 30])
    
    def test_basic_range(self):
        self.assertEqual(self.parser.parse('1-5'), [1, 2, 3, 4, 5])
    
    def test_list_of_ranges(self):
        self.assertEqual(self.parser.parse('1-5,10-15'), [1, 2, 3, 4, 5, 10, 11, 12, 13, 14, 15])
    
    def test_zero(self):
        self.assertEqual(self.parser.parse('0, 0-5'), [0, 0, 1, 2, 3, 4, 5])
    
    def test_validation(self):
        self.assertRaises(ValueError, self.parser.parse, 'hello world')
    
    def test_range_end_lower_than_start(self):
        self.assertRaises(ValueError, self.parser.parse, '10-5')
    

if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(RangeParserTest)
    unittest.TextTestRunner(verbosity=5).run(suite)