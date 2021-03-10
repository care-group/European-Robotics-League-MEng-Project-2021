import unittest
from semantic_copy import SemanticToCoords

class Test_Map(unittest.TestCase):
    def setUp(self):
        self.sem = SemanticToCoords()

    def tearDown(self):
        #self.sem.save_semantic_map()
        pass

    def test_additions(self):
        # Minimum input
        self.assertTrue(self.sem.add_new_to_semantic_map({"name": "", "type":"", "coords": [], "others":{}}))
        # Missing keys
        self.assertFalse(self.sem.add_new_to_semantic_map({"name": ""}))
        self.assertFalse(self.sem.add_new_to_semantic_map({"type": ""}))
        # Keys with wrong value types
        self.assertFalse(self.sem.add_new_to_semantic_map({"name": "", "type":"", "coords": "", "others":{}}))
        # More inputs
        self.assertTrue(self.sem.add_new_to_semantic_map({"name": "", "type":"", "in":"", "coords": [], "others":{}}))


if __name__ == '__main__':
    unittest.main()