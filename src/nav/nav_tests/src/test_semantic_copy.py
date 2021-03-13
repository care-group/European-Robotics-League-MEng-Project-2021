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
        # if item already exists
        self.sem.add_new_to_semantic_map({"name": "test", "type":"", "coords": [1, 2, 3], "others":{}})
        self.assertTrue(self.sem.add_new_to_semantic_map({"name": "test", "type":"", "coords": [1, 2, 3], "others":{}}))
        self.assertTrue(self.sem.add_new_to_semantic_map({"name": "test", "type":"", "coords": [2, 2, 3], "others":{}}, 3))

    
    def test_delete_by_coords(self):
        # Basic exact
        self.sem.add_new_to_semantic_map({"name": "", "type":"", "coords": [1, 2, 3], "others":{}})
        self.assertNotEqual(self.sem.delete_from_semantic_map_by_coords([1, 2, 3]), None)
        # Tuple test
        self.sem.add_new_to_semantic_map({"name": "", "type":"", "coords": [1, 2, 3], "others":{}})
        self.assertNotEqual(self.sem.delete_from_semantic_map_by_coords((1, 2, 3)), None)
        # Remove something that isn't there
        self.assertEqual(self.sem.delete_from_semantic_map_by_coords((1000, 1000, 1000)), False)
        # Not bad inputs
        self.assertEqual(self.sem.delete_from_semantic_map_by_coords([]), False)
        self.assertEqual(self.sem.delete_from_semantic_map_by_coords((1000, 1000)), False)
        # Threshold testing
        self.sem.add_new_to_semantic_map({"name": "", "type":"", "coords": [1, 2, 3], "others":{}})
        self.assertNotEqual(self.sem.delete_from_semantic_map_by_coords((1.1, 2, 3)), None)
        self.sem.add_new_to_semantic_map({"name": "", "type":"", "coords": [2, 2, 3], "others":{}})
        self.assertNotEqual(self.sem.delete_from_semantic_map_by_coords((1, 2, 3), distance_threshold=3), None)



if __name__ == '__main__':
    unittest.main()