class Node():
    def __init__(self, data, point_id):
        self.point = (data[0], data[1])
        self.point_id = point_id
        self.left_node = None
        self.right_node = None
        
class KdTree():
    def __init__(self):
        self.root = None
        self.kdtree_display_dict = {}
        
    def insert_points(self, datas, display_output=False):
        for i, data in enumerate(datas):
            point_id = i
            point = tuple(data[0], data[1])
            level = 0
            self.root = self.build_kdtree(self.root, level, point, point_id)
        
        if display_output:
            print("KdTree Build Complete!")
            self.display_kdtree(self.root)
            for pair in self.kdtree_display_dict.items():
                print(f"Depth {pair[0]}, Points={pair[1]}")
                
    def build_kdtree(self, node, depth, point, point_id):
        if node is None:
            node = Node(point, point_id)
        
        current_node = Node(point, point_id)
    