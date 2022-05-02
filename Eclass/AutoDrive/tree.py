class Node():
    def __init__(self, val):
        self.val = val
        self.left = None
        self.right = None
        self.depth = 0

class BST():
    def __init__(self):
        self.root = None
    
    def setRoot(self, val):
        self.root = Node(val)

    def retreive(self, val):
        """검색 기능"""
        if (self.retreive_node(self.root, val)) is False:
            return False
        return True

    def retreive_node(self, current_node, val):
        if current_node is None:
            return False
        elif val == current_node.val:
            return current_node
        elif val < current_node.val:
            return self.retreive_node(current_node.left, val)
        else:
            return self.retreive_node(current_node.right, val)


    def insert(self, val):
        """삽입 기능"""
        if (self.root is None):
            self.setRoot(val)
        else:
            self.insert_node(self.root, val)

    def insert_node(self, current_node, val):
        if val <= current_node.val:
            if (current_node.left):
                self.insert_node(current_node.left, val)
            else:
                current_node.left = Node(val)

        elif val > current_node.val:
            if current_node.right:
                self.insert_node(current_node.right, val)
            else:
                current_node.right = Node(val)

    def traverse(self):
        return self.traverse_node(self.root)
    
    def traverse_node(self, current_node):
        result = []
        if current_node.left is not None:
            result.extend(self.traverse_node(current_node.left))
        if current_node is not None:
            result.extend([current_node.val])
        if current_node.right is not None:
            result.extend(self.traverse_node(current_node.right))
        return result

def main():
    test = [4,1,2,5,6,8,10]
    bst = BST()
    for t in test:
        bst.insert(t)

    print(bst.traverse())
if __name__ == "__main__":
    main()