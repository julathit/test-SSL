import py_trees
from py_trees.common import Status

class Behaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Behaviour, self).__init__(name)

    def update(self):
        return Status.SUCCESS