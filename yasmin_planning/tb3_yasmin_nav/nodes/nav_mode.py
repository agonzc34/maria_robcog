import abc
from yasmin import State
from yasmin import StateMachine


class NavNode(State):
    def __init__(self, node):
        super().__init__(["loop", "end"])
        self.node = node

    def execute(self, blackboard):
        pass

    @abc.abstractmethod
    def select_nav_mode():
        pass
