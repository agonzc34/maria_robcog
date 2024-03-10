from typing import Any
from threading import Lock
from yasmin.blackboard import Blackboard

class LBlackboard(Blackboard):
    def __init__(self, init=None) -> None:
        self.lock = Lock()
        if init is not None:
            self.__dict__.update(init)

    def __getitem__(self, key) -> Any:
        self.lock.acquire()
        value = self.__dict__[key]
        self.lock.release()
        return value

    def __setitem__(self, key, value) -> None:
        self.lock.acquire()
        self.__dict__[key] = value
        self.lock.release()

    def __delitem__(self, key) -> None:
        self.lock.acquire()
        del self.__dict__[key]
        self.lock.release()

    def __contains__(self, key) -> bool:
        self.lock.acquire()
        value = key in self.__dict__
        self.lock.release()
        return value

    def __len__(self) -> int:
        self.lock.acquire()
        value = len(self.__dict__)
        self.lock.release()
        return value

    def __repr__(self) -> str:
        self.lock.acquire()
        value = repr(self.__dict__)
        self.lock.release()
        return value