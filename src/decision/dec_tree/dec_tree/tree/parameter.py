from enum import Enum


class NAV_STATUS(Enum):
    NOGOAL = 0
    SUCCEEDED = 1
    RUNNING =2
    FAILED = 3
    CANCELED = 4
    ERROR = 5

