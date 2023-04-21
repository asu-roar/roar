#!/usr/bin/python3


class SupervisorError(Exception):
    
    """
    Implementation:
    ---------------
    This class implements a custom exception class called SupervisorError which
    is based on the base Exception class. To be used by all supervisor scripts and
    modules instead of other Exception classes.

    Usage:
    ------
    Supervisor modules will raise a SupervisorError when an error is encountered.
    You can handle this gracefully in your code through the following
    implementation:
    try:
        module.class.method()
    except SupervisorError:
        # Code to be executed when an error is encountered.
    """

    pass