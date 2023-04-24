#!/usr/bin/python3


"""
Custom exceptions used by the supervisor package.
"""


# -------------------------------- Base Classes --------------------------------


class SupervisorError(Exception):

    """
    Base class for all supervisor exceptions.
    """

    pass


class ModuleError(SupervisorError):

    """
    Subclass of SupervisorError. Base class for all module exceptions.
    """

    pass


class HandlerError(SupervisorError):

    """
    UNUSED
    Subclass of SupervisorError. Base class for all module handler exceptions.
    """

    pass


# --------------------------------- Sub Classes ---------------------------------


class ModuleLaunchError(ModuleError):

    """
    Subclass of ModuleError.
    Errors related to the launching of a module.
    """

    pass


class ModuleShutdownError(ModuleError):

    """
    Subclass of ModuleError.
    Errors related to the shutting down of a module.
    """

    pass


class ModuleStatusError(ModuleError):

    """
    Subclass of ModuleError.
    Errors related to the shutting down of a module.
    """

    pass
