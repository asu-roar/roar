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
    Subclass of SupervisorError. Base class for all module handler exceptions.
    """

    pass


# --------------------------------- Module Sub Classes ---------------------------------


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
    Errors related to performing module operations in an invalid state.
    """

    pass

# --------------------------------- Handler Sub Classes ---------------------------------


class HandlerInitError(HandlerError):

    """
    Subclass of HandlerError.
    Errors related to the initializing of a ModuleHandler.
    """

    pass


class HandlerLoadError(HandlerError):

    """
    Subclass of HandlerError.
    Errors related to the loading of modules using a ModuleHandler.
    """

    pass


class HandlerStatusError(HandlerError):

    """
    Subclass of HandlerError.
    Errors related to the status of a Module from the ModuleHandler.
    """

    pass

class HandlerLaunchError(HandlerError):

    """
    Subclass of HandlerError.
    Errors related to launching Modules using a ModuleHandler.
    """

    pass


class HandlerShutdownError(HandlerError):
    
    """
    Subclass of HandlerError.
    Errors related to shutting down Modules using a ModuleHandler.
    """

    pass