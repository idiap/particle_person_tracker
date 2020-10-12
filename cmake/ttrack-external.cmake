# This include file only defines which dependencies we need, globally
set(TTRACK_DEPENDENCIES cv boost cmotion2d)
set(TTRACK_OPTIONALS python sphinx doxygen)

if (NOT NORSB)
    list(APPEND TTRACK_OPTIONALS rsb rsc protobuf rst)
endif (NOT NORSB)

if (NOT NOSTEREOMATCHER)
    list(APPEND TTRACK_OPTIONALS stereomatcher)
endif (NOT NOSTEREOMATCHER)

if (NOT NOVFOA)
    list(APPEND TTRACK_OPTIONALS vfoamodule)
endif (NOT NOVFOA)

if (NOT NOCTUFD)
    list(APPEND TTRACK_OPTIONALS ctufd)
endif (NOT NOCTUFD)

if (NOT NOTORCHFD)
    list(APPEND TTRACK_OPTIONALS torchfd)
endif (NOT NOTORCHFD)

if (NOT NONODDETECTOR)
    list(APPEND TTRACK_OPTIONALS noddetector)
endif (NOT NONODDETECTOR)
