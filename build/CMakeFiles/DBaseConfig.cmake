# - Config file for the DBase package
# It defines the following variables
#  DBase_INCLUDE_DIR - include directories for DBase
#  DBase_SRC_DIR     - src     directories for DBase
#  DBase_LIB_DIR     - lib     directories for DBase
 
# Compute paths
if(1)
    set(DBase_INCLUDE_DIR "D:/_pkg/DPackages/DBase/include")
endif()
if(0)
    set(DBase_LIB_DIR     "D:/_pkg/DPackages/DBase/lib"    )
endif()
if(0)
    set(DBase_SRC_DIR     "D:/_pkg/DPackages/DBase/src"    )
endif()

# set paths
message("-- [DBase]: Package found!")
if(DEFINED DBase_INCLUDE_DIR)
    include_directories(${DBase_INCLUDE_DIR})
    message("-- [DBase]: Include ${DBase_INCLUDE_DIR}")
endif()
if(DEFINED DBase_LIB_DIR)
    link_directories(${DBase_LIB_DIR})
    message("-- [DBase]: Link ${DBase_LIB_DIR}")
endif()
if(DEFINED DBase_SRC_DIR)
    aux_source_directory(${DBase_SRC_DIR} DBase_FILES)
    message("-- [DBase]: Can use DBase_SRC_FILES")
endif()
