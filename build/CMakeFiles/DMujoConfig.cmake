# - Config file for the DMujo package
# It defines the following variables
#  DMujo_INCLUDE_DIR - include directories for DMujo
#  DMujo_SRC_DIR     - src     directories for DMujo
#  DMujo_LIB_DIR     - lib     directories for DMujo
 
# Compute paths
if(1)
    set(DMujo_INCLUDE_DIR "C:/Program Files (x86)/DBase/DMujo/include")
endif()
if(0)
    set(DMujo_LIB_DIR     "C:/Program Files (x86)/DBase/DMujo/lib"    )
endif()
if(0)
    set(DMujo_SRC_DIR     "C:/Program Files (x86)/DBase/DMujo/src"    )
endif()

# set paths
message("-- [DMujo]: Package found!")
if(DEFINED DMujo_INCLUDE_DIR)
    include_directories(${DMujo_INCLUDE_DIR})
    message("-- [DMujo]: Include ${DMujo_INCLUDE_DIR}")
endif()
if(DEFINED DMujo_LIB_DIR)
    link_directories(${DMujo_LIB_DIR})
    message("-- [DMujo]: Link ${DMujo_LIB_DIR}")
endif()
if(DEFINED DMujo_SRC_DIR)
    aux_source_directory(${DMujo_SRC_DIR} DMujo_FILES)
    message("-- [DMujo]: Can use DMujo_SRC_FILES")
endif()
