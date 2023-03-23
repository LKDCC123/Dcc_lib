# change these values -------------
set(PKG_NAME DBase)
set(${PKG_NAME}_INCLUDE_ON  1)
set(${PKG_NAME}_LIB_ON      0)
set(${PKG_NAME}_SRC_ON      0)
# ---------------------------------
set(PKG_DIR ${CMAKE_SOURCE_DIR})
message("-- Package name is: ${PKG_NAME}")

set(${PKG_NAME}_include_dirs    ${CMAKE_INSTALL_PREFIX}/${PKG_NAME}/include)
set(${PKG_NAME}_lib_dirs        ${CMAKE_INSTALL_PREFIX}/${PKG_NAME}/lib)
set(${PKG_NAME}_src_dirs        ${CMAKE_INSTALL_PREFIX}/${PKG_NAME}/src)
set(${PKG_NAME}_cmake_dirs      ${CMAKE_INSTALL_PREFIX}/${PKG_NAME}/share)

set(CONFIG_BUILD_FILE ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PKG_NAME}Config.cmake)
configure_file(Config.cmake.in ${CONFIG_BUILD_FILE} @ONLY)
configure_file(${CONFIG_BUILD_FILE} ${CONFIG_BUILD_FILE} @ONLY)

if(${PKG_NAME}_INCLUDE_ON) 
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/include 
        DESTINATION  ${CMAKE_INSTALL_PREFIX}/${PKG_NAME}
    )
endif()

if(${PKG_NAME}_LIB_ON) 
    aux_source_directory(${CMAKE_SOURCE_DIR}/src SRC_FILES)
    add_library(${PROJECT_NAME} STATIC ${SRC_FILES})
    install(FILES ${PROJECT_BINARY_DIR}/Release/${PROJECT_NAME}.lib 
        DESTINATION ${CMAKE_INSTALL_PREFIX}/${PKG_NAME}/lib)
endif()

if(${PKG_NAME}_SRC_ON) 
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/src
        DESTINATION ${CMAKE_INSTALL_PREFIX}/${PKG_NAME})
endif()

install(FILES ${CONFIG_BUILD_FILE} 
    DESTINATION ${CMAKE_INSTALL_PREFIX}/${PKG_NAME}/share/${PKG_NAME} COMPONENT dev
)