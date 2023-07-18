include(ExternalProject)

set(DEPENDENCY_INSTALL_DIR ${PROJECT_BINARY_DIR}/install)
set(DEPENDENCY_INCLUDE_DIR ${DEPENDENCY_INSTALL_DIR}/include)
set(DEPENDENCY_LIB_DIR ${DEPENDENCY_INSTALL_DIR}/lib)

# ------------------ HELPER FUNCTIONS ------------------ #
function(add_dep dep)
    set(DEPENDENCY_LIST ${DEPENDENCY_LIST} ${dep} PARENT_SCOPE)
endfunction()

function(add_include include_dir)
    # add / at the end or else cmake will think that we want to copy the parent folder
    if (NOT (${include_dir} MATCHES ".*/$"))
        set(include_dir "${include_dir}/")
    endif()

    set(DEPENDENCY_INCLUDE_LIST ${DEPENDENCY_INCLUDE_LIST} ${include_dir} PARENT_SCOPE)
endfunction()

function(add_lib lib)
    set(DEPENDENCY_LIBS ${DEPENDENCY_LIBS} ${lib} PARENT_SCOPE)
endfunction(add_lib)

function(add_compilation_option option)
    set(OBSERVER_DEPENDENCY_COMPILATION_OPTIONS ${OBSERVER_DEPENDENCY_COMPILATION_OPTIONS} ${option} PARENT_SCOPE)
endfunction(add_compilation_option)

# ----------------------- OPENCV ----------------------- #
find_package(OpenCV REQUIRED)

add_lib("${OpenCV_LIBS}") # OpenCV_LIBS is a list so quote it

# ---------------------- LIBWEBRTC --------------------- #
# depot tools is used to fetch webrtc
ExternalProject_Add(
    web-rtc-build
    SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/..
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/webrtc
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${DEPENDENCY_INSTALL_DIR}
        -DCMAKE_INSTALL_INCLUDEDIR=${DEPENDENCY_INCLUDE_DIR}
        -DCMAKE_INSTALL_LIBDIR=${DEPENDENCY_LIB_DIR}
        -DWEBRTC_BUILD_TYPE=${WEBRTC_BUILD_TYPE}
        -DWEBRTC_INCLUDE_DEFAULT_AUDIO=${WEBRTC_INCLUDE_DEFAULT_AUDIO}
    TEST_COMMAND ""
)

add_dep(web-rtc-build)
# add_lib(webrtc_world)
add_lib(webrtc)
add_lib(webrtc_extra)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    if (${WEBRTC_INCLUDE_DEFAULT_AUDIO})
        add_lib(X11)
        add_lib(Xext)
        add_lib(Xdamage)
        add_lib(Xfixes)
        add_lib(Xcomposite)
        add_lib(Xrandr)
        add_lib(Xtst)
    endif()
endif()