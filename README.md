Makes it easy to use chromium webrtc native from projects using cmake

If you intend to use webrtc in its entirety, I recommend that you use https://github.com/introlab/webrtc-native-build instead of this repository, mine is intended for a very narrow use case.

# Include it in your project
with cmake
```cmake

# Define your pats
# e.g.
set(DEPENDENCY_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/dependencies)
set(DEPENDENCY_INCLUDE_DIR ${DEPENDENCY_INSTALL_DIR}/include)
set(DEPENDENCY_LIB_DIR ${DEPENDENCY_INSTALL_DIR}/lib)

ExternalProject_Add(
    web-rtc-build
    SOURCE_DIR ${PATH_TO_THIS_REPO}
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${DEPENDENCY_INSTALL_DIR}
        -DCMAKE_INSTALL_INCLUDEDIR=${DEPENDENCY_INCLUDE_DIR}
        -DCMAKE_INSTALL_LIBDIR=${DEPENDENCY_LIB_DIR}
        -DWEBRTC_BUILD_TYPE=${WEBRTC_BUILD_TYPE}
        -DWEBRTC_INCLUDE_DEFAULT_AUDIO=${WEBRTC_INCLUDE_DEFAULT_AUDIO} # ON/OFF
    TEST_COMMAND ""
)

# add the following to your project/target(s)
# add web-rtc-build as dependency of your target(s)
# link libraries: webrtc, webrtc_extra
# link directories: ${DEPENDENCY_LIB_DIR}
# include directories: ${DEPENDENCY_INCLUDE_DIR}
# look at examples/Dependency.cmake for an example
```

# Run examples
- cd examples
- cmake ../examples -DCMAKE_BUILD_TYPE=Debug -DWEBRTC_BUILD_TYPE=Release -DWEBRTC_INCLUDE_DEFAULT_AUDIO=OFF
change as needed
- make 
- ./2_AIO_brodcast_from_cv
- double click the client html file to open it in a browser (client.html)
- put the offer generated by the server in the client and click send
- copy the answer generated by the client and paste it in the console