cmake_minimum_required(VERSION 3.14)

project(webrtc_build)

if (NOT WEBRTC_BUILD_TYPE)
    message(FATAL_ERROR "WEBRTC_BUILD_TYPE not defined")
    # CMAKE_BUILD_TYPE not reliable in this context.
    # I need explicit control over the build type.
endif()

# OFF: set it off in any of this two cases
#   1. You need more control over the audio module than the default impl provides
#   2. You won't use audio.
# 
# ON:
#   If you not provide an audio module to peer connection manager, then
#   it will use the default audio module (adm). For that it requires ALSA or PulseAudio
#   on linux. 
# 
# Anyway you can provide a fake adm for both ON/OFF.
option(WEBRTC_INCLUDE_DEFAULT_AUDIO "Use the default audio module if not provieded" ON)

# -------------------------------------------------------- #
#                      SET DIRECTORIES                     #
# -------------------------------------------------------- #
set(LIBWEBRTC_ALL_DIR ${CMAKE_BINARY_DIR}/webrtc-bundle)
set(LIBWEBRTC_WEBRTC_DIR ${LIBWEBRTC_ALL_DIR}/webrtc/src)
set(LIBWEBRTC_WEBRTC_BUILD_DIR ${LIBWEBRTC_WEBRTC_DIR}/out/${WEBRTC_BUILD_TYPE})
set(LIBWEBRTC_WEBRTC_BUILD_BIN_DIR ${LIBWEBRTC_WEBRTC_DIR}/out/${WEBRTC_BUILD_TYPE}/obj)
set(LIBWEBRTC_WEBRTC_INCLUDE_DIR ${DEPENDENCY_INCLUDE_DIR}/webrtc)
set(WEBRTC_INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR}/webrtc)
set(WEBRTC_INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR})

file(MAKE_DIRECTORY ${LIBWEBRTC_ALL_DIR})

# -------------------------------------------------------- #
#                          HELPERS                         #
# -------------------------------------------------------- #
function(convert_cmake_boolean_to_string input output)
    if(${input})
        set(${output} "true" PARENT_SCOPE)
    else()
        set(${output} "false" PARENT_SCOPE)
    endif()
endfunction()

# macro that replaces ; with a space
macro(convert_list_to_string input output)
    string (REPLACE ";" " " ${output} "${input}")
endmacro()

# wrapper of execute_process to print the system status
function(execute_proc_info)
    set(SINGLE_VAL_ARGS WORKING_DIRECTORY ERROR_VARIABLE)
    set(MULTI_VAL_ARGS COMMAND)
    cmake_parse_arguments(EP_WRAPPER "" "${SINGLE_VAL_ARGS}"
                          "${MULTI_VAL_ARGS}" ${ARGN})

    convert_list_to_string("${EP_WRAPPER_COMMAND}" EP_WRAPPER_COMMAND_STR)
    message(STATUS "Running: ${EP_WRAPPER_COMMAND_STR}")
    message("\tWorking directory: ${EP_WRAPPER_WORKING_DIRECTORY}")
    execute_process(
        COMMAND ${EP_WRAPPER_COMMAND}
        WORKING_DIRECTORY ${EP_WRAPPER_WORKING_DIRECTORY}
        # OUTPUT_VARIABLE _OUTPUT # uncomment to debug command
        RESULT_VARIABLE _RESULT
        # ERROR_VARIABLE _ERROR # uncomment to debug command
        # NOTE: cmake doesn't show the ouput unless output/error_variable is not set
    )
    if (NOT _RESULT EQUAL 0)
        message(FATAL_ERROR "Error (check above): " ${_RESULT}) # ${_OUTPUT} ${_RESULT}
        
        if (EP_WRAPPER_ERROR_VARIABLE)
            set(${EP_WRAPPER_ERROR_VARIABLE} ${_ERROR} PARENT_SCOPE)
        endif()
    endif()
    message(STATUS "DONE.")
endfunction()

macro(install_symlink _target _link)
    # install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_sym_link ${_target} ${_link})")
    INSTALL(CODE "execute_process( \
                COMMAND ${CMAKE_COMMAND} -E create_symlink \
                ${_target} \
                ${_link}   \
                )"
            )
    install(CODE "message(STATUS \"Created symlink: ${_link} -> ${_target}\")")
endmacro()

# find git
find_package(Git)
if(GIT_FOUND)
    message(STATUS "Git found: ${GIT_EXECUTABLE}")
else()
    message(FATAL_ERROR "Git not found")
endif()

# -------------------------------------------------------- #
#                     GET WEBRTC SOURCE                    #
# -------------------------------------------------------- #
if (NOT EXISTS ${LIBWEBRTC_ALL_DIR}/depot_tools)
    execute_proc_info(
        COMMAND ${GIT_EXECUTABLE} clone https://chromium.googlesource.com/chromium/tools/depot_tools.git
        WORKING_DIRECTORY ${LIBWEBRTC_ALL_DIR}
    )
endif()

message(STATUS LIBWEBRTC_WEBRTC_BUILD_DIR=${LIBWEBRTC_WEBRTC_BUILD_DIR})

# Ensure that "depot_tools" is somewhere in PATH
LIST(APPEND CMAKE_PROGRAM_PATH ${LIBWEBRTC_ALL_DIR}/depot_tools)

find_program(FETCH_EXECUTABLE fetch) # comes from depot_tools

# -------------------------------------------------------- #
#                     GET WEBRTC SOURCE                    #
# -------------------------------------------------------- #
# check if we already have the source code
file(GLOB RESULT ${LIBWEBRTC_ALL_DIR}/webrtc)
list(LENGTH RESULT RES_LEN)

if (RES_LEN EQUAL 0)
    message(STATUS "Fetching WebRTC... This may take a while")

    # ensure it exists, else it cannot be used as wd
    file(MAKE_DIRECTORY ${LIBWEBRTC_ALL_DIR}/webrtc)
    execute_proc_info(
        COMMAND ${FETCH_EXECUTABLE} --no-history webrtc
        WORKING_DIRECTORY ${LIBWEBRTC_ALL_DIR}/webrtc
    )
endif()

# ------------------------------------------------------ #
#              GENERATES WEBRTC BUILD FILES              #
# ------------------------------------------------------ #

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_package(X11 QUIET)
    message(STATUS "WITH X11=${X11_FOUND}")

    # check if host has alsa lib
    execute_process(COMMAND ldconfig -p 
                    COMMAND grep libasound.so
                    COMMAND wc -l
                    OUTPUT_VARIABLE ALSA_FOUND
                    OUTPUT_STRIP_TRAILING_WHITESPACE)

    # check if host has pulseaudio lib
    execute_process(COMMAND ldconfig -p 
                    COMMAND grep libpulse.so 
                    COMMAND wc -l
                    OUTPUT_VARIABLE PulseAudio_FOUND
                    OUTPUT_STRIP_TRAILING_WHITESPACE)
    
    # Result variable == 0 -> false
    message(STATUS "WITH ALSA=${ALSA_FOUND} > 0")
    message(STATUS "WITH PulseAudio=${PulseAudio_FOUND} > 0")

    if ((NOT ${ALSA_FOUND} OR NOT ${PulseAudio_FOUND})
        AND ${WEBRTC_INCLUDE_DEFAULT_AUDIO})
        message(FATAL_ERROR "Requested WEBRTC with audio by default, but in linux it requires"
        "ALSA or PulseAudio and none was found.")
    endif()
endif()

if(${WEBRTC_BUILD_TYPE} MATCHES "Debug")
    set(IS_DEBUG ON)
else()
    set(IS_DEBUG OFF)
endif()

convert_cmake_boolean_to_string(${X11_FOUND} X11_FOUND_STR)
convert_cmake_boolean_to_string(${IS_DEBUG} GN_IS_DEBUG_STR)

convert_cmake_boolean_to_string(${WEBRTC_INCLUDE_DEFAULT_AUDIO} WEBRTC_INCLUDE_DEFAULT_AUDIO_STR)

# convert_cmake_boolean_to_string(${ALSA_FOUND} ALSA_FOUND_STR) # alsa is enabled by default
convert_cmake_boolean_to_string(${PulseAudio_FOUND} PulseAudio_FOUND_STR)

# NOTE: If you change any of this arguments, you need to delete the build directory
#       (${LIBWEBRTC_WEBRTC_BUILD_BIN_DIR}) in order to it to take effect.
set(GN_ARGS
        is_debug=${GN_IS_DEBUG_STR} # lowercase
        use_rtti=true # false by default. It doesn't allow type(T). OpenCV uses RTTI
                        # and opencv cannot be built without it unless you modify the
                        # source or disable some modules like flann.
        rtc_enable_protobuf=false
        rtc_build_examples=false
        rtc_build_tools=false
        treat_warnings_as_errors=false
        use_custom_libcxx=false # used for cross compile, false = system standar lib
        is_chrome_branded=true # h264
        

        rtc_include_pulse_audio=${PulseAudio_FOUND_STR} # false by default

        rtc_include_internal_audio_device=${WEBRTC_INCLUDE_DEFAULT_AUDIO_STR}
                                               # true = will use a default implementation
                                               # if not provided in create peer function
                                               # modules/audio_device/g3doc/audio_device_module.md
        
        rtc_use_x11=${X11_FOUND_STR}
        
        is_clang=true # use "goog clang" to compile
        # use_lld=true # default false, use lld linker
        # rtc_use_pipewire=false # needs use_sysroot
    )

convert_list_to_string("${GN_ARGS}" GN_ARGS_STR)

find_program(GN_EXECUTABLE gn)

if(NOT GN_EXECUTABLE)
    message(FATAL_ERROR "GN not found. Add it to the path, e.g. export PATH=$PATH:`realpath ../depot_tools`")
endif()

message(STATUS "Building WebRTC with the following arguments: ${GN_ARGS_STR}")

if (NOT EXISTS ${LIBWEBRTC_WEBRTC_BUILD_DIR})
    file(MAKE_DIRECTORY ${LIBWEBRTC_WEBRTC_BUILD_DIR})
endif()

execute_proc_info(
    COMMAND ${GN_EXECUTABLE} gen ${LIBWEBRTC_WEBRTC_BUILD_DIR} --args=${GN_ARGS_STR}
    WORKING_DIRECTORY ${LIBWEBRTC_WEBRTC_BUILD_DIR}
)

# ------------------------------------------------------ #
#                         PATCHES                        #
# ------------------------------------------------------ #
function(patch_file SOURCE_FILE SEARCH_STRING PATCH_STRING DESTINATION_FILE PATCH_NAME)
    # set(STAMP_FILE ${SOURCE_FILE}.patched)

    # IF (EXISTS ${STAMP_FILE})
    #     message(STATUS "Patch ${PATCH_NAME} already applied to ${DESTINATION_FILE}")
    #     return()
    # endif()

    if (NOT EXISTS ${SOURCE_FILE})
        message(FATAL_ERROR "File not found: ${OG_CHECKS_H_PATH}")
    endif()

    file(READ ${SOURCE_FILE} FILE_CONTENT)

    string(FIND "${FILE_CONTENT}" "${SEARCH_STRING}" SEARCH_STRING_INDEX)

    IF (${SEARCH_STRING_INDEX} EQUAL -1)
        message(FATAL_ERROR "String not found: ${SEARCH_STRING}"
                            "\nIn SOURCE_FILE: ${SOURCE_FILE}"
                            "\nYou can disable the patch \"${PATCH_NAME}\" if not needed.")
    endif()

    string(REPLACE "${SEARCH_STRING}" "${PATCH_STRING}" MODIFIED_CONTENT "${FILE_CONTENT}")   
    file(WRITE ${DESTINATION_FILE} "${MODIFIED_CONTENT}")

    # file(WRITE ${STAMP_FILE} "")
    message(STATUS "Patch ${PATCH_NAME} applied to ${DESTINATION_FILE}")
endfunction()

set(FILES_TO_PATCH ${LIBWEBRTC_WEBRTC_DIR}/rtc_base/checks.h)

# restore files from backup if they exist
foreach(file ${FILES_TO_PATCH})
    set(BACKUP_FN ${file}.bak)
    if (EXISTS ${BACKUP_FN})
        file(READ ${BACKUP_FN} FILE_CONTENT)
        file(WRITE ${file} "${FILE_CONTENT}")
        message(STATUS "Restored ${file} from backup")
    else()
        file(READ ${file} FILE_CONTENT)
        file(WRITE ${BACKUP_FN} "${FILE_CONTENT}")
        message(STATUS "Created backup of ${file}")
    endif()
endforeach()

# --- PATCH 1 - Projects can have different build types -- #
#
#   Context:    For example, when this project is compiled as debug,
#               RTC_DCHECK_IS_ON will be 0 (false) because of:
#                   #if !defined(NDEBUG) || defined(DCHECK_ALWAYS_ON)
#                   #define RTC_DCHECK_IS_ON 1
#               but, if the parent is compiled with Release,
#               RTC_DCHECK_IS_ON will be 1 (true), which will lead the
#               code to use undefined functions from webrtc.
#
#   Solution:   Patch checks.h so we can change the value of RTC_DCHECK_IS_ON
#               based on some flag. Or just set it to 1 or 0 depending on the
#               build type.
set(OG_CHECKS_H_PATH ${LIBWEBRTC_WEBRTC_DIR}/rtc_base/checks.h)

if (${WEBRTC_BUILD_TYPE} MATCHES "Debug")
    set(P1_REPLACE_STR "#if 1")
else()
    set(P1_REPLACE_STR "#if 0")
endif()

patch_file(
    ${OG_CHECKS_H_PATH} 
    "#if !defined(NDEBUG) || defined(DCHECK_ALWAYS_ON)" 
    ${P1_REPLACE_STR}
    ${OG_CHECKS_H_PATH}
    "1 - RTC_DCHECK_IS_ON"
)

# -------------------------------------------------------- #
#                       BUILD WEBRTC                       #
# -------------------------------------------------------- #
if (WIN32)
    set(SHELLCOMMAND cmd /c )
endif ()

# check for libwebrtc.a or libwebrtc.lib
if (NOT EXISTS ${LIBWEBRTC_WEBRTC_BUILD_BIN_DIR}/${CMAKE_STATIC_LIBRARY_PREFIX}webrtc${CMAKE_STATIC_LIBRARY_SUFFIX})
    execute_proc_info(
        COMMAND ${SHELLCOMMAND} ninja -l10 webrtc rtc_json jsoncpp builtin_video_decoder_factory builtin_video_encoder_factory peerconnection p2p_server_utils task_queue  default_task_queue_factory
        WORKING_DIRECTORY ${LIBWEBRTC_WEBRTC_BUILD_DIR}
    )
endif()

# -------------------------------------------------------- #
#                          INSTALL                         #
# -------------------------------------------------------- #

# ----------------- INSTALL INCLUDE FILES ---------------- #
file(GLOB rtc_subdirectories LIST_DIRECTORIES true "${LIBWEBRTC_WEBRTC_DIR}/*")
list(FILTER rtc_subdirectories EXCLUDE REGEX "\/(third_party|testing|g3doc|tools|\.git|build|buildtools)$")

foreach(dir ${rtc_subdirectories})
    if(IS_DIRECTORY ${dir})
        get_filename_component(last_part "${dir}" NAME) # TODO cmake_path if cmake >= 3.20
        install_symlink(${dir} ${WEBRTC_INCLUDE_INSTALL_DIR}/${last_part})
    else()
        continue()
    endif()
endforeach()

install_symlink(${LIBWEBRTC_WEBRTC_DIR}/third_party/abseil-cpp/absl ${WEBRTC_INCLUDE_INSTALL_DIR}/absl)

install_symlink(${LIBWEBRTC_WEBRTC_DIR}/third_party/jsoncpp/source/include/json ${WEBRTC_INCLUDE_INSTALL_DIR}/json)
# NOTE: generated files cannot be installed in include/webrtc/json, because it's a link!
install(DIRECTORY ${LIBWEBRTC_WEBRTC_DIR}/third_party/jsoncpp/generated/
        DESTINATION ${WEBRTC_INCLUDE_INSTALL_DIR}/json_generated
        FILES_MATCHING 
            PATTERN "*.h"
            PATTERN "*.hpp")

# file(COPY
#         ${LIBWEBRTC_WEBRTC_DIR}/third_party/libyuv/include/libyuv.h
#         DESTINATION ${WEBRTC_INCLUDE_INSTALL_DIR}/libyuv)
install(DIRECTORY ${LIBWEBRTC_WEBRTC_DIR}/third_party/libyuv/include/libyuv/
        DESTINATION ${WEBRTC_INCLUDE_INSTALL_DIR}/libyuv
        FILES_MATCHING 
            PATTERN "*.h"
            PATTERN "*.hpp")

# --------------------- INSTALL LIBS --------------------- #
file(GLOB_RECURSE LIBWEBRTC ${LIBWEBRTC_WEBRTC_BUILD_DIR}/${CMAKE_STATIC_LIBRARY_PREFIX}*${CMAKE_STATIC_LIBRARY_SUFFIX})

file(GLOB_RECURSE webrtc_jsoncpp_objs ${LIBWEBRTC_WEBRTC_BUILD_BIN_DIR}/third_party/jsoncpp/jsoncpp/*${CMAKE_C_OUTPUT_EXTENSION})
file(GLOB_RECURSE webrtc_json_utils_objs ${LIBWEBRTC_WEBRTC_BUILD_BIN_DIR}/rtc_base/rtc_json/*${CMAKE_C_OUTPUT_EXTENSION})
file(GLOB_RECURSE rtc_sequen_checker_objs ${LIBWEBRTC_WEBRTC_BUILD_BIN_DIR}/rtc_base/synchronization/sequence_checker_internal/*${CMAKE_C_OUTPUT_EXTENSION})
# NOTE: add any third_party used by me to webrtc_extra

add_library(webrtc_extra STATIC ${webrtc_jsoncpp_objs} ${webrtc_json_utils_objs} ${rtc_sequen_checker_objs})
set_target_properties(webrtc_extra PROPERTIES LINKER_LANGUAGE CXX)

if (${GTK3_FOUND})
    set(USE_X11 ON)
    message(STATUS "PLEASE LINK TO: X11 Xext Xdamage Xfixes Xcomposite Xrandr Xtst")
endif()

# ------------------- CONFIGURE FILE ------------------- #
# - webrtc H264 impl is NOT supported in Android and iOS.
# - H264 can be disabled by proprietary_codecs=false
# None of this two cases are true, so we can enable H264.
set(WEBRTC_USE_H264 ON)

if (WIN32)
    set(WEBRTC_WIN ON)
elseif(UNIX)
    set(WEBRTC_POSIX ON)

    if (CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set(WEBRTC_MAC ON)
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(WEBRTC_LINUX ON)
    else()
        message(FATAL_ERROR "This cmake script only supports Windows, Linux and macOS. "
                            "Your operative system name is ${CMAKE_SYSTEM_NAME}.")
    endif()
else()
    message(FATAL_ERROR "This cmake script only supports Windows, Linux and macOS. "
                        "Your operative system name is ${CMAKE_SYSTEM_NAME}.")
endif()

configure_file(
    ${CMAKE_CURRENT_LIST_DIR}/webrtc_config.h.in
    ${WEBRTC_INCLUDE_INSTALL_DIR}/webrtc_config.h
    @ONLY
)

install(TARGETS webrtc_extra
    LIBRARY DESTINATION ${WEBRTC_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${WEBRTC_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${WEBRTC_INSTALL_LIBDIR}
)

file(COPY ${LIBWEBRTC_WEBRTC_BUILD_BIN_DIR}/${CMAKE_STATIC_LIBRARY_PREFIX}webrtc${CMAKE_STATIC_LIBRARY_SUFFIX}
    DESTINATION ${WEBRTC_INSTALL_LIBDIR})


# -------------------------------------------------------- #
#                     Broadcast webrtc                     #
# -------------------------------------------------------- #
# install broadrtc headers
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/broadcast/
        DESTINATION ${WEBRTC_INCLUDE_INSTALL_DIR}/broadrtc
        FILES_MATCHING 
            PATTERN "*.h"
            PATTERN "*.hpp")