set(OSCORE_SRCS
        "main.cpp"
        "config.h"
        "madgwick.cpp"
        "mag.cpp"
        "imu.cpp"
        "imu_heater.cpp"
        "fw_update.cpp"
        "encoder.cpp"
        "sbus.cpp"
        "servo.cpp"
        "led.cpp"
        "buzzer.cpp"
        "alert_buzzer.cpp"
        "melody_player.cpp"
        "host_link.cpp"
        "safety_event.cpp"
        "status_indicator.cpp"
        "pid.cpp"
        "nvs_params.cpp"
        "serial_port.cpp"
        "serial_cmd.cpp"
        "serial_frames.cpp"
        "serial_tx.cpp"
)

if(CONFIG_OSCORE_USB_BACKEND_TINYUSB)
    list(APPEND OSCORE_SRCS "usb_descriptors.c")
endif()

string(TIMESTAMP OSCORE_BUILD_TIME "%Y-%m-%d %H:%M")
set(OSCORE_PRODUCT_NAME "Neoracer V1")
set(OSCORE_FIRMWARE_NAME "osrcore")
set(OSCORE_HARDWARE_NAME "OSCORE_NEO_ESP32S3_RevA")
if(DEFINED ENV{OSCORE_RELEASE_DATE})
    set(OSCORE_RELEASE_DATE "$ENV{OSCORE_RELEASE_DATE}")
else()
    set(OSCORE_RELEASE_DATE "20260628")
endif()
execute_process(
    COMMAND git rev-parse --short=7 HEAD
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/.."
    OUTPUT_VARIABLE OSCORE_GIT_SHA
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT OSCORE_GIT_SHA)
    set(OSCORE_GIT_SHA "5caddcf")
endif()
execute_process(
    COMMAND git status --porcelain --untracked-files=no
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/.."
    OUTPUT_VARIABLE OSCORE_GIT_STATUS
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
set(OSCORE_DIRTY_SUFFIX "")
set(OSCORE_DIRTY_FLAG "No")
if(OSCORE_GIT_STATUS)
    set(OSCORE_DIRTY_SUFFIX "-d")
    set(OSCORE_DIRTY_FLAG "Yes")
endif()
set(PROJECT_VER "NEORACER_V1-${OSCORE_RELEASE_DATE}-g${OSCORE_GIT_SHA}${OSCORE_DIRTY_SUFFIX}")
configure_file(
    "${CMAKE_CURRENT_LIST_DIR}/version.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/version.h"
    @ONLY
)

idf_component_register(
    SRCS ${OSCORE_SRCS}
    INCLUDE_DIRS "." "${CMAKE_CURRENT_BINARY_DIR}/generated"
)
