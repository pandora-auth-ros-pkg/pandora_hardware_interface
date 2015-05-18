


add_definitions(-D__LINUX__)


file(STRINGS "cmake/VERSION" LIBPIXY_VERSION)
add_definitions(-D__LIBPIXY_VERSION__="${LIBPIXY_VERSION}")


add_library (pixyusb STATIC src/libpixyusb/chirpreceiver.cpp
                            src/libpixyusb/pixyinterpreter.cpp
                            src/libpixyusb/pixy.cpp
                            src/libpixyusb/usblink.cpp
                            src/common/chirp.cpp)



target_link_libraries(pixyusb
	${LIBUSB_1_LIBRARIES}
	
)






set (THIS_LIB libpixyusb)






