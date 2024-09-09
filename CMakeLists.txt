cmake_minimum_required( VERSION 3.23 )
project( M5AtomMotionSolarTracker )

set( CMAKE_CXX_STANDARD 14 )

include_directories( .
		"C:/Users/Adam/AppData/Local/Arduino15/packages/m5stack/hardware/esp32/2.1.1/cores/esp32/"
		"C:/Users/adam.howell/AppData/Local/Arduino15/packages/m5stack/hardware/esp32/2.1.1/cores/esp32/"
		"C:/Users/Adam/AppData/Local/Arduino15/packages/m5stack/hardware/esp32/2.1.1/libraries/Wire/src/"
		"C:/Users/adam.howell/AppData/Local/Arduino15/packages/m5stack/hardware/esp32/2.1.1/libraries/Wire/src/"
		"C:/Users/Adam/AppData/Local/Arduino15/packages/m5stack/hardware/esp32/2.1.1/tools/sdk/esp32/include/freertos/include/"
		"C:/Users/adam.howell/AppData/Local/Arduino15/packages/m5stack/hardware/esp32/2.1.1/tools/sdk/esp32/include/freertos/include/"
		"C:/Users/Adam/AppData/Local/Arduino15/packages/m5stack/hardware/esp32/2.1.1/tools/sdk/esp32/include/soc/esp32/include/"
		"C:/Users/adam.howell/AppData/Local/Arduino15/packages/m5stack/hardware/esp32/2.1.1/tools/sdk/esp32/include/soc/esp32/include/"
		"C:/Users/Adam/Documents/Arduino/libraries/M5Atom/src/"
		"C:/Users/adam.howell/Documents/Arduino/libraries/M5Atom/src/"
		"C:/Users/Adam/Documents/Arduino/libraries/M5-DLight/src/"
		"C:/Users/adam.howell/Documents/Arduino/libraries/M5-DLight/src/"
		"C:/Users/Adam Howell/Documents/Arduino/libraries/M5Atom/src/"
		"C:/Users/Adam Howell/Documents/Arduino/libraries/M5-DLight/src/"
		"C:/Users/Adam Howell/AppData/Local/Arduino15/packages/m5stack/hardware/esp32/2.1.1/cores/esp32/"
		"C:/Users/Adam Howell/AppData/Local/Arduino15/packages/m5stack/hardware/esp32/2.1.1/libraries/Wire/src/"
)

add_executable( M5AtomMotionSolarTracker
		M5AtomMotionTest.ino
		AtomMotion.h
		AtomMotion.cpp
)

set_target_properties( M5AtomMotionSolarTracker PROPERTIES LINKER_LANGUAGE CXX )
