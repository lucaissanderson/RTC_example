idf_component_register(SRCS "RTC_example.cpp" "DS3231_RTC.cpp"
                    INCLUDE_DIRS "."
		            REQUIRES driver freertos)

set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 11)  # Use C++11 standard
