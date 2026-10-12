zephyr_library_amend()

if(CONFIG_INPUT_AZOTEQ_IQS915X)
  set(init_header ${CMAKE_CURRENT_SOURCE_DIR}/IQS9150_init.h)
  set(init_generator ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/generate_init_array.py)
  set(init_array ${CMAKE_CURRENT_BINARY_DIR}/iqs915x_init_array.h)
  add_custom_command(
    OUTPUT ${init_array}
    COMMAND ${PYTHON_EXECUTABLE} ${init_generator} ${init_header} --output ${init_array}
    DEPENDS ${init_header} ${init_generator}
    VERBATIM
  )
  add_custom_target(iqs915x_init_array DEPENDS ${init_array})
  add_dependencies(${ZEPHYR_CURRENT_LIBRARY} iqs915x_init_array)
  zephyr_library_include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
endif()

zephyr_library_sources_ifdef(CONFIG_INPUT_AZOTEQ_IQS915X iqs915x.c)
zephyr_library_sources_ifdef(CONFIG_INPUT_AZOTEQ_IQS915X iqs915x_power.c)
zephyr_library_sources_ifdef(CONFIG_INPUT_AZOTEQ_IQS915X iqs915x_gestures.c)
