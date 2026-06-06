zephyr_library_amend()

set(IQS915X_INIT_DATA_INPUT ${CMAKE_CURRENT_LIST_DIR}/IQS9150_init.h)
set(IQS915X_INIT_DATA_CONVERTER ${CMAKE_CURRENT_LIST_DIR}/../../scripts/convert_init_header.py)
set(IQS915X_INIT_DATA_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(IQS915X_INIT_DATA_ARRAY ${IQS915X_INIT_DATA_GEN_DIR}/iqs915x_init_data_bretagne_array.h)

if(NOT DEFINED PYTHON_EXECUTABLE)
	find_package(Python3 REQUIRED COMPONENTS Interpreter)
	set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
endif()

add_custom_command(
	OUTPUT ${IQS915X_INIT_DATA_ARRAY}
	COMMAND ${CMAKE_COMMAND} -E make_directory ${IQS915X_INIT_DATA_GEN_DIR}
	COMMAND ${PYTHON_EXECUTABLE} ${IQS915X_INIT_DATA_CONVERTER} ${IQS915X_INIT_DATA_INPUT} --output ${IQS915X_INIT_DATA_ARRAY}
	DEPENDS ${IQS915X_INIT_DATA_INPUT} ${IQS915X_INIT_DATA_CONVERTER}
	COMMENT "Generating IQS915x init-data array header from IQS9150_init.h"
	VERBATIM
)

add_custom_target(iqs915x_init_data_header_gen DEPENDS ${IQS915X_INIT_DATA_ARRAY})

zephyr_include_directories(${IQS915X_INIT_DATA_GEN_DIR})
zephyr_library_include_directories(${IQS915X_INIT_DATA_GEN_DIR})

add_dependencies(${ZEPHYR_CURRENT_LIBRARY} iqs915x_init_data_header_gen)

set_source_files_properties(iqs915x.c PROPERTIES OBJECT_DEPENDS ${IQS915X_INIT_DATA_ARRAY})

zephyr_library_sources_ifdef(CONFIG_INPUT_AZOTEQ_IQS915X iqs915x.c)
