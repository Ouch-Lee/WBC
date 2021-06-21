
include_directories( ${PROJECT_SOURCE_DIR}/lib/include/ )
aux_source_directory( ${PROJECT_SOURCE_DIR}/lib/src/ WBC_LIB)

add_library( wholeBodyCtrl STATIC ${WBC_LIB} )

install(TARGETS wholeBodyCtrl ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/wholeBodyCtrl)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/lib/include/ DESTINATION ${PROJECT_SOURCE_DIR}/build/wholeBodyCtrl)
