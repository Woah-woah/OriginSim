
add_library(${PROJECT_NAME}.lio
        lio_utils.cpp
        lidarodom.cpp
        )

target_link_libraries(${PROJECT_NAME}.lio
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.algo
        ${third_party_libs}
        )

