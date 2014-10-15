cmake_minimum_required(VERSION 2.6)

project(PCLObjectExtractor)
 
find_package(Qt REQUIRED)
find_package(VTK REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(PkgConfig REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCES src/main.cpp src/qt/pclobjectextractor.cpp)
set(HEADERS src/qt/pclobjectextractor.h)
set(FORMS src/qt/pclobjectextractor.ui)
set(VTK_LIBRARIES vtkRendering vtkGraphics vtkHybrid QVTK)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin/)
set(CMAKE_BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin/)

QT4_WRAP_UI(FORMS_HEADERS ${FORMS})
QT4_WRAP_CPP(HEADERS_MOC ${HEADERS})

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

add_executable(PCLObjectExtractor ${SOURCES}
                                  ${FORMS_HEADERS}
                                  ${HEADERS_MOC})

target_link_libraries(PCLObjectExtractor ${QT_LIBRARIES}
                                         ${PCL_LIBRARIES}
                                         ${VTK_LIBRARIES})

install(TARGETS PCLObjectExtractor DESTINATION /Applications/)
install(FILES icon.png DESTINATION /Applications/)

set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_VERSION_MAJOR "1")
set(CPACK_PACKAGE_VERSION_MINOR "0")
set(CPACK_PACKAGE_VERSION_PATCH "0")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libpcl-all-dev")
set(CPACK_PACKAGE_DESCRIPTION "Configure UExhibitor and launch missions")
set(CPACK_PACKAGE_CONTACT "Andrew Watson watsontandrew@gmail.com")
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/Debian/postinst")

include(CPack)
