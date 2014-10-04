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

set(project_SOURCES main.cpp pclobjectextractor.cpp)
set(project_HEADERS pclobjectextractor.h)
set(project_FORMS pclobjectextractor.ui)
set(project_RESOURCES icon.qrc)
set(VTK_LIBRARIES vtkRendering vtkGraphics vtkHybrid QVTK)

QT4_WRAP_UI(project_FORMS_HEADERS ${project_FORMS})
QT4_WRAP_CPP(project_HEADERS_MOC ${project_HEADERS})
QT4_ADD_RESOURCES(RESOURCE_ADDED ${project_RESOURCES})

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

ADD_EXECUTABLE(PCLObjectExtractor ${project_SOURCES}
                                  ${project_FORMS_HEADERS}
                                  ${project_HEADERS_MOC}
                                  ${project_RESOURCES})

TARGET_LINK_LIBRARIES(PCLObjectExtractor ${QT_LIBRARIES}
                                         ${PCL_LIBRARIES}
                                         ${VTK_LIBRARIES})
