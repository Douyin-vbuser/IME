cmake_minimum_required(VERSION 3.10)
project(ime_controller)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找JNI
find_package(JNI REQUIRED)
include_directories(${JNI_INCLUDE_DIRS})
include_directories(include)

# 根据操作系统选择源文件和库
if(WIN32)
    set(SOURCES src/windows/ime_controller.cpp)
    set(PLATFORM_LIBS Imm32)
elseif(APPLE)
    set(SOURCES src/macos/ime_controller.mm)
    set(PLATFORM_LIBS
        "-framework Foundation"
        "-framework Carbon"
    )
    # 为MacOS设置特殊编译选项
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ObjC++")
elseif(UNIX)
    set(SOURCES src/linux/ime_controller.cpp)
    # 查找X11
    find_package(X11 REQUIRED)
    include_directories(${X11_INCLUDE_DIR})
    set(PLATFORM_LIBS ${X11_LIBRARIES} ${X11_Xkb_LIB})
endif()

# 创建共享库
add_library(ime_controller SHARED ${SOURCES})

# 链接必要的库
target_link_libraries(ime_controller PRIVATE ${JNI_LIBRARIES} ${PLATFORM_LIBS})

# 设置输出名称
set_target_properties(ime_controller PROPERTIES
    PREFIX ""
    OUTPUT_NAME "ime_controller"
)

# 设置平台特定的输出后缀
if(WIN32)
    set_target_properties(ime_controller PROPERTIES SUFFIX ".dll")
elseif(APPLE)
    set_target_properties(ime_controller PROPERTIES SUFFIX ".dylib")
else()
    set_target_properties(ime_controller PROPERTIES SUFFIX ".so")
endif()