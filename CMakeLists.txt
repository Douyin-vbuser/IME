cmake_minimum_required(VERSION 3.10)
project(dda)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找JNI
find_package(JNI REQUIRED)
include_directories(${JNI_INCLUDE_DIRS})
include_directories(include)

set(SOURCES
        src/dda.cpp
)

# 创建共享库
add_library(dda SHARED ${SOURCES})

# 链接必要的库
target_link_libraries(dda PRIVATE ${JNI_LIBRARIES} ${PLATFORM_LIBS})

# 设置输出名称
set_target_properties(dda PROPERTIES
    PREFIX ""
    OUTPUT_NAME "dda"
)

# 设置平台特定的输出后缀
if(WIN32)
    set_target_properties(dda PROPERTIES SUFFIX ".dll")
elseif(APPLE)
    set_target_properties(dda PROPERTIES SUFFIX ".dylib")
else()
    set_target_properties(dda PROPERTIES SUFFIX ".so")
endif()

set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

# 添加更多优化选项
if(MSVC)
    target_compile_options(dda PRIVATE
            /O2 /Oi /Ot /Oy /GL /Qpar /fp:fast
            /arch:AVX2
    )
else()
    target_compile_options(dda PRIVATE
            -O3 -march=native -ffast-math
            -ftree-vectorize -fomit-frame-pointer
            -funroll-loops -fno-stack-protector
    )
endif()