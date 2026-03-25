cmake_minimum_required(VERSION 3.20)
project(Reconstruct2D LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# 第三方库路径配置
# ============================================================
set(GLEW_ROOT "E:/MMT/PreMesysa/3rdparty/glew" CACHE PATH "GLEW root directory")

# ============================================================
# OpenGL (系统自带)
# ============================================================
find_package(OpenGL REQUIRED)

# ============================================================
# GLEW - 使用指定路径
# ============================================================
set(GLEW_INCLUDE_DIR "${GLEW_ROOT}/include")
set(GLEW_LIBRARY "${GLEW_ROOT}/lib/glew32.lib")
set(GLEW_DLL "${GLEW_ROOT}/bin/Release/x64/glew32.dll")

if(NOT EXISTS "${GLEW_INCLUDE_DIR}/GL/glew.h")
    message(FATAL_ERROR "GLEW header not found at: ${GLEW_INCLUDE_DIR}/GL/glew.h")
endif()
if(NOT EXISTS "${GLEW_LIBRARY}")
    message(FATAL_ERROR "GLEW library not found at: ${GLEW_LIBRARY}")
endif()

add_library(GLEW::GLEW SHARED IMPORTED)
set_target_properties(GLEW::GLEW PROPERTIES
    IMPORTED_IMPLIB "${GLEW_LIBRARY}"
    IMPORTED_LOCATION "${GLEW_DLL}"
    INTERFACE_INCLUDE_DIRECTORIES "${GLEW_INCLUDE_DIR}"
)

# ============================================================
# FreeGLUT - 使用本地 3rdparty
# ============================================================
set(FREEGLUT_ROOT "${CMAKE_SOURCE_DIR}/3rdparty/freeglut")
set(FREEGLUT_INCLUDE_DIR "${FREEGLUT_ROOT}/include")
set(FREEGLUT_LIBRARY "${FREEGLUT_ROOT}/lib/x64/freeglut.lib")
set(FREEGLUT_DLL "${FREEGLUT_ROOT}/bin/x64/freeglut.dll")

# ============================================================
# ANN - 直接编译源码
# ============================================================
set(ANN_ROOT "${CMAKE_SOURCE_DIR}/3rdparty/ann_1.1.2")
set(ANN_SOURCES
    ${ANN_ROOT}/src/ANN.cpp
    ${ANN_ROOT}/src/bd_fix_rad_search.cpp
    ${ANN_ROOT}/src/bd_pr_search.cpp
    ${ANN_ROOT}/src/bd_search.cpp
    ${ANN_ROOT}/src/bd_tree.cpp
    ${ANN_ROOT}/src/brute.cpp
    ${ANN_ROOT}/src/kd_dump.cpp
    ${ANN_ROOT}/src/kd_fix_rad_search.cpp
    ${ANN_ROOT}/src/kd_pr_search.cpp
    ${ANN_ROOT}/src/kd_search.cpp
    ${ANN_ROOT}/src/kd_split.cpp
    ${ANN_ROOT}/src/kd_tree.cpp
    ${ANN_ROOT}/src/kd_util.cpp
    ${ANN_ROOT}/src/perf.cpp
)

# ============================================================
# 项目源文件
# ============================================================
set(SOURCES
    src/TestReconstruct2D.cpp
    src/Reconstruct2D.cpp
    src/circle.cpp
    src/PrecTimer.cpp
)

set(HEADERS
    src/Reconstruct2D.h
    src/Point.h
    src/circle.h
    src/PrecTimer.h
)

# ANN 编译为静态库，需要定义 DLL_EXPORTS 避免 dllimport 错误
set_source_files_properties(${ANN_SOURCES} PROPERTIES
    COMPILE_DEFINITIONS "DLL_EXPORTS"
)

add_executable(TestReconstruct2D ${SOURCES} ${HEADERS} ${ANN_SOURCES})

# ============================================================
# Include 目录
# ============================================================
target_include_directories(TestReconstruct2D PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${GLEW_INCLUDE_DIR}
    ${FREEGLUT_INCLUDE_DIR}
    ${ANN_ROOT}/include
)

# ============================================================
# 链接库
# ============================================================
target_link_libraries(TestReconstruct2D PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLEW::GLEW
    ${FREEGLUT_LIBRARY}
    comdlg32
)

# ============================================================
# MSVC 特定设置
# ============================================================
if(MSVC)
    target_compile_definitions(TestReconstruct2D PRIVATE
        _CRT_SECURE_NO_WARNINGS
        NOMINMAX
        WIN32_LEAN_AND_MEAN
    )
    # 禁用常见的类型转换警告
    target_compile_options(TestReconstruct2D PRIVATE /W3 /wd4244 /wd4267 /wd4305 /wd4996)
endif()

# ============================================================
# 复制 data 目录和 DLL 到构建输出目录
# ============================================================
file(COPY ${CMAKE_SOURCE_DIR}/data DESTINATION ${CMAKE_BINARY_DIR})

# 构建后复制 GLEW DLL
if(EXISTS "${GLEW_DLL}")
    add_custom_command(TARGET TestReconstruct2D POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${GLEW_DLL}" $<TARGET_FILE_DIR:TestReconstruct2D>
    )
endif()

# 构建后复制 FreeGLUT DLL
if(EXISTS "${FREEGLUT_DLL}")
    add_custom_command(TARGET TestReconstruct2D POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${FREEGLUT_DLL}" $<TARGET_FILE_DIR:TestReconstruct2D>
    )
endif()
