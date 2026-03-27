# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# 须由 ESP-IDF 的 export 脚本或 idf.py 设置；否则 include 会变成 /tools/cmake/project.cmake 并报错
if("$ENV{IDF_PATH}" STREQUAL "")
    message(FATAL_ERROR
        "IDF_PATH 未设置。请在本机任选其一：\n"
        "  1) 使用「ESP-IDF PowerShell / CMD」或 VS Code/Cursor 的 ESP-IDF 扩展，再执行 idf.py build；\n"
        "  2) 在终端先运行 ESP-IDF 目录下的 export.ps1（PowerShell）或 export.bat（CMD）；\n"
        "  3) 将系统/用户环境变量 IDF_PATH 设为 esp-idf 根目录（例如 D:/Espressif/frameworks/esp-idf-v5.5.3）。\n"
        "勿在未设置 IDF_PATH 时单独运行 cmake 配置本工程。")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(inmp441_try)