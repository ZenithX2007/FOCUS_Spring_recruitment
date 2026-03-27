@echo off
echo ========================================
echo ESP-IDF 项目编译脚本
echo ========================================
echo.

REM 设置 ESP-IDF 路径
set IDF_PATH=E:\IDEA_PROJECT\esp-idf\v5.4.3\esp-idf
echo 已设置 IDF_PATH=%IDF_PATH%
echo.

REM 切换到项目目录
cd /d %~dp0
echo 当前目录：%CD%
echo.

REM 检查是否已设置目标
if not exist "sdkconfig" (
    echo [步骤 1/3] 设置目标芯片为 ESP32-S3...
    call python "%IDF_PATH%\tools\idf.py" set-target esp32s3
    if errorlevel 1 (
        echo 错误：设置目标失败！
        pause
        exit /b 1
    )
) else (
    echo [跳过] sdkconfig 已存在
)
echo.

REM 编译项目
echo [步骤 2/3] 开始编译项目...
call python "%IDF_PATH%\tools\idf.py" build
if errorlevel 1 (
    echo 错误：编译失败！
    pause
    exit /b 1
)
echo.

REM 完成
echo ========================================
echo ✓ 编译成功！
echo 固件位置：build\inmp441_try.bin
echo ========================================
echo.
pause
