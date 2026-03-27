idf_component_register(
    SRCS "myuart.c"               # 要编译的源文件
    INCLUDE_DIRS "."               # 头文件暴露的路径（当前目录）
    REQUIRES driver                # 依赖的官方组件（uart依赖driver）
)

# 可选：编译优化（和你原有配置一致）
component_compile_options(-ffast-math -O3 -Wno-error=format -Wno-format)