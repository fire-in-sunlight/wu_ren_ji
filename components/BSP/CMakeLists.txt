# 注册当前组件（默认组件名为 BSP）
idf_component_register(
    SRCS 
        "motor.cpp"
        "mpu6050.cpp"
    INCLUDE_DIRS 
        "."
    REQUIRES 
        driver
        cxx
    PRIV_REQUIRES
        freertos
)