# 让 CMake 自动找到所有的 C 和 C++ 源码文件
file(GLOB_RECURSE SOURCES "*.cpp" "*.c" "*.cc")

# 注册为 ESP-IDF 组件，并声明依赖
idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "." "edge-impulse-sdk" "model-parameters" "tflite-model"
    REQUIRES esp_timer freertos esp_system cxx espressif__esp-dsp
)

# 【终极奥义】：加入 "-w" (小写w)，暴力关闭所有第三方代码的警告！
# 加入 "-Wno-maybe-uninitialized" 专门针对 GCC 14 的神经质检查
target_compile_options(${COMPONENT_LIB} PRIVATE 
    "-Os" 
    "-w" 
    "-Wno-error" 
    "-Wno-maybe-uninitialized"
)
target_compile_definitions(${COMPONENT_LIB} PRIVATE "-DEI_CLASSIFIER_ALLOCATION_METHOD=1")