# ESP32-S3 + MAX30102 + SSD1306 OLED 心率血氧例程
# 由 STM32F103C8T6 例程移植（ESP-IDF 原生框架，建议 v5.1 及以上）
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(max30102_esp32s3)
