BLE_CUSTOM_SERVICE
===========================
蓝牙-自定义服务

环境依赖
###########
1. zephyr v3.5.99
2. nrf connect sdk v2.6.1

部署步骤
###########
config
######
1. config文件: prj.conf
2. overlay文件: app.overlay

源文件
######
1. ./src/main.c
2. ./src/ble.c
3. ./src/custom_srv.c

相关库
######
1.  v2.6.1/nrf/lib/dk_buttons_and_leds/dk_buttons_and_leds.c   按键及LED

