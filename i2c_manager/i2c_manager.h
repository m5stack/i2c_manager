// 修改 i2c_manager.h 文件中的宏定义部分

#ifndef _I2C_MANAGER_H
#define _I2C_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

// 使用新的I2C驱动API
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// 保持原有的宏定义以兼容
#define CONCATX(A, B) A ## B
#define CONCAT(A, B) CONCATX(A, B)
#define STR_LITERAL(s) # s
#define STR_EXPAND(s) STR_LITERAL(s)
#define STR_QUOTE(s) STR_EXPAND(STR_EXPAND(s))

#ifdef I2C_OEM
    #define I2C_NAME_PREFIX CONCAT(I2C_OEM, _i2c)
#else
    #define I2C_NAME_PREFIX i2c_manager
#endif
#define I2C_TAG STR_EXPAND(I2C_NAME_PREFIX)

// 修复: 正确的宏定义方式
#define I2C_FN(s) CONCAT(I2C_NAME_PREFIX, s)

// 新的I2C管理器配置标志
#define I2C_ADDR_10 ( 1 << 15 )
#define I2C_REG_16  ( 1 << 31 )
#define I2C_NO_REG  ( 1 << 30 )

// I2C管理器结构体
typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handles[128]; // 支持128个不同地址的设备
    SemaphoreHandle_t mutex;
    bool initialized;
    uint32_t scl_io_num;
    uint32_t sda_io_num;
    uint32_t clk_speed;
} i2c_manager_t;

// 修复: 正确声明函数原型
esp_err_t I2C_FN(_init)(i2c_port_t port);
esp_err_t I2C_FN(_read)(i2c_port_t port, uint16_t addr, uint32_t reg, uint8_t *buffer, uint16_t size);
esp_err_t I2C_FN(_write)(i2c_port_t port, uint16_t addr, uint32_t reg, const uint8_t *buffer, uint16_t size);
esp_err_t I2C_FN(_close)(i2c_port_t port);
esp_err_t I2C_FN(_lock)(i2c_port_t port);
esp_err_t I2C_FN(_unlock)(i2c_port_t port);
esp_err_t I2C_FN(_force_unlock)(i2c_port_t port);

// 为外部调用提供标准函数名的宏别名
#ifndef I2C_OEM
#define i2c_manager_init(port)         I2C_FN(_init)(port)
#define i2c_manager_read(port, addr, reg, buffer, size) \
    I2C_FN(_read)(port, addr, reg, buffer, size)
#define i2c_manager_write(port, addr, reg, buffer, size) \
    I2C_FN(_write)(port, addr, reg, buffer, size)
#define i2c_manager_close(port)        I2C_FN(_close)(port)
#define i2c_manager_lock(port)         I2C_FN(_lock)(port)
#define i2c_manager_unlock(port)       I2C_FN(_unlock)(port)
#define i2c_manager_force_unlock(port) I2C_FN(_force_unlock)(port)
#endif

// 额外的辅助函数
i2c_master_bus_handle_t i2c_manager_get_bus_handle(i2c_port_t port);
esp_err_t i2c_manager_get_device_handle(i2c_port_t port, uint16_t addr, i2c_master_dev_handle_t *dev_handle);
esp_err_t i2c_manager_probe_device(i2c_port_t port, uint16_t addr);
esp_err_t i2c_manager_scan_bus(i2c_port_t port, uint8_t *found_devices, size_t max_devices, size_t *num_found);

#ifdef __cplusplus
}
#endif

#endif /* _I2C_MANAGER_H */