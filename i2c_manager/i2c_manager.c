/*
 * I2C Manager for ESP-IDF 5.4.1+
 * Using new I2C Master Driver API
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#include "sdkconfig.h"
#include "i2c_manager.h"

static const char* TAG = I2C_TAG;

// I2C管理器实例
static i2c_manager_t i2c_managers[I2C_NUM_MAX] = {0};

// 配置定义
#ifdef CONFIG_I2C_MANAGER_0_ENABLED
    #define I2C_ZERO
    #ifndef CONFIG_I2C_MANAGER_0_SDA
        #define CONFIG_I2C_MANAGER_0_SDA 21
    #endif
    #ifndef CONFIG_I2C_MANAGER_0_SCL
        #define CONFIG_I2C_MANAGER_0_SCL 22
    #endif
    #ifndef CONFIG_I2C_MANAGER_0_FREQ_HZ
        #define CONFIG_I2C_MANAGER_0_FREQ_HZ 400000
    #endif
    #ifndef CONFIG_I2C_MANAGER_0_TIMEOUT
        #define CONFIG_I2C_MANAGER_0_TIMEOUT 20
    #endif
    #ifndef CONFIG_I2C_MANAGER_0_LOCK_TIMEOUT
        #define CONFIG_I2C_MANAGER_0_LOCK_TIMEOUT 50
    #endif
#endif

#ifdef CONFIG_I2C_MANAGER_1_ENABLED
    #define I2C_ONE
    #ifndef CONFIG_I2C_MANAGER_1_SDA
        #define CONFIG_I2C_MANAGER_1_SDA 18
    #endif
    #ifndef CONFIG_I2C_MANAGER_1_SCL
        #define CONFIG_I2C_MANAGER_1_SCL 19
    #endif
    #ifndef CONFIG_I2C_MANAGER_1_FREQ_HZ
        #define CONFIG_I2C_MANAGER_1_FREQ_HZ 400000
    #endif
    #ifndef CONFIG_I2C_MANAGER_1_TIMEOUT
        #define CONFIG_I2C_MANAGER_1_TIMEOUT 20
    #endif
    #ifndef CONFIG_I2C_MANAGER_1_LOCK_TIMEOUT
        #define CONFIG_I2C_MANAGER_1_LOCK_TIMEOUT 50
    #endif
#endif

// 超时定义
#ifdef I2C_ZERO
    #define I2C_MANAGER_0_TIMEOUT_TICKS (CONFIG_I2C_MANAGER_0_TIMEOUT / portTICK_PERIOD_MS)
    #define I2C_MANAGER_0_LOCK_TIMEOUT_TICKS (CONFIG_I2C_MANAGER_0_LOCK_TIMEOUT / portTICK_PERIOD_MS)
#endif

#ifdef I2C_ONE
    #define I2C_MANAGER_1_TIMEOUT_TICKS (CONFIG_I2C_MANAGER_1_TIMEOUT / portTICK_PERIOD_MS)
    #define I2C_MANAGER_1_LOCK_TIMEOUT_TICKS (CONFIG_I2C_MANAGER_1_LOCK_TIMEOUT / portTICK_PERIOD_MS)
#endif

// 错误处理宏
#define ERROR_PORT(port, fail) { \
    ESP_LOGE(TAG, "Invalid port or not configured for I2C Manager: %d", (int)port); \
    return fail; \
}

#if defined(I2C_ZERO) && defined(I2C_ONE)
    #define I2C_PORT_CHECK(port, fail) \
        if (port != I2C_NUM_0 && port != I2C_NUM_1) ERROR_PORT(port, fail);
#else
    #if defined(I2C_ZERO)
        #define I2C_PORT_CHECK(port, fail) \
            if (port != I2C_NUM_0) ERROR_PORT(port, fail);
    #elif defined(I2C_ONE)
        #define I2C_PORT_CHECK(port, fail) \
            if (port != I2C_NUM_1) ERROR_PORT(port, fail);
    #else
        #define I2C_PORT_CHECK(port, fail) \
            ERROR_PORT(port, fail);
    #endif
#endif

// 获取设备句柄（如果不存在则创建）
static esp_err_t get_device_handle(i2c_port_t port, uint16_t addr, i2c_master_dev_handle_t *dev_handle)
{
    i2c_manager_t *mgr = &i2c_managers[port];
    
    // 检查是否已存在该地址的设备句柄
    uint8_t addr_index = addr & 0x7F; // 使用地址的低7位作为索引
    if (mgr->dev_handles[addr_index] != NULL) {
        *dev_handle = mgr->dev_handles[addr_index];
        return ESP_OK;
    }
    
    // 创建新的设备句柄
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = (addr & I2C_ADDR_10) ? I2C_ADDR_BIT_LEN_10 : I2C_ADDR_BIT_LEN_7,
        .device_address = addr & 0x3FF,
        .scl_speed_hz = mgr->clk_speed,
    };
    
    esp_err_t ret = i2c_master_bus_add_device(mgr->bus_handle, &dev_cfg, &mgr->dev_handles[addr_index]);
    if (ret == ESP_OK) {
        *dev_handle = mgr->dev_handles[addr_index];
    }
    
    return ret;
}

// 初始化I2C管理器
esp_err_t I2C_FN(_init)(i2c_port_t port)
{
    I2C_PORT_CHECK(port, ESP_FAIL);
    
    i2c_manager_t *mgr = &i2c_managers[port];
    
    if (mgr->initialized) {
        return ESP_OK; // 已经初始化
    }
    
    esp_err_t ret = ESP_OK;
    
    // 创建互斥锁
    mgr->mutex = xSemaphoreCreateMutex();
    if (mgr->mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex for port %d", port);
        return ESP_ERR_NO_MEM;
    }
    
    // 配置I2C总线
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = port,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    // 根据端口设置GPIO和频率
    if (port == I2C_NUM_0) {
#ifdef I2C_ZERO
        i2c_mst_config.scl_io_num = CONFIG_I2C_MANAGER_0_SCL;
        i2c_mst_config.sda_io_num = CONFIG_I2C_MANAGER_0_SDA;
        mgr->clk_speed = CONFIG_I2C_MANAGER_0_FREQ_HZ;
        mgr->scl_io_num = CONFIG_I2C_MANAGER_0_SCL;
        mgr->sda_io_num = CONFIG_I2C_MANAGER_0_SDA;
#endif
    } else if (port == I2C_NUM_1) {
#ifdef I2C_ONE
        i2c_mst_config.scl_io_num = CONFIG_I2C_MANAGER_1_SCL;
        i2c_mst_config.sda_io_num = CONFIG_I2C_MANAGER_1_SDA;
        mgr->clk_speed = CONFIG_I2C_MANAGER_1_FREQ_HZ;
        mgr->scl_io_num = CONFIG_I2C_MANAGER_1_SCL;
        mgr->sda_io_num = CONFIG_I2C_MANAGER_1_SDA;
#endif
    }
    
    // 创建I2C主机总线
    ret = i2c_new_master_bus(&i2c_mst_config, &mgr->bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus for port %d: %s", port, esp_err_to_name(ret));
        vSemaphoreDelete(mgr->mutex);
        mgr->mutex = NULL;
        return ret;
    }
    
    mgr->initialized = true;
    
    ESP_LOGI(TAG, "I2C Manager initialized for port %d (SDA: %lu, SCL: %lu, speed: %lu Hz)",
             port, mgr->sda_io_num, mgr->scl_io_num, mgr->clk_speed);
    
    return ESP_OK;
}

// 读取数据
esp_err_t I2C_FN(_read)(i2c_port_t port, uint16_t addr, uint32_t reg, uint8_t *buffer, uint16_t size)
{
    I2C_PORT_CHECK(port, ESP_FAIL);
    
    if (buffer == NULL || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 确保初始化
    esp_err_t ret = I2C_FN(_init)(port);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ESP_LOGV(TAG, "Reading port %d, addr 0x%03x, reg 0x%08lx", port, addr, reg);
    
    i2c_master_dev_handle_t dev_handle;
    ret = get_device_handle(port, addr, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get device handle for addr 0x%02x", addr);
        return ret;
    }
    
    // 获取超时时间
    TickType_t timeout = pdMS_TO_TICKS(1000); // 默认1秒
#ifdef I2C_ZERO
    if (port == I2C_NUM_0) {
        timeout = I2C_MANAGER_0_TIMEOUT_TICKS;
    }
#endif
#ifdef I2C_ONE
    if (port == I2C_NUM_1) {
        timeout = I2C_MANAGER_1_TIMEOUT_TICKS;
    }
#endif
    
    // 获取锁
    if (I2C_FN(_lock)(port) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to acquire lock for port %d", port);
        return ESP_ERR_TIMEOUT;
    }
    
    // 执行I2C传输
    if (!(reg & I2C_NO_REG)) {
        // 需要先写寄存器地址
        uint8_t reg_buf[4];
        int reg_len = 1;
        
        if (reg & I2C_REG_16) {
            reg_buf[0] = (reg >> 8) & 0xFF;
            reg_buf[1] = reg & 0xFF;
            reg_len = 2;
        } else {
            reg_buf[0] = reg & 0xFF;
        }
        
        ret = i2c_master_transmit_receive(dev_handle, reg_buf, reg_len, buffer, size, timeout);
    } else {
        // 直接读取
        ret = i2c_master_receive(dev_handle, buffer, size, timeout);
    }
    
    I2C_FN(_unlock)(port);
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "I2C read failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer, size, ESP_LOG_VERBOSE);
    }
    
    return ret;
}

// 写入数据
esp_err_t I2C_FN(_write)(i2c_port_t port, uint16_t addr, uint32_t reg, const uint8_t *buffer, uint16_t size)
{
    I2C_PORT_CHECK(port, ESP_FAIL);
    
    if (buffer == NULL || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 确保初始化
    esp_err_t ret = I2C_FN(_init)(port);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ESP_LOGV(TAG, "Writing port %d, addr 0x%03x, reg 0x%08lx", port, addr, reg);
    
    i2c_master_dev_handle_t dev_handle;
    ret = get_device_handle(port, addr, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get device handle for addr 0x%02x", addr);
        return ret;
    }
    
    // 获取超时时间
    TickType_t timeout = pdMS_TO_TICKS(1000); // 默认1秒
#ifdef I2C_ZERO
    if (port == I2C_NUM_0) {
        timeout = I2C_MANAGER_0_TIMEOUT_TICKS;
    }
#endif
#ifdef I2C_ONE
    if (port == I2C_NUM_1) {
        timeout = I2C_MANAGER_1_TIMEOUT_TICKS;
    }
#endif
    
    // 获取锁
    if (I2C_FN(_lock)(port) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to acquire lock for port %d", port);
        return ESP_ERR_TIMEOUT;
    }
    
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer, size, ESP_LOG_VERBOSE);
    
    // 执行I2C传输
    if (!(reg & I2C_NO_REG)) {
        // 需要写寄存器地址 + 数据
        uint8_t *write_buf = malloc(size + 4); // 最多4字节寄存器地址
        if (write_buf == NULL) {
            I2C_FN(_unlock)(port);
            return ESP_ERR_NO_MEM;
        }
        
        int reg_len = 1;
        if (reg & I2C_REG_16) {
            write_buf[0] = (reg >> 8) & 0xFF;
            write_buf[1] = reg & 0xFF;
            reg_len = 2;
        } else {
            write_buf[0] = reg & 0xFF;
        }
        
        memcpy(&write_buf[reg_len], buffer, size);
        ret = i2c_master_transmit(dev_handle, write_buf, reg_len + size, timeout);
        
        free(write_buf);
    } else {
        // 直接写入数据
        ret = i2c_master_transmit(dev_handle, buffer, size, timeout);
    }
    
    I2C_FN(_unlock)(port);
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "I2C write failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

// 关闭I2C管理器
esp_err_t I2C_FN(_close)(i2c_port_t port)
{
    I2C_PORT_CHECK(port, ESP_FAIL);
    
    i2c_manager_t *mgr = &i2c_managers[port];
    
    if (!mgr->initialized) {
        return ESP_OK; // 已经关闭
    }
    
    // 删除所有设备句柄
    for (int i = 0; i < 128; i++) {
        if (mgr->dev_handles[i] != NULL) {
            i2c_master_bus_rm_device(mgr->dev_handles[i]);
            mgr->dev_handles[i] = NULL;
        }
    }
    
    // 删除I2C总线
    if (mgr->bus_handle != NULL) {
        i2c_del_master_bus(mgr->bus_handle);
        mgr->bus_handle = NULL;
    }
    
    // 删除互斥锁
    if (mgr->mutex != NULL) {
        vSemaphoreDelete(mgr->mutex);
        mgr->mutex = NULL;
    }
    
    mgr->initialized = false;
    
    ESP_LOGI(TAG, "I2C Manager closed for port %d", port);
    
    return ESP_OK;
}

// 获取锁
esp_err_t I2C_FN(_lock)(i2c_port_t port)
{
    I2C_PORT_CHECK(port, ESP_FAIL);
    
    i2c_manager_t *mgr = &i2c_managers[port];
    
    if (!mgr->initialized || mgr->mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    TickType_t timeout = portMAX_DELAY;
#ifdef I2C_ZERO
    if (port == I2C_NUM_0) {
        timeout = I2C_MANAGER_0_LOCK_TIMEOUT_TICKS;
    }
#endif
#ifdef I2C_ONE
    if (port == I2C_NUM_1) {
        timeout = I2C_MANAGER_1_LOCK_TIMEOUT_TICKS;
    }
#endif
    
    if (xSemaphoreTake(mgr->mutex, timeout) == pdTRUE) {
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "Failed to acquire lock for port %d within timeout", port);
        return ESP_ERR_TIMEOUT;
    }
}

// 释放锁
esp_err_t I2C_FN(_unlock)(i2c_port_t port)
{
    I2C_PORT_CHECK(port, ESP_FAIL);
    
    i2c_manager_t *mgr = &i2c_managers[port];
    
    if (!mgr->initialized || mgr->mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreGive(mgr->mutex) == pdTRUE) {
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "Failed to release lock for port %d", port);
        return ESP_ERR_INVALID_STATE;
    }
}

// 强制释放锁
esp_err_t I2C_FN(_force_unlock)(i2c_port_t port)
{
    I2C_PORT_CHECK(port, ESP_FAIL);
    
    i2c_manager_t *mgr = &i2c_managers[port];
    
    if (!mgr->initialized || mgr->mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // 强制释放锁 - 注意这可能会导致竞态条件
    xSemaphoreGive(mgr->mutex);
    
    ESP_LOGW(TAG, "Forced unlock for port %d", port);
    
    return ESP_OK;
}

// 获取I2C管理器句柄 (用于高级用法)
i2c_master_bus_handle_t i2c_manager_get_bus_handle(i2c_port_t port)
{
    if (port >= I2C_NUM_MAX) {
        return NULL;
    }
    
    i2c_manager_t *mgr = &i2c_managers[port];
    
    if (!mgr->initialized) {
        // 尝试初始化
        if (I2C_FN(_init)(port) != ESP_OK) {
            return NULL;
        }
    }
    
    return mgr->bus_handle;
}

// 获取设备句柄 (用于高级用法)
esp_err_t i2c_manager_get_device_handle(i2c_port_t port, uint16_t addr, i2c_master_dev_handle_t *dev_handle)
{
    I2C_PORT_CHECK(port, ESP_FAIL);
    
    if (dev_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 确保初始化
    esp_err_t ret = I2C_FN(_init)(port);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return get_device_handle(port, addr, dev_handle);
}

// 探测I2C设备 - 修复版本
esp_err_t i2c_manager_probe_device(i2c_port_t port, uint16_t addr)
{
    I2C_PORT_CHECK(port, ESP_FAIL);
    
    // 确保初始化
    esp_err_t ret = I2C_FN(_init)(port);
    if (ret != ESP_OK) {
        return ret;
    }
    
    i2c_manager_t *mgr = &i2c_managers[port];
    
    // 获取锁
    if (I2C_FN(_lock)(port) != ESP_OK) {
        return ESP_ERR_TIMEOUT;
    }
    
    // 使用总线句柄进行探测，而不是设备句柄
    ret = i2c_master_probe(mgr->bus_handle, addr & 0x7F, 100); // 100ms 超时
    
    I2C_FN(_unlock)(port);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Device found at address 0x%02x on port %d", addr, port);
    } else {
        ESP_LOGD(TAG, "No device found at address 0x%02x on port %d", addr, port);
    }
    
    return ret;
}

// 扫描I2C总线上的所有设备
esp_err_t i2c_manager_scan_bus(i2c_port_t port, uint8_t *found_devices, size_t max_devices, size_t *num_found)
{
    I2C_PORT_CHECK(port, ESP_FAIL);
    
    if (found_devices == NULL || num_found == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *num_found = 0;
    
    ESP_LOGI(TAG, "Scanning I2C bus on port %d...", port);
    
    for (uint8_t addr = 0x08; addr < 0x78; addr++) { // 标准I2C地址范围
        if (*num_found >= max_devices) {
            break;
        }
        
        if (i2c_manager_probe_device(port, addr) == ESP_OK) {
            found_devices[*num_found] = addr;
            (*num_found)++;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 小延时避免总线拥塞
    }
    
    ESP_LOGI(TAG, "I2C scan complete. Found %d devices on port %d", *num_found, port);
    
    return ESP_OK;
}

#ifndef I2C_OEM
// 兼容性函数 - 如果不是OEM版本，提供标准接口
void* i2c_manager_locking(void)
{
    // 返回锁管理函数指针 - 这里简化实现
    return NULL;
}

void* i2c_hal(i2c_port_t port)
{
    // HAL接口 - 这里简化实现
    return i2c_manager_get_bus_handle(port);
}

#endif