// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "esp32-hal-i2c.h"
#include "esp32-hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "rom/ets_sys.h"
#include "soc/i2c_reg.h"
#include "soc/i2c_struct.h"
#include "soc/dport_reg.h"

//#define I2C_DEV(i)   (volatile i2c_dev_t *)((i)?DR_REG_I2C1_EXT_BASE:DR_REG_I2C_EXT_BASE)
//#define I2C_DEV(i)   ((i2c_dev_t *)(REG_I2C_BASE(i)))
#define I2C_SCL_IDX(p)  ((p==0)?I2CEXT0_SCL_OUT_IDX:((p==1)?I2CEXT1_SCL_OUT_IDX:0))
#define I2C_SDA_IDX(p) ((p==0)?I2CEXT0_SDA_OUT_IDX:((p==1)?I2CEXT1_SDA_OUT_IDX:0))


struct i2c_struct_t {
    i2c_dev_t * dev;
#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif
    uint8_t num;
};

enum {
    I2C_CMD_RSTART,
    I2C_CMD_WRITE,
    I2C_CMD_READ,
    I2C_CMD_STOP,
    I2C_CMD_END
};

#if CONFIG_DISABLE_HAL_LOCKS
#define I2C_MUTEX_LOCK()
#define I2C_MUTEX_UNLOCK()

static i2c_t _i2c_bus_array[2] = {
    {(volatile i2c_dev_t *)(DR_REG_I2C_EXT_BASE), 0},
    {(volatile i2c_dev_t *)(DR_REG_I2C1_EXT_BASE), 1}
};
#else
#define I2C_MUTEX_LOCK()    do {} while (xSemaphoreTake(i2c->lock, portMAX_DELAY) != pdPASS)
#define I2C_MUTEX_UNLOCK()  xSemaphoreGive(i2c->lock)

static i2c_t _i2c_bus_array[2] = {
    {(volatile i2c_dev_t *)(DR_REG_I2C_EXT_BASE), NULL, 0},
    {(volatile i2c_dev_t *)(DR_REG_I2C1_EXT_BASE), NULL, 1}
};
#endif

i2c_err_t i2cAttachSCL(i2c_t * i2c, int8_t scl)
{
    if(i2c == NULL){
        return I2C_ERROR_DEV;
    }
    pinMode(scl, OUTPUT_OPEN_DRAIN | PULLUP);
    pinMatrixOutAttach(scl, I2C_SCL_IDX(i2c->num), false, false);
    pinMatrixInAttach(scl, I2C_SCL_IDX(i2c->num), false);
    return I2C_ERROR_OK;
}

i2c_err_t i2cDetachSCL(i2c_t * i2c, int8_t scl)
{
    if(i2c == NULL){
        return I2C_ERROR_DEV;
    }
    pinMatrixOutDetach(scl, false, false);
    pinMatrixInDetach(I2C_SCL_IDX(i2c->num), false, false);
    pinMode(scl, INPUT);
    return I2C_ERROR_OK;
}

i2c_err_t i2cAttachSDA(i2c_t * i2c, int8_t sda)
{
    if(i2c == NULL){
        return I2C_ERROR_DEV;
    }
    pinMode(sda, OUTPUT_OPEN_DRAIN | PULLUP);
    pinMatrixOutAttach(sda, I2C_SDA_IDX(i2c->num), false, false);
    pinMatrixInAttach(sda, I2C_SDA_IDX(i2c->num), false);
    return I2C_ERROR_OK;
}

i2c_err_t i2cDetachSDA(i2c_t * i2c, int8_t sda)
{
    if(i2c == NULL){
        return I2C_ERROR_DEV;
    }
    pinMatrixOutDetach(sda, false, false);
    pinMatrixInDetach(I2C_SDA_IDX(i2c->num), false, false);
    pinMode(sda, INPUT);
    return I2C_ERROR_OK;
}

/*
 * index     - command index (0 to 15)
 * op_code   - is the command
 * byte_num  - This register is to store the amounts of data that is read and written. byte_num in RSTART, STOP, END is null.
 * ack_val   - Each data byte is terminated by an ACK bit used to set the bit level.
 * ack_exp   - This bit is to set an expected ACK value for the transmitter.
 * ack_check - This bit is to decide whether the transmitter checks ACK bit. 1 means yes and 0 means no.
 * */
void i2cSetCmd(i2c_t * i2c, uint8_t index, uint8_t op_code, uint8_t byte_num, bool ack_val, bool ack_exp, bool ack_check)
{
    i2c->dev->command[index].val = 0;
    i2c->dev->command[index].ack_en = ack_check;
    i2c->dev->command[index].ack_exp = ack_exp;
    i2c->dev->command[index].ack_val = ack_val;
    i2c->dev->command[index].byte_num = byte_num;
    i2c->dev->command[index].op_code = op_code;
}

void i2cResetCmd(i2c_t * i2c){
    int i;
    for(i=0;i<16;i++){
        i2c->dev->command[i].val = 0;
    }
}

void i2cResetFiFo(i2c_t * i2c)
{
    i2c->dev->fifo_conf.tx_fifo_rst = 1;
    i2c->dev->fifo_conf.tx_fifo_rst = 0;
    i2c->dev->fifo_conf.rx_fifo_rst = 1;
    i2c->dev->fifo_conf.rx_fifo_rst = 0;
}

i2c_err_t i2cWrite(i2c_t * i2c, uint16_t address, bool addr_10bit, uint8_t * data, uint8_t len, bool sendStop)
{
    int i;
    uint8_t index = 0;
    uint8_t dataLen = len + (addr_10bit?2:1);
    address = (address << 1);

    if(i2c == NULL){
        return I2C_ERROR_DEV;
    }

    I2C_MUTEX_LOCK();

    while(dataLen) {
        uint8_t willSend = (dataLen > 32)?32:dataLen;
        uint8_t dataSend = willSend;

        i2cResetFiFo(i2c);
        i2cResetCmd(i2c);
        //Clear Interrupts
        i2c->dev->int_clr.val = 0xFFFFFFFF;

        //CMD START
        i2cSetCmd(i2c, 0, I2C_CMD_RSTART, 0, false, false, false);

        //CMD WRITE(ADDRESS + DATA)
        if(!index) {
            i2c->dev->fifo_data.data = address & 0xFF;
            dataSend--;
            if(addr_10bit) {
                i2c->dev->fifo_data.data = (address >> 8) & 0xFF;
                dataSend--;
            }
        }
        i = 0;
        while(i<dataSend) {
            i++;
            i2c->dev->fifo_data.val = data[index++];
            while(i2c->dev->status_reg.tx_fifo_cnt < i);
        }
        i2cSetCmd(i2c, 1, I2C_CMD_WRITE, willSend, false, false, true);
        dataLen -= willSend;

        //CMD STOP or CMD END if there is more data
        if(dataLen || !sendStop) {
            i2cSetCmd(i2c, 2, I2C_CMD_END, 0, false, false, false);
        } else if(sendStop) {
            i2cSetCmd(i2c, 2, I2C_CMD_STOP, 0, false, false, false);
        }

        //START Transmission
        i2c->dev->ctr.trans_start = 1;

        //WAIT Transmission
        uint32_t startAt = millis();
        while(1) {
            //have been looping for too long
            if((millis() - startAt)>50){
                //log_e("Timeout! Addr: %x", address >> 1);
                I2C_MUTEX_UNLOCK();
                return I2C_ERROR_BUS;
            }

            //Bus failed (maybe check for this while waiting?
            if(i2c->dev->int_raw.arbitration_lost) {
                //log_e("Bus Fail! Addr: %x", address >> 1);
                I2C_MUTEX_UNLOCK();
                return I2C_ERROR_BUS;
            }

            //Bus timeout
            if(i2c->dev->int_raw.time_out) {
                //log_e("Bus Timeout! Addr: %x", address >> 1);
                I2C_MUTEX_UNLOCK();
                return I2C_ERROR_TIMEOUT;
            }

            //Transmission did not finish and ACK_ERR is set
            if(i2c->dev->int_raw.ack_err) {
                //log_w("Ack Error! Addr: %x", address >> 1);
                while(i2c->dev->status_reg.bus_busy);
                I2C_MUTEX_UNLOCK();
                return I2C_ERROR_ACK;
            }

            if((sendStop && i2c->dev->command[2].done) || !i2c->dev->status_reg.bus_busy){
                break;
            }
        }

    }
    I2C_MUTEX_UNLOCK();
    return I2C_ERROR_OK;
}

i2c_err_t i2cRead(i2c_t * i2c, uint16_t address, bool addr_10bit, uint8_t * data, uint8_t len, bool sendStop)
{
    address = (address << 1) | 1;
    uint8_t addrLen = (addr_10bit?2:1);
    uint8_t index = 0;
    uint8_t cmdIdx;
    uint8_t willRead;

    if(i2c == NULL){
        return I2C_ERROR_DEV;
    }

    I2C_MUTEX_LOCK();

    i2cResetFiFo(i2c);
    i2cResetCmd(i2c);

    //CMD START
    i2cSetCmd(i2c, 0, I2C_CMD_RSTART, 0, false, false, false);

    //CMD WRITE ADDRESS
    i2c->dev->fifo_data.val = address & 0xFF;
    if(addr_10bit) {
        i2c->dev->fifo_data.val = (address >> 8) & 0xFF;
    }
    i2cSetCmd(i2c, 1, I2C_CMD_WRITE, addrLen, false, false, true);

    while(len) {
        cmdIdx = (index)?0:2;
        willRead = (len > 32)?32:(len-1);
        if(cmdIdx){
            i2cResetFiFo(i2c);
        }

        if(willRead){
            i2cSetCmd(i2c, cmdIdx++, I2C_CMD_READ, willRead, false, false, false);
        }

        if((len - willRead) > 1) {
            i2cSetCmd(i2c, cmdIdx++, I2C_CMD_END, 0, false, false, false);
        } else {
            willRead++;
            i2cSetCmd(i2c, cmdIdx++, I2C_CMD_READ, 1, true, false, false);
            if(sendStop) {
                i2cSetCmd(i2c, cmdIdx++, I2C_CMD_STOP, 0, false, false, false);
            }
        }

        //Clear Interrupts
        i2c->dev->int_clr.val = 0xFFFFFFFF;

        //START Transmission
        i2c->dev->ctr.trans_start = 1;

        //WAIT Transmission
        uint32_t startAt = millis();
        while(1) {
            //have been looping for too long
            if((millis() - startAt)>50){
                //log_e("Timeout! Addr: %x", address >> 1);
                I2C_MUTEX_UNLOCK();
                return I2C_ERROR_BUS;
            }

            //Bus failed (maybe check for this while waiting?
            if(i2c->dev->int_raw.arbitration_lost) {
                //log_e("Bus Fail! Addr: %x", address >> 1);
                I2C_MUTEX_UNLOCK();
                return I2C_ERROR_BUS;
            }

            //Bus timeout
            if(i2c->dev->int_raw.time_out) {
                //log_e("Bus Timeout! Addr: %x", address >> 1);
                I2C_MUTEX_UNLOCK();
                return I2C_ERROR_TIMEOUT;
            }

            //Transmission did not finish and ACK_ERR is set
            if(i2c->dev->int_raw.ack_err) {
                //log_w("Ack Error! Addr: %x", address >> 1);
                I2C_MUTEX_UNLOCK();
                return I2C_ERROR_ACK;
            }

            if(i2c->dev->command[cmdIdx-1].done) {
                break;
            }
        }

        int i = 0;
        while(i<willRead) {
            i++;
            data[index++] = i2c->dev->fifo_data.val & 0xFF;
        }
        len -= willRead;
    }
    I2C_MUTEX_UNLOCK();
    return I2C_ERROR_OK;
}

i2c_err_t i2cSetFrequency(i2c_t * i2c, uint32_t clk_speed)
{
    i2c_err_t error_type = I2C_ERROR_DEV;

    if(i2c == NULL){
        return error_type;
    }

    // if APB_CLK_FREQ == 80MHz, period = 12.5ns = 12500 ps = 80/us
    uint32_t APB_period_in_ps = 1000000000 / (APB_CLK_FREQ / 1000); //12500
    uint32_t clk_period_in_ps = 1000000000 / (clk_speed    / 1000); //2000000 // works down to 1 khz

    // APB Clock periods for a full I2C clock cycle
    uint32_t fullPeriod_in_APB_clocks = (APB_CLK_FREQ/clk_speed);

    I2C_MUTEX_LOCK();

    if (clk_speed <= 100000)
    {
        // Standard-mode, >= 10000ns/cycle, l/H = 2 (typ.)
        uint32_t scl_min_low_in_ns  = 4700; // 4.7us
        uint32_t scl_min_high_in_ns = 4000; // 4.0us
        int32_t  scl_pad_low_in_ns  =    0;
        int32_t  scl_pad_high_in_ns =    0;

        uint32_t scl_high_period_in_ps =
            (scl_min_high_in_ns + scl_pad_high_in_ns) * 1000;
        uint32_t scl_low_period_in_ps  =
            clk_period_in_ps - scl_high_period_in_ps + scl_pad_low_in_ns * 1000;

        //tHIGH - APB Clock periods for SCL high (<16384)
        i2c->dev->scl_high_period.period =
            clk_period_in_ps / 2 / APB_period_in_ps - 1 - 8;
            //scl_high_period_in_ps / APB_period_in_ps;
        //tLOW - APB Clock periods for SCL low (<16384)
        i2c->dev->scl_low_period.period =
            clk_period_in_ps / 2 / APB_period_in_ps - 1 - 8;
            //scl_low_period_in_ps / APB_period_in_ps;
        //tHD;STA - APB Clock periods between negedge of SDA and
        //          the negedge of SCL for start mark (<1024)
        i2c->dev->scl_start_hold.time = (4000000) / APB_period_in_ps; // 4.0us
        //tSU;STA - APB Clock periods between the posedge of SCL and
        //          the negedge of SDA for restart mark (<1024)
        i2c->dev->scl_rstart_setup.time = (4700000) / APB_period_in_ps; // 4.7us
        //tBUF - APB Clock periods after the STOP bit's posedge (<16384)
        i2c->dev->scl_stop_hold.time = (4700000) / APB_period_in_ps; // 4.7us
        //tSU;STO - APB Clock periods between the posedge of SCL and
        //          the posedge of SDA (<1024)
        i2c->dev->scl_stop_setup.time = (4000000) / APB_period_in_ps; // 4.0us
        //tHD;DAT - APB Clock periods to hold data after
        //          the negedge of SCL (<1024)
        i2c->dev->sda_hold.time = fullPeriod_in_APB_clocks/8; // >=300ns (>=24 APB clocks)
        //(esp) - APB Clock periods delay between the posedge of SCL and
        //        sampling SDA (<1024)
        i2c->dev->sda_sample.time = fullPeriod_in_APB_clocks/8; // Why? 100@100k, 25@400k,10@1000k
        //(esp) - APB Clock periods max for receiving a data (<1048575)
        i2c->dev->timeout.tout = (2500000/APB_period_in_ps)*1000; // 2.5ms
        //tSP - APB Clock periods max spike rejection (<8)
        //      (not specified for standard mode)
        i2c->dev->scl_filter_cfg.en = 1;
        i2c->dev->scl_filter_cfg.thres = (50000) / APB_period_in_ps; // 50ns
        i2c->dev->sda_filter_cfg.en = 1;
        i2c->dev->sda_filter_cfg.thres = (50000) / APB_period_in_ps; // 50ns

        error_type = I2C_ERROR_OK;
    }
    else if (clk_speed <= 400000)
    {
        // Fast-mode, >= 2500ns/cycle, l/H = 16/9 (typ.)
        uint32_t scl_min_low_in_ns  = 1300; // 1.3us
        uint32_t scl_min_high_in_ns =  600; // 0.6us
        int32_t  scl_pad_low_in_ns  =    0;
        int32_t  scl_pad_high_in_ns =    0;

        uint32_t scl_high_period_in_ps =
            (scl_min_high_in_ns + scl_pad_high_in_ns) * 1000;
        uint32_t scl_low_period_in_ps  =
            clk_period_in_ps - scl_high_period_in_ps + scl_pad_low_in_ns * 1000;

        //tHIGH - APB Clock periods for SCL high (<16384)
        i2c->dev->scl_high_period.period =
            clk_period_in_ps / 2 / APB_period_in_ps - 1 - 8;
            //scl_high_period_in_ps / APB_period_in_ps;
        //tLOW - APB Clock periods for SCL low (<16384)
        i2c->dev->scl_low_period.period =
            clk_period_in_ps / 2 / APB_period_in_ps - 1 - 8;
            //scl_low_period_in_ps / APB_period_in_ps;
        //tHD;STA - APB Clock periods between negedge of SDA and
        //          the negedge of SCL for start mark (<1024)
        i2c->dev->scl_start_hold.time = (600000) / APB_period_in_ps; // 0.6us
        //tSU;STA - APB Clock periods between the posedge of SCL and
        //          the negedge of SDA for restart mark (<1024)
        i2c->dev->scl_rstart_setup.time = (600000) / APB_period_in_ps; // 0.6us
        //tBUF - APB Clock periods after the STOP bit's posedge (<16384)
        i2c->dev->scl_stop_hold.time = (1300000) / APB_period_in_ps; // 1.3us
        //tSU;STO - APB Clock periods between the posedge of SCL and
        //          the posedge of SDA (<1024)
        i2c->dev->scl_stop_setup.time = (600000) / APB_period_in_ps; // 0.6us
        //tHD;DAT - APB Clock periods to hold data after
        //          the negedge of SCL (<1024)
        i2c->dev->sda_hold.time = fullPeriod_in_APB_clocks/8; // >=300ns (>=24 APB clocks) - tried "(400000) / APB_period_in_ps" which caused very narrow data pulses - does this be proportional to sda_sample.time???
        //(esp) - APB Clock periods delay between the posedge of SCL and
        //        sampling SDA (<1024)
        i2c->dev->sda_sample.time = fullPeriod_in_APB_clocks/8; // Why? 100@100k, 25@400k,10@1000k
        //(esp) - APB Clock periods max for receiving a data (<1048575)
        i2c->dev->timeout.tout = (2500000/APB_period_in_ps)*1000; // 2.5ms
        //tSP - APB Clock periods max spike rejection (<8)
        //      (not specified for standard mode)
        i2c->dev->scl_filter_cfg.en = 1;
        i2c->dev->scl_filter_cfg.thres = (50000) / APB_period_in_ps; // 50ns
        i2c->dev->sda_filter_cfg.en = 1;
        i2c->dev->sda_filter_cfg.thres = (50000) / APB_period_in_ps; // 50ns

        error_type = I2C_ERROR_OK;
    }
    else if (clk_speed <= 1000000)
    {
        // Fast-mode Plus, >= 1000ns/cycle, l/H = 1 (typ.)???
        uint32_t scl_min_low_in_ns  =  500; // 0.5us
        uint32_t scl_min_high_in_ns =  260; // 0.26us
        int32_t  scl_pad_low_in_ns  =    0;
        int32_t  scl_pad_high_in_ns =    0;

        uint32_t scl_high_period_in_ps =
            (scl_min_high_in_ns + scl_pad_high_in_ns) * 1000;
        uint32_t scl_low_period_in_ps  =
            clk_period_in_ps - scl_high_period_in_ps + scl_pad_low_in_ns * 1000;

        //tHIGH - APB Clock periods for SCL high (<16384)
        i2c->dev->scl_high_period.period =
            clk_period_in_ps / 2 / APB_period_in_ps - 1 - 8;
            //scl_high_period_in_ps / APB_period_in_ps;
        //tLOW - APB Clock periods for SCL low (<16384)
        i2c->dev->scl_low_period.period =
            clk_period_in_ps / 2 / APB_period_in_ps - 1 - 8;
            //scl_low_period_in_ps / APB_period_in_ps;
        //tHD;STA - APB Clock periods between negedge of SDA and
        //          the negedge of SCL for start mark (<1024)
        i2c->dev->scl_start_hold.time = (260000) / APB_period_in_ps; // 0.26us
        //tSU;STA - APB Clock periods between the posedge of SCL and
        //          the negedge of SDA for restart mark (<1024)
        i2c->dev->scl_rstart_setup.time = (260000) / APB_period_in_ps; // 0.26us
        //tBUF - APB Clock periods after the STOP bit's posedge (<16384)
        i2c->dev->scl_stop_hold.time = (500000) / APB_period_in_ps; // 0.5us
        //tSU;STO - APB Clock periods between the posedge of SCL and
        //          the posedge of SDA (<1024)
        i2c->dev->scl_stop_setup.time = (260000) / APB_period_in_ps; // 0.26us
        //tHD;DAT - APB Clock periods to hold data after
        //          the negedge of SCL (<1024)
        i2c->dev->sda_hold.time = fullPeriod_in_APB_clocks/8; // >=300ns (>=24 APB clocks)
        //(esp) - APB Clock periods delay between the posedge of SCL and
        //        sampling SDA (<1024)
        i2c->dev->sda_sample.time = fullPeriod_in_APB_clocks/8; // Why? 100@100k, 25@400k,10@1000k
        //(esp) - APB Clock periods max for receiving a data (<1048575)
        i2c->dev->timeout.tout = (2500000/APB_period_in_ps)*1000; // 2.5ms
        //tSP - APB Clock periods max spike rejection (<8)
        //      (not specified for standard mode)
        i2c->dev->scl_filter_cfg.en = 1;
        i2c->dev->scl_filter_cfg.thres = (50000) / APB_period_in_ps; // 50ns
        i2c->dev->sda_filter_cfg.en = 1;
        i2c->dev->sda_filter_cfg.thres = (50000) / APB_period_in_ps; // 50ns

        error_type = I2C_ERROR_OK;
    }
    else {
        error_type = I2C_ERROR_DEV;       
    }
    I2C_MUTEX_UNLOCK();
    return error_type;
}

uint32_t i2cGetFrequency(i2c_t * i2c)
{
    if(i2c == NULL){
        return 0;
    }

    return APB_CLK_FREQ/(i2c->dev->scl_low_period.period+i2c->dev->scl_high_period.period);
}

/*
 * mode          - 0 = Slave, 1 = Master
 * slave_addr    - I2C Address
 * addr_10bit_en - enable slave 10bit address mode.
 * */

i2c_t * i2cInit(uint8_t i2c_num, uint16_t slave_addr, bool addr_10bit_en)
{
    if(i2c_num > 1){
        return NULL;
    }

    i2c_t * i2c = &_i2c_bus_array[i2c_num];

#if !CONFIG_DISABLE_HAL_LOCKS
    if(i2c->lock == NULL){
        i2c->lock = xSemaphoreCreateMutex();
        if(i2c->lock == NULL) {
            return NULL;
        }
    }
#endif

    if(i2c_num == 0) {
        SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG,DPORT_I2C_EXT0_CLK_EN);
        CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG,DPORT_I2C_EXT0_RST);
    } else {
        SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG,DPORT_I2C_EXT1_CLK_EN);
        CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG,DPORT_I2C_EXT1_RST);
    }
    
    I2C_MUTEX_LOCK();
    i2c->dev->ctr.val = 0;
    i2c->dev->ctr.ms_mode = (slave_addr == 0);
    i2c->dev->ctr.sda_force_out = 1 ;
    i2c->dev->ctr.scl_force_out = 1 ;
    i2c->dev->ctr.clk_en = 1;

    //the max clock number of receiving  a data
    i2c->dev->timeout.tout = 400000;//clocks max=1048575
    //disable apb nonfifo access
    i2c->dev->fifo_conf.nonfifo_en = 0;

    i2c->dev->slave_addr.val = 0;
    if (slave_addr) {
        i2c->dev->slave_addr.addr = slave_addr;
        i2c->dev->slave_addr.en_10bit = addr_10bit_en;
    }
    I2C_MUTEX_UNLOCK();

    return i2c;
}

void i2cInitFix(i2c_t * i2c){
    if(i2c == NULL){
        return;
    }
    I2C_MUTEX_LOCK();
    i2cResetFiFo(i2c);
    i2cResetCmd(i2c);
    i2c->dev->int_clr.val = 0xFFFFFFFF;
    i2cSetCmd(i2c, 0, I2C_CMD_RSTART, 0, false, false, false);
    i2c->dev->fifo_data.data = 0;
    i2cSetCmd(i2c, 1, I2C_CMD_WRITE, 1, false, false, false);
    i2cSetCmd(i2c, 2, I2C_CMD_STOP, 0, false, false, false);
    i2c->dev->ctr.trans_start = 1;
    while(!i2c->dev->command[2].done);
    I2C_MUTEX_UNLOCK();
}
