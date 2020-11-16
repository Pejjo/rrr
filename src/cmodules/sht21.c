/*Licensed under RRR MODULE LICENSE VERSION 1.
 *
 * Copyright 2020 Per Johansson <nisse@hoj.nu>
 *
 * This file may be expanded, modified and customized and re-licensed under
 * the terms of either
 *  - GPL version 3 or later
 *  or
 *  - RRR MODULE LICENCE VERSION 1 or later
 *  .
 *
 */

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include "log.h"
#include "cmodule.h"
#include "util/posix.h"
#include "util/rrr_time.h"


#define SOFTRESET 0xFE
#define I2C_ADDRESS 0x40
#define TRIGGER_TEMPERATURE_NO_HOLD 0xF3
#define TRIGGER_HUMIDITY_NO_HOLD 0xF5
#define READ_USER_REG 0xE7
#define READ_SNB_CMD0 0xFA
#define READ_SNB_CMD1 0x0F
#define READ_SNCA_CMD0 0xFC
#define READ_SNCA_CMD1 0xC9


struct config_data {
	char *device_path;
	int device_addr;
	char *device_prefix;
};

static struct config_data config_data = {0};

int state; // Global state variable

uint8_t calculate_checksum(uint8_t *data, int len) {
	// CRC Checksum using the polynomial given in the datasheet
       	#define POLYNOMIAL  0x131  //P(x)=x^8+x^5+x^4+1 = 100110001
	int crc = 0;
        // calculates 8-Bit checksum with given polynomial
        for (int pos=0; pos<len; pos++) {
		crc ^= (data[pos]);
		for (int bit=0; bit<8; bit++) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ POLYNOMIAL;
			}
			else {
				crc = (crc << 1);
			}
		}
	}
        return (uint8_t)crc;
}

float temp_convert(uint8_t *data) {
	int rawval;
	float temp;

	rawval = (data[0]<<8) | (data[1]&0xFC); // join msb and lsb and mask status bits
	temp=-46.85+175.72*rawval/0xffff;
	return temp;
}

float humi_convert(uint8_t *data) {
        int rawval;
        float humi;

        rawval = (data[0]<<8) | (data[1]&0xFC); // join msb and lsb and mask status bits
        humi=-6.0+125*rawval/0xffff;
        return humi;
}

// Read the given I2C slave device's register and return the read value in `*result`:
int i2c_write_read(int i2c_fd, uint8_t addr, uint8_t *wr_dta, uint8_t wr_len, uint8_t *result, uint8_t rd_len) {
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset[1];

    msgs[0].addr = addr;
    msgs[0].flags = 0;
    msgs[0].len = wr_len;
    msgs[0].buf = wr_dta;

    msgs[1].addr = addr;
    msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
    msgs[1].len = rd_len;
    msgs[1].buf = result;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 2;

    if (ioctl(i2c_fd, I2C_RDWR, &msgset) < 0) {
        RRR_MSG_0("Error in ioctl(I2C_RDWR) in i2c_read");
        return -1;
    }
    return 0;
}

int get_serial(int i2c_fd, uint8_t *id)
{
	uint8_t tx_dta[2];
	uint8_t rx_dta[8]={0};
	int ret;

	tx_dta[0] = READ_SNB_CMD0;
	tx_dta[1] = READ_SNB_CMD1;
	ret = i2c_write_read(i2c_fd, config_data.device_addr, tx_dta, 2, rx_dta, 8);
	if (ret < 0)
		goto out;
	if (calculate_checksum(rx_dta,8)!=0)
		RRR_MSG_2("SNB Checksum error\n");

	id[2] = rx_dta[0];
	id[3] = rx_dta[2];
	id[4] = rx_dta[4];
	id[5] = rx_dta[6];

	tx_dta[0] = READ_SNCA_CMD0;
	tx_dta[1] = READ_SNCA_CMD1;
	ret = i2c_write_read(i2c_fd, config_data.device_addr, tx_dta, 2, rx_dta, 6);
	if (ret < 0)
		goto out;
        if (calculate_checksum(rx_dta,6)!=0)
                RRR_MSG_2("SNCA Checksum error\n");

	id[0] = rx_dta[3];
	id[1] = rx_dta[4];
	id[6] = rx_dta[0];
	id[7] = rx_dta[1];

out:

	return ret;
}

int config(RRR_CONFIG_ARGS) {
	struct config_data *data = &config_data;

	int ret = 0;

	ctx->application_ptr = data;

	RRR_MSG_1("Sht21 in config()\n");

	RRR_INSTANCE_CONFIG_PARSE_OPTIONAL_UTF8_DEFAULT_NULL("sht_device_path", device_path);

	if (data->device_path == NULL || *(data->device_path) == '\0') {
		RRR_MSG_0("Could not find setting 'sht_device_path' in configuration\n");
		ret = 1;
		goto out;
	}


        RRR_INSTANCE_CONFIG_PARSE_OPTIONAL_UTF8_DEFAULT_NULL("sht_device_prefix", device_prefix);

        if (data->device_prefix == NULL || *(data->device_prefix) == '\0') {
                RRR_MSG_0("Could not find setting 'sht_device_prefix' in configuration\n");
                ret = 1;
		goto out;
        }

	RRR_INSTANCE_CONFIG_PARSE_OPTIONAL_UNSIGNED("sht_device_address", device_addr, I2C_ADDRESS);

	RRR_MSG_1("Sht21 device: %s\n", data->device_path);

	state=0;

	out:
	return ret;
}

int source(RRR_SOURCE_ARGS) {
	#define BUFSZ 32
	(void)(ctx);
	(void)(message_addr);
        struct rrr_msg_msg *sendmsg;
        char topicbuf[BUFSZ];
        char databuf[BUFSZ];
	uint64_t  timestamp;

	int iic_file;
	uint8_t tmp_buf[8]; // We only need 3 bytes for data, but let the buffer be 8 to also host serial no 

	timestamp=rrr_time_get_64(); // We get a time here, for error messages etc

	//----- OPEN THE I2C BUS -----
	if ((iic_file = open(config_data.device_path, O_RDWR)) < 0) {
		//ERROR HANDLING: you can check errno to see what went wrong
		RRR_MSG_1("Failed to open i2c bus\n");
		return 1;
	}
	
	int addr = config_data.device_addr;          //<<<<<The I2C address of the slave
	if (ioctl(iic_file, I2C_SLAVE, addr) < 0) {
		RRR_MSG_1("Failed to acquire bus access and/or talk to slave.\n");
		//ERROR HANDLING; you can check errno to see what went wrong
		return 1;
	}

	if (state==0) {
		RRR_DBG_2("Init: Sensor Reset\n");
                tmp_buf[0]=SOFTRESET;
                if (write(iic_file, tmp_buf, 1) != 1) {
                 	/* ERROR HANDLING: i2c transaction failed */
                        RRR_DBG_2("Failed to reset sensor, seq %d", state);
                }

		state=1;
	}
	else if (state==1) {
		if(get_serial(iic_file, tmp_buf)==0) {


		RRR_DBG_2("%02x%02x%02x%02x%02x%02x%02x%02x\n",
                     tmp_buf[0], tmp_buf[1], tmp_buf[2], tmp_buf[3],
                     tmp_buf[4], tmp_buf[5], tmp_buf[6], tmp_buf[7]);

			snprintf(databuf, BUFSZ,
		       "%02x%02x%02x%02x%02x%02x%02x%02x\n",
                     		tmp_buf[0], tmp_buf[1], tmp_buf[2], tmp_buf[3],
                     		tmp_buf[4], tmp_buf[5], tmp_buf[6], tmp_buf[7]);

			snprintf(topicbuf, BUFSZ, "%s/id", config_data.device_prefix);
                        rrr_msg_msg_new_with_data (&sendmsg, RRR_TYPE_STR, MSG_CLASS_DATA, rrr_time_get_64(), topicbuf, strlen(topicbuf), databuf, strlen(databuf));
                        rrr_send_and_free(ctx, sendmsg, message_addr);
		}

                tmp_buf[0]=TRIGGER_TEMPERATURE_NO_HOLD;
                if (write(iic_file, tmp_buf, 1) != 1) {
                	/* ERROR HANDLING: i2c transaction failed */
                        RRR_DBG_2("Failed to trig conversion, seq %d", state);
                }
		timestamp=rrr_time_get_64(); // We get time here, since the sensor conversion is now, while the read is delayed several seconds

                state=8;
	}
	else if (state>=8) { // Init and ID done. ID should trig first temp
		if ((state&0x01)==0) {	// Read temp, Trig Humid

			if (read(iic_file, tmp_buf, 3) != 3) {
    				/* ERROR HANDLING: i2c transaction failed */
				RRR_DBG_2("Failed to read data, seq %d", state);
  			} 
			else {
				if (calculate_checksum(tmp_buf, 3)==0) { // CRC=OK
					RRR_DBG_2("Temp: %.2f C\n", temp_convert(tmp_buf));
					snprintf(databuf, BUFSZ,"%.2f",temp_convert(tmp_buf));
		                        snprintf(topicbuf, BUFSZ, "%s/temperature", config_data.device_prefix);
                		        rrr_msg_msg_new_with_data (&sendmsg, RRR_TYPE_STR, MSG_CLASS_DATA, timestamp, topicbuf, strlen(topicbuf), databuf, strlen(databuf));
                        		rrr_send_and_free(ctx, sendmsg, message_addr);
				}
				else
				{
					RRR_DBG_2("Checksum failed\n");
				}
  			}
			tmp_buf[0]=TRIGGER_HUMIDITY_NO_HOLD;
			if (write(iic_file, tmp_buf, 1) != 1) {
                                /* ERROR HANDLING: i2c transaction failed */
                                RRR_DBG_2("Failed to trig conversion, seq %d", state);
                        }
			timestamp=rrr_time_get_64(); // We get time here, since the sensor read may be dealyed several seconds
		}
		else {	// Read Hunmid, Trig Temp
                        if (read(iic_file, tmp_buf, 3) != 3) {
                                /* ERROR HANDLING: i2c transaction failed */
                                RRR_DBG_2("Failed to read data, seq %d", state);
                        }
                        else {
				if (calculate_checksum(tmp_buf, 3)==0) { // CRC=OK
					RRR_DBG_2("Humid: %.2f rH\n", humi_convert(tmp_buf));
					snprintf(databuf, BUFSZ,"%.2f",humi_convert(tmp_buf));
		                        snprintf(topicbuf, BUFSZ, "%s/humidity", config_data.device_prefix);
                		        rrr_msg_msg_new_with_data (&sendmsg, RRR_TYPE_STR, MSG_CLASS_DATA, timestamp, topicbuf, strlen(topicbuf), databuf, strlen(databuf));
                        		rrr_send_and_free(ctx, sendmsg, message_addr);

				}
                                else {
                                        RRR_DBG_2("Checksum failed\n");
                                }

                        }
                        tmp_buf[0]=TRIGGER_TEMPERATURE_NO_HOLD;
                        if (write(iic_file, tmp_buf, 1) != 1) {
                                /* ERROR HANDLING: i2c transaction failed */
                                RRR_DBG_2("Failed to trig conversion, seq %d", state);
                        }
			timestamp=rrr_time_get_64(); // We get time here, since the sensor read may be dealyed several seconds
		}
		state++; // Next step.
		if (state>=128) { // Maybe parameter?
			state=1; // New ID
		}
	}

	rrr_free(message);

	return 0;
}

int cleanup(RRR_CLEANUP_ARGS) {
	struct config_data *data = ctx->application_ptr;

	RRR_MSG_1("Sht21 cleanup\n");

	RRR_FREE_IF_NOT_NULL(data->device_path);
        RRR_FREE_IF_NOT_NULL(data->device_prefix);


	ctx->application_ptr = NULL;

	return 0;
}
