
//June 12 2012
//This program is given to you without any warranty.
//The author disclaims copyright to this source code.

//based on RM-ITG-3050.pdf
//Document Number: RM-ITG-3050-00
//Revision: 1.1
//Release Date: 05/19/2011

//interface
#define ITG3050_I2C_ADDRESS_0	0x68
#define ITG3050_I2C_ADDRESS_1	0x69

#define ITG3050_WHO_AM_I	0x0
#define ITG3050_X_OFFS_USRH	0x0c
#define ITG3050_X_OFFS_USRL	0x0d
#define ITG3050_Y_OFFS_USRH	0x0e
#define ITG3050_Y_OFFS_USRL	0x0f
#define ITG3050_Z_OFFS_USRH	0x10 
#define ITG3050_Z_OFFS_USRL	0x11
#define ITG3050_FIFO_EN		0x12
#define ITG3050_AUX_VDDIO	0x13
#define ITG3050_AUX_SLV_ADDR	0x14
#define ITG3050_SMPLRT_DIV	0x15
#define ITG3050_DLPF_FS_SYNC	0x16
#define ITG3050_INT_CFG		0x17
#define ITG3050_AUX_ADDR	0x18
#define ITG3050_INT_STATUS	0x1a
#define ITG3050_TEMP_OUT_H	0x1b
#define ITG3050_TEMP_OUT_L	0x1c
#define ITG3050_GYRO_XOUT_H	0x1d
#define ITG3050_GYRO_XOUT_L	0x1e
#define ITG3050_GYRO_YOUT_H	0x1f
#define ITG3050_GYRO_YOUT_L	0x20
#define ITG3050_GYRO_ZOUT_H	0x21
#define ITG3050_GYRO_ZOUT_L	0x22
#define ITG3050_AUX_XOUT_H	0x23
#define ITG3050_AUX_XOUT_L	0x24
#define ITG3050_AUX_YOUT_H	0x25
#define ITG3050_AUX_YOUT_L	0x26
#define ITG3050_AUX_ZOUT_H	0x27
#define ITG3050_AUX_ZOUT_L	0x28
#define ITG3050_FIFO_COUNTH	0x3a
#define ITG3050_FIFO_COUNTL	0x3b
#define ITG3050_FIFO_R		0x3c
#define ITG3050_USER_CTRL	0x3d
#define ITG3050_PWR_MGM		0x3e

//WHO_AM_I
#define ITG3050_I2C_IF_ DIS	7
#define ITG3050_ID_MASK		0x7e
#define ITG3050_ID_POS		1
#define ITG3050_ID_VALUE	0x68

//FIFO_EN
#define ITG3050_TEMP_OUT	7
#define ITG3050_GYRO_XOUT	6
#define ITG3050_GYRO_YOUT	5
#define ITG3050_GYRO_ZOUT	4
#define ITG3050_AUX_XOUT	3
#define ITG3050_AUX_YOUT	2
#define ITG3050_AUX_ZOUT	1
#define ITG3050_FIFO_FOOTER	0

//AUX_VDDIO
#define ITG3050_AUX_VDDIO	2

//AUX_SLV_ADDR
#define ITG3050_CLKOUT_EN	7
#define ITG3050_AUX_ID_MASK	0x7f

//DLPF_FS_SYNC
#define ITG3050_EXT_SYNC_SET2	7
#define ITG3050_EXT_SYNC_SET1	6
#define ITG3050_EXT_SYNC_SET0	5
#define ITG3050_FS_SEL1		4
#define ITG3050_FS_SEL0		3
#define ITG3050_DLPF_CFG2	2
#define ITG3050_DLPF_CFG1	1
#define ITG3050_DLPF_CFG0	0

//INT_CFG
#define ITG3050_ACTL			7
#define ITG3050_OPEN			6
#define ITG3050_LATCH_INT_EN		5
#define ITG3050_INT_ANYRD_2CLEAR	4
#define ITG3050_ITG_RDY_EN		2
#define ITG3050_RAW_RDY_EN		0

//INT_STATUS
#define ITG3050_ITG_RDY		2
#define ITG3050_RAW_DATA_RDY	0

//FIFO_COUNTH
#define ITG3050_FIFO_COUNT_H_MASK	0x03

//USER_CTRL
#define ITG3050_FIFO_ EN	6
#define ITG3050_AUX_IF_EN	5
#define ITG3050_AUX_IF_RST	3
#define ITG3050_FIFO_RST	1
#define ITG3050_GYRO_RST	0

//PWR_MGM
#define ITG3050_H_RESET		7
#define ITG3050_SLEEP		6
#define ITG3050_STBY_XG		5
#define ITG3050_STBY_YG		4
#define ITG3050_STBY_ZG		3
#define ITG3050_CLK_SEL2	2
#define ITG3050_CLK_SEL1	1
#define ITG3050_CLK_SEL0	0



