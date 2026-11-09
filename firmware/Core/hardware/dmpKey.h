#ifndef __DMPKEY_H__
#define __DMPKEY_H__

/*
 * dmpKey.h  (精简版)
 * ---------------------------------------------------------------
 * 本工程使用的 inv_mpu_dmp_motion_driver.c 是"野火整合版":
 * 它已经把 DMP 固件地址表(CFG_xx / D_0_xx)直接写进了 .c 文件,
 * 只剩下下面这些 8 位常量需要由本头文件提供。
 *
 * 数值取自 InvenSense eMPL 5.1 原版 dmpKey.h, 不能随意改动,
 * 否则 DMP 写进去的特征配置会错位。
 */

/* ---- DINA 系列 ---- */
#define DINA0C  0x0c
#define DINA20  0x20
#define DINA26  0x26
#define DINA28  0x28
#define DINA2C  0x2c
#define DINA30  0x30
#define DINA36  0x36
#define DINA38  0x38
#define DINA46  0x46
#define DINA4C  0x4c
#define DINA56  0x56
#define DINA66  0x66
#define DINA6C  0x6c
#define DINA76  0x76
#define DINA80  0x80
#define DINA90  0x90
#define DINAAA  0xaa
#define DINAAB  0xab

/* 注意: 下面 6 个不满足 "值 = 名字后两位" 的规律, 是原版里单独定义的 */
#define DINAC0  0xb0
#define DINAC1  0xb1
#define DINAC2  0xb4
#define DINAC3  0xb5
#define DINAC4  0xb8
#define DINAC5  0xb9

#define DINAC9  0xc9
#define DINACD  0xcd
#define DINADF  0xdf
#define DINAF1  0xf1
#define DINAF2  0xf2
#define DINAF8  0xf0
#define DINAFE  0xfe

/* ---- DINB 系列 ---- */
#define DINBC0  0xc0
#define DINBC2  0xc2
#define DINBC4  0xc4
#define DINBC6  0xc6

#endif /* __DMPKEY_H__ */
