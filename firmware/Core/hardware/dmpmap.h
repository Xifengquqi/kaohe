#ifndef __DMPMAP_H__
#define __DMPMAP_H__

/*
 * dmpmap.h  (精简版)
 * ---------------------------------------------------------------
 * DMP 内部 RAM 地址表。本工程只用到"敲击检测"相关的 4 个地址,
 * 数值取自 InvenSense eMPL 5.1 原版 dmpmap.h, 不要随意改动。
 */

#define DMP_TAP_THX     468     /* 敲击阈值 X 轴 */
#define DMP_TAP_THY     472     /* 敲击阈值 Y 轴 */
#define DMP_TAP_THZ     476     /* 敲击阈值 Z 轴 */
#define DMP_TAPW_MIN    478     /* 敲击最小间隔 */

#endif /* __DMPMAP_H__ */
