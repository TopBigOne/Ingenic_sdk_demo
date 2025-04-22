/*
 * sample-common.c
 *
 * Copyright (C) 2014 Ingenic Semiconductor Co.,Ltd
 */

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>

#include <imp/imp_log.h>
#include <imp/imp_common.h>
#include <imp/imp_system.h>
#include <imp/imp_framesource.h>
#include <imp/imp_encoder.h>
#include <imp/imp_isp.h>
#include <imp/imp_osd.h>

#include "sample-common.h"

#define TAG "Sample-Common"

const IMPEncoderRcMode S_RC_METHOD = IMP_ENC_RC_MODE_CBR;

#define SHOW_FRM_BITRATE
#ifdef SHOW_FRM_BITRATE
#define FRM_BIT_RATE_TIME 2
#define STREAM_TYPE_NUM 3
static int frmrate_sp[STREAM_TYPE_NUM] = {0};
static int statime_sp[STREAM_TYPE_NUM] = {0};
static int bitrate_sp[STREAM_TYPE_NUM] = {0};
#endif

int direct_switch = 0;

int gosd_enable = 0; /* 1: ipu osd, 2: isp osd, 3: ipu osd and isp osd */

//  通道配置数组是3：
struct chn_conf chn[FS_CHN_NUM] = {

    // index 0:--------------------------------------------------------------------------------------------------
    {
        .index = CH0_INDEX, // 下标-0
        .enable = CHN0_EN, // : 1
        .payloadType = IMP_ENC_PROFILE_AVC_MAIN, //IMP_ENC_PROFILE_HEVC_MAIN, ：h264
        .fs_chn_attr = {
            .i2dattr.i2d_enable = 0,
            .i2dattr.flip_enable = 0,
            .i2dattr.mirr_enable = 0,
            .i2dattr.rotate_enable = 1,
            .i2dattr.rotate_angle = 270,

            .pixFmt = PIX_FMT_NV12,
            .outFrmRateNum = FIRST_SENSOR_FRAME_RATE_NUM,
            .outFrmRateDen = FIRST_SENSOR_FRAME_RATE_DEN,
            .nrVBs = 2,
            .type = FS_PHY_CHANNEL,

            .crop.enable = FIRST_CROP_EN,
            .crop.top = 0,
            .crop.left = 0,
            .crop.width = FIRST_SENSOR_WIDTH,
            .crop.height = FIRST_SENSOR_HEIGHT,

            .scaler.enable = 1,
            .scaler.outwidth = FIRST_SENSOR_WIDTH,
            .scaler.outheight = FIRST_SENSOR_HEIGHT,

            .picWidth = FIRST_SENSOR_WIDTH,
            .picHeight = FIRST_SENSOR_HEIGHT,
        },
        .framesource_chn = {DEV_ID_FS, CH0_INDEX, 0},
        .imp_encoder = {DEV_ID_ENC, CH0_INDEX, 0}, // 编码器
    },
    // index 1:--------------------------------------------------------------------------------------------------
    {
        .index = CH1_INDEX,
        .enable = CHN1_EN, // : 1
        .payloadType = IMP_ENC_PROFILE_AVC_MAIN, //IMP_ENC_PROFILE_HEVC_MAIN, h264-main
        .fs_chn_attr = {
            .i2dattr.i2d_enable = 0,
            .i2dattr.flip_enable = 0,
            .i2dattr.mirr_enable = 0,
            .i2dattr.rotate_enable = 1,
            .i2dattr.rotate_angle = 270,

            .pixFmt = PIX_FMT_NV12,
            .outFrmRateNum = FIRST_SENSOR_FRAME_RATE_NUM,
            .outFrmRateDen = FIRST_SENSOR_FRAME_RATE_DEN,
            .nrVBs = 2,
            .type = FS_PHY_CHANNEL,

            .crop.enable = 0,
            .crop.top = 0,
            .crop.left = 0,
            .crop.width = FIRST_SENSOR_WIDTH,
            .crop.height = FIRST_SENSOR_HEIGHT,

            .scaler.enable = 1,
            .scaler.outwidth = FIRST_SENSOR_WIDTH_SECOND,
            .scaler.outheight = FIRST_SENSOR_HEIGHT_SECOND,

            .picWidth = FIRST_SENSOR_WIDTH_SECOND,
            .picHeight = FIRST_SENSOR_HEIGHT_SECOND,
        },
        .framesource_chn = {DEV_ID_FS, CH1_INDEX, 0},
        .imp_encoder = {DEV_ID_ENC, CH1_INDEX, 0}, // 编码器
    },
    // index 2:--------------------------------------------------------------------------------------------------
    {
        .index = CH2_INDEX,
        .enable = CHN2_EN,// enable = 0;
        .payloadType = IMP_ENC_PROFILE_HEVC_MAIN, // h265-main
        .fs_chn_attr = {
            .i2dattr.i2d_enable = 0,
            .i2dattr.flip_enable = 0,
            .i2dattr.mirr_enable = 0,
            .i2dattr.rotate_enable = 1,
            .i2dattr.rotate_angle = 270,

            .pixFmt = PIX_FMT_NV12,
            .outFrmRateNum = FIRST_SENSOR_FRAME_RATE_NUM,
            .outFrmRateDen = FIRST_SENSOR_FRAME_RATE_DEN,
            .nrVBs = 2,
            .type = FS_PHY_CHANNEL,

            .crop.enable = 0,
            .crop.top = 0,
            .crop.left = 0,
            .crop.width = FIRST_SENSOR_WIDTH,
            .crop.height = FIRST_SENSOR_HEIGHT,

            .scaler.enable = 1,
            .scaler.outwidth = FIRST_SENSOR_WIDTH_THIRD,
            .scaler.outheight = FIRST_SENSOR_HEIGHT_THIRD,

            .picWidth = FIRST_SENSOR_WIDTH_THIRD,
            .picHeight = FIRST_SENSOR_HEIGHT_THIRD,
        },
        .framesource_chn = {DEV_ID_FS, CH2_INDEX, 0},
        .imp_encoder = {DEV_ID_ENC, CH2_INDEX, 0}, // 编码器
    }
};

// 摄像头注册信息
IMPSensorInfo Def_Sensor_Info[1] = {
    {
        FIRST_SNESOR_NAME, // "gc5603"
        TX_SENSOR_CONTROL_INTERFACE_I2C, // I2C控制总线
        .i2c = {FIRST_SNESOR_NAME, FIRST_I2C_ADDR, FIRST_I2C_ADAPTER_ID},
        FIRST_RST_GPIO,
        FIRST_PWDN_GPIO,
        FIRST_POWER_GPIO,
        FIRST_SENSOR_ID,
        FIRST_VIDEO_INTERFACE,
        FIRST_MCLK,
        FIRST_DEFAULT_BOOT
    }
};

IMPSensorInfo sensor_info[1];

int sample_system_init() {
    IMP_LOG_DBG(TAG, "sample_system_init start\n");
    PRINT_CURR_FUNC(__FUNCTION__)
    int ret = 0;

    /* isp osd and ipu osd buffer size set */
    if (1 == gosd_enable) {
        /* only use ipu osd */
        // rmem池大小
        IMP_OSD_SetPoolSize(512 * 1024);
    } else if (2 == gosd_enable) {
        /* only use isp osd */
        // 创建ISPOSD使用的rmem内存大小
        IMP_ISP_Tuning_SetOsdPoolSize(512 * 1024);
    } else if (3 == gosd_enable) {
        /* use ipu osd and isp osd */
        IMP_OSD_SetPoolSize(512 * 1024);
        IMP_ISP_Tuning_SetOsdPoolSize(512 * 1024);
    }
    // 摄像头注册信息
    memset(&sensor_info, 0, sizeof(sensor_info));
    memcpy(&sensor_info[0], &Def_Sensor_Info[0], sizeof(IMPSensorInfo));

    /* open isp 打开ISP模块 */
    ret = IMP_ISP_Open();
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "failed to open ISP\n");
        return -1;
    }

    /* add sensor */
    // 添加一个sensor，用于向ISP模块提供数据源 ：IMPVI_MAIN 主摄像头
    ret = IMP_ISP_AddSensor(IMPVI_MAIN, &sensor_info[0]);
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "failed to AddSensor\n");
        return -1;
    }

    /* enable sensor */
    // 使用一个sensor
    ret = IMP_ISP_EnableSensor(IMPVI_MAIN, &sensor_info[0]);
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "failed to EnableSensor\n");
        return -1;
    }

    /* init imp system */
    ret = IMP_System_Init();
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "IMP_System_Init failed\n");
        return -1;
    }

    /* enable turning, to debug graphics */
    ret = IMP_ISP_EnableTuning();
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "IMP_ISP_EnableTuning failed\n");
        return -1;
    }

    /* set contrast, sharpness, saturation, brightness */
    unsigned char value = 128;
    // 我个人理解：以下是给摄像头设置画面效果
    // 设置ISP 综合效果图片对比度
    IMP_ISP_Tuning_SetContrast(IMPVI_MAIN, &value);
    // 设置ISP 综合效果图片锐度
    IMP_ISP_Tuning_SetSharpness(IMPVI_MAIN, &value);
    // 设置ISP 综合效果图片饱和度
    IMP_ISP_Tuning_SetSaturation(IMPVI_MAIN, &value);
    // 设置ISP 综合效果图片亮度
    IMP_ISP_Tuning_SetBrightness(IMPVI_MAIN, &value);

    // 主摄像头：摄像头模式： 白天模式
    /* set runningmode */
    IMPISPRunningMode dn = IMPISP_RUNNING_MODE_DAY;
    ret = IMP_ISP_Tuning_SetISPRunningMode(IMPVI_MAIN, &dn);
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "IMP_ISP_Tuning_SetISPRunningMode failed\n");
        return -1;
    }

    /* set fps */
    // Sensor帧率
    IMPISPSensorFps fpsAttr;
    fpsAttr.num = FIRST_SENSOR_FRAME_RATE_NUM;
    fpsAttr.den = FIRST_SENSOR_FRAME_RATE_DEN;
    ret = IMP_ISP_Tuning_SetSensorFPS(0, &fpsAttr);
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "IMP_ISP_Tuning_SetSensorFPS failed\n");
        return -1;
    }

    IMP_LOG_DBG(TAG, "sample_system_init success\n");

    return 0;
}

int sample_system_exit() {
    IMP_LOG_DBG(TAG, "sample_system_exit start\n");
    PRINT_CURR_FUNC(__FUNCTION__)

    int ret = 0;

    /* exit imp system */
    ret = IMP_System_Exit();
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "IMP_System_Exit failed\n");
        return -1;
    }

    /* disable sensor */
    ret = IMP_ISP_DisableSensor(IMPVI_MAIN);
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "IMP_ISP_DisableSensor failed\n");
        return -1;
    }

    /* delete sensor */
    ret = IMP_ISP_DelSensor(IMPVI_MAIN, &sensor_info[0]);
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "IMP_ISP_DelSensor failed\n");
        return -1;
    }

    /* disable turning */
    ret = IMP_ISP_DisableTuning();
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "IMP_ISP_DisableTuning failed\n");
        return -1;
    }

    /* close isp */
    ret = IMP_ISP_Close();
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "failed to open ISP\n");
        return -1;
    }

    IMP_LOG_DBG(TAG, " sample_system_exit success\n");

    return 0;
}

int sample_framesource_init() {
    IMP_LOG_DBG(TAG, "sample_framesource_init start\n");

    int i, ret;

    // 创建3个通道
    for (i = 0; i < FS_CHN_NUM; i++) {
        if (chn[i].enable) {
            // 创建通道
            ret = IMP_FrameSource_CreateChn(chn[i].index, &chn[i].fs_chn_attr);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "IMP_FrameSource_CreateChn(chn%d) error !\n", chn[i].index);
                return -1;
            }

            // 设置通道属性
            ret = IMP_FrameSource_SetChnAttr(chn[i].index, &chn[i].fs_chn_attr);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "IMP_FrameSource_SetChnAttr(chn%d) error !\n", chn[i].index);
                return -1;
            }
        }
    }

    IMP_LOG_DBG(TAG, "sample_framesource_init success\n");

    return 0;
}

int sample_framesource_exit() {
    IMP_LOG_DBG(TAG, "sample_framesource_exit start\n");

    int i, ret;

    /* destroy framesource channels */
    for (i = 0; i < FS_CHN_NUM; i++) {
        if (chn[i].enable) {
            ret = IMP_FrameSource_DestroyChn(chn[i].index);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "IMP_FrameSource_DestroyChn(%d) error: %d\n", chn[i].index, ret);
                return -1;
            }
        }
    }

    IMP_LOG_DBG(TAG, "sample_framesource_exit success\n");

    return 0;
}

int sample_encoder_init() {
    IMP_LOG_DBG(TAG, "sample_encoder_init() ------------------------------------------------- start\n");


    int i, ret, chnNum = 0;
    int s32picWidth = 0, s32picHeight = 0;
    // 通道属性
    IMPFSChnAttr *imp_chn_attr_tmp;
    // 编码Channel属性
    IMPEncoderChnAttr channel_attr;
    // I2D属性: 图片旋转等
    IMPFSI2DAttr sti2dattr;
    for (i = 0; i < FS_CHN_NUM; i++) {
        if (chn[i].enable) {
            imp_chn_attr_tmp = &chn[i].fs_chn_attr;
            chnNum = chn[i].index;

            memset(&channel_attr, 0, sizeof(IMPEncoderChnAttr));
            memset(&sti2dattr, 0, sizeof(IMPFSI2DAttr));

            ret = IMP_FrameSource_GetI2dAttr(chn[i].index, &sti2dattr);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "IMP_FrameSource_GetI2dAttr(%d) error !\n", chn[i].index);
                return -1;
            }

            if ((1 == sti2dattr.i2d_enable) && //  可用？
                ((sti2dattr.rotate_enable) && // 旋转
                    (sti2dattr.rotate_angle == 90 || sti2dattr.rotate_angle == 270))) {
                /*this depend on your sensor or channels*/
                //s32picWidth = (chn[i].fs_chn_attr.picHeight +15) & (~15);
                //s32picHeight = (chn[i].fs_chn_attr.picWidth +15) & (~15);
                s32picWidth = (chn[i].fs_chn_attr.picHeight); /*this depend on your sensor or channels*/
                s32picHeight = (chn[i].fs_chn_attr.picWidth);
            } else {
                s32picWidth = chn[i].fs_chn_attr.picWidth;
                s32picHeight = chn[i].fs_chn_attr.picHeight;
            }
            // 分辨率缩放比例（ratio） 的逻辑
            float ratio = 1;
            // case 1:当输入分辨率 大于 1280×720 时，ratio > 1（需要增强处理）
            if (((uint64_t) s32picWidth * s32picHeight) > (1280 * 720)) {
                ratio = log10f(((uint64_t) s32picWidth * s32picHeight) / (1280 * 720.0)) + 1;
            } else {
                // case 2: 当输入分辨率 小于 1280×720 时，ratio < 1（需要减弱处理）
                ratio = 1.0 / (log10f((1280 * 720.0) / ((uint64_t) s32picWidth * s32picHeight)) + 1);
            }

            printf("    Resolution: %dx%d, Ratio: %.3f\n", s32picWidth, s32picHeight, ratio);


            ratio = ratio > 0.1 ? ratio : 0.1;

            // 通过ratio得到一个码率
            unsigned int uTargetBitRate = BITRATE_720P_Kbs * ratio;

            printf("    uTargetBitRate: %d.\n", uTargetBitRate);
            printf("    码率控制模型    : %d.\n", S_RC_METHOD);
            // 设置编码默认属性
            ret = IMP_Encoder_SetDefaultParam(&channel_attr,
                                              chn[i].payloadType,
                                              S_RC_METHOD, // bitrate 模式
                                              s32picWidth, s32picHeight,
                                              imp_chn_attr_tmp->outFrmRateNum,
                                              imp_chn_attr_tmp->outFrmRateDen,
                                              imp_chn_attr_tmp->outFrmRateNum * 2 / imp_chn_attr_tmp->outFrmRateDen,
                                              // gop
                                              2,
                                              (S_RC_METHOD == IMP_ENC_RC_MODE_FIXQP) ? 35 : -1,
                                              uTargetBitRate);
            PRINT_CURR_FUNC("   start set video encoder params.")
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "IMP_Encoder_SetDefaultParam(%d) error !\n", chnNum);
                return -1;
            }
#ifdef LOW_BITSTREAM
			IMPEncoderRcAttr *rcAttr = &channel_attr.rcAttr;
			uTargetBitRate /= 2;

			switch (rcAttr->attrRcMode.rcMode) {
				case IMP_ENC_RC_MODE_FIXQP:
					rcAttr->attrRcMode.attrFixQp.iInitialQP = 38;
					break;
				case IMP_ENC_RC_MODE_CBR:
					rcAttr->attrRcMode.attrCbr.uTargetBitRate = uTargetBitRate;
					rcAttr->attrRcMode.attrCbr.iInitialQP = -1;
					rcAttr->attrRcMode.attrCbr.iMinQP = 34;
					rcAttr->attrRcMode.attrCbr.iMaxQP = 51;
					rcAttr->attrRcMode.attrCbr.iIPDelta = -1;
					rcAttr->attrRcMode.attrCbr.iPBDelta = -1;
					rcAttr->attrRcMode.attrCbr.eRcOptions = IMP_ENC_RC_SCN_CHG_RES | IMP_ENC_RC_OPT_SC_PREVENTION;
					rcAttr->attrRcMode.attrCbr.uMaxPictureSize = uTargetBitRate * 4 / 3;
					break;
				case IMP_ENC_RC_MODE_VBR:
					rcAttr->attrRcMode.attrVbr.uTargetBitRate = uTargetBitRate;
					rcAttr->attrRcMode.attrVbr.uMaxBitRate = uTargetBitRate * 4 / 3;
					rcAttr->attrRcMode.attrVbr.iInitialQP = -1;
					rcAttr->attrRcMode.attrVbr.iMinQP = 34;
					rcAttr->attrRcMode.attrVbr.iMaxQP = 51;
					rcAttr->attrRcMode.attrVbr.iIPDelta = -1;
					rcAttr->attrRcMode.attrVbr.iPBDelta = -1;
					rcAttr->attrRcMode.attrVbr.eRcOptions = IMP_ENC_RC_SCN_CHG_RES | IMP_ENC_RC_OPT_SC_PREVENTION;
					rcAttr->attrRcMode.attrVbr.uMaxPictureSize = uTargetBitRate * 4 / 3;
					break;
				case IMP_ENC_RC_MODE_CAPPED_VBR:
					rcAttr->attrRcMode.attrCappedVbr.uTargetBitRate = uTargetBitRate;
					rcAttr->attrRcMode.attrCappedVbr.uMaxBitRate = uTargetBitRate * 4 / 3;
					rcAttr->attrRcMode.attrCappedVbr.iInitialQP = -1;
					rcAttr->attrRcMode.attrCappedVbr.iMinQP = 34;
					rcAttr->attrRcMode.attrCappedVbr.iMaxQP = 51;
					rcAttr->attrRcMode.attrCappedVbr.iIPDelta = -1;
					rcAttr->attrRcMode.attrCappedVbr.iPBDelta = -1;
					rcAttr->attrRcMode.attrCappedVbr.eRcOptions = IMP_ENC_RC_SCN_CHG_RES | IMP_ENC_RC_OPT_SC_PREVENTION;
					rcAttr->attrRcMode.attrCappedVbr.uMaxPictureSize = uTargetBitRate * 4 / 3;
					rcAttr->attrRcMode.attrCappedVbr.uMaxPSNR = 42;
					break;
				case IMP_ENC_RC_MODE_CAPPED_QUALITY:
					rcAttr->attrRcMode.attrCappedQuality.uTargetBitRate = uTargetBitRate;
					rcAttr->attrRcMode.attrCappedQuality.uMaxBitRate = uTargetBitRate * 4 / 3;
					rcAttr->attrRcMode.attrCappedQuality.iInitialQP = -1;
					rcAttr->attrRcMode.attrCappedQuality.iMinQP = 34;
					rcAttr->attrRcMode.attrCappedQuality.iMaxQP = 51;
					rcAttr->attrRcMode.attrCappedQuality.iIPDelta = -1;
					rcAttr->attrRcMode.attrCappedQuality.iPBDelta = -1;
					rcAttr->attrRcMode.attrCappedQuality.eRcOptions = IMP_ENC_RC_SCN_CHG_RES | IMP_ENC_RC_OPT_SC_PREVENTION;
					rcAttr->attrRcMode.attrCappedQuality.uMaxPictureSize = uTargetBitRate * 4 / 3;
					rcAttr->attrRcMode.attrCappedQuality.uMaxPSNR = 42;
					break;
				case IMP_ENC_RC_MODE_INVALID:
					IMP_LOG_ERR(TAG, "unsupported rcmode:%d, we only support fixqp, cbr vbr and capped vbr\n", rcAttr->attrRcMode.rcMode);
					return -1;
			}
#endif
            if (direct_switch == 1) {
                if (0 == chnNum) channel_attr.bEnableIvdc = true;
            }

            // 创建编码Channel
            ret = IMP_Encoder_CreateChn(chnNum, &channel_attr);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "IMP_Encoder_CreateChn(%d) error !\n", chnNum);
                return -1;
            }
            // 注册编码Channel到Group
            ret = IMP_Encoder_RegisterChn(chn[i].index, chnNum);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "IMP_Encoder_RegisterChn(%d, %d) error: %d\n", chn[i].index, chnNum, ret);
                return -1;
            }
        }
    }

    IMP_LOG_DBG(TAG, "sample_encoder_init success\n");

    return 0;
}

int sample_encoder_exit(void) {
    IMP_LOG_DBG(TAG, "sample_encoder_exit start\n");

    int ret = 0, i = 0, chnNum = 0;
    IMPEncoderChnStat chn_stat;

    for (i = 0; i < FS_CHN_NUM; i++) {
        if (chn[i].enable) {
            chnNum = chn[i].index;
            memset(&chn_stat, 0, sizeof(IMPEncoderChnStat));
            ret = IMP_Encoder_Query(chnNum, &chn_stat);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "IMP_Encoder_Query(%d) error: %d\n", chnNum, ret);
                return -1;
            }

            if (chn_stat.registered) {
                ret = IMP_Encoder_UnRegisterChn(chnNum);
                if (ret < 0) {
                    IMP_LOG_ERR(TAG, "IMP_Encoder_UnRegisterChn(%d) error: %d\n", chnNum, ret);
                    return -1;
                }

                ret = IMP_Encoder_DestroyChn(chnNum);
                if (ret < 0) {
                    IMP_LOG_ERR(TAG, "IMP_Encoder_DestroyChn(%d) error: %d\n", chnNum, ret);
                    return -1;
                }

                ret = IMP_Encoder_DestroyGroup(chnNum);
                if (ret < 0) {
                    IMP_LOG_ERR(TAG, "IMP_Encoder_DestroyGroup(%d) error: %d\n", chnNum, ret);
                    return -1;
                }
            }
        }
    }

    IMP_LOG_DBG(TAG, "sample_encoder_exit success\n");

    return 0;
}

int sample_framesource_streamon() {
    PRINT_CURR_FUNC(__FUNCTION__)
    int ret = 0, i = 0;

    /* enable framesource channels */
    for (i = 0; i < FS_CHN_NUM; i++) {
        if (chn[i].enable) {
            IMP_LOG_DBG(TAG, "%s %s \n", "------step 6-1 ------------------------------>","IMP_FrameSource_EnableChn()");
            ret = IMP_FrameSource_EnableChn(chn[i].index);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "IMP_FrameSource_EnableChn failed, error chn index: %d\n", chn[i].index);
                return -1;
            }
        }
    }

    return 0;
}

int sample_framesource_streamoff() {
    int ret = 0, i = 0;

    /* disable framesource channels */
    for (i = 0; i < FS_CHN_NUM; i++) {
        if (chn[i].enable) {
            ret = IMP_FrameSource_DisableChn(chn[i].index);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "IMP_FrameSource_DisableChn failed, error chn index: %d\n", chn[i].index);
                return -1;
            }
        }
    }

    return 0;
}

int sample_jpeg_init() {
    PRINT_CURR_FUNC(__FUNCTION__)
    int i, ret;
    IMPEncoderChnAttr channel_attr;
    IMPFSChnAttr *imp_chn_attr_tmp;

    for (i = 0; i < FS_CHN_NUM; i++) {
        if (chn[i].enable) {
            imp_chn_attr_tmp = &chn[i].fs_chn_attr;
            memset(&channel_attr, 0, sizeof(IMPEncoderChnAttr));
            ret = IMP_Encoder_SetDefaultParam(&channel_attr, IMP_ENC_PROFILE_JPEG, IMP_ENC_RC_MODE_FIXQP,
                                              imp_chn_attr_tmp->picWidth, imp_chn_attr_tmp->picHeight,
                                              imp_chn_attr_tmp->outFrmRateNum, imp_chn_attr_tmp->outFrmRateDen, 0, 0,
                                              25, 0);

            /* Create Channel */
            ret = IMP_Encoder_CreateChn(4 + chn[i].index, &channel_attr);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "IMP_Encoder_CreateChn(%d) error: %d\n",
                            chn[i].index, ret);
                return -1;
            }

            /* Resigter Channel */
            ret = IMP_Encoder_RegisterChn(i, 4 + chn[i].index);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "IMP_Encoder_RegisterChn(0, %d) error: %d\n",
                            chn[i].index, ret);
                return -1;
            }
        }
    }

    return 0;
}

int sample_jpeg_exit() {
    int ret = 0, i = 0, chnNum = 0;
    IMPEncoderChnStat chn_stat;

    for (i = 0; i < FS_CHN_NUM; i++) {
        if (chn[i].enable) {
            chnNum = 4 + chn[i].index;
            memset(&chn_stat, 0, sizeof(IMPEncoderChnStat));
            ret = IMP_Encoder_Query(chnNum, &chn_stat);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "IMP_Encoder_Query(%d) error: %d\n", chnNum, ret);
                return -1;
            }

            if (chn_stat.registered) {
                ret = IMP_Encoder_UnRegisterChn(chnNum);
                if (ret < 0) {
                    IMP_LOG_ERR(TAG, "IMP_Encoder_UnRegisterChn(%d) error: %d\n", chnNum, ret);
                    return -1;
                }

                ret = IMP_Encoder_DestroyChn(chnNum);
                if (ret < 0) {
                    IMP_LOG_ERR(TAG, "IMP_Encoder_DestroyChn(%d) error: %d\n", chnNum, ret);
                    return -1;
                }
            }
        }
    }

    return 0;
}
#include <inttypes.h>
static int save_stream(int fd, IMPEncoderStream *stream) {
    PRINT_CURR_FUNC("MU_SHI_ER-------->save_stream()")
    int ret, i, nr_pack = stream->packCount;

    IMP_LOG_DBG(TAG, "	MU_SHI_ER---------->packCount=%d, stream->seq=%u start----------\n", stream->packCount,
                stream->seq)    ;
    //nr_pack: 一帧码流的所有包的个数
    for (i = 0; i < nr_pack; i++) {
        IMP_LOG_DBG(TAG, "	[%d]:%10u,%10lld,%10u,%10u,%10u\n", i, stream->pack[i].length, stream->pack[i].timestamp,
                    stream->pack[i].frameEnd, *((uint32_t *)(&stream->pack[i].nalType)), stream->pack[i].sliceType)        ;

        // 编码帧，码流数据结构体
        IMPEncoderPack *pack = &stream->pack[i];
        // 码流包长度
        if (pack->length) {
            puts("                                                                  ");
            printf("            |----------------------------------------------------------|\n");
            printf("            | stream_seq      : %d\n",stream->seq);
            printf("            | stream_size     : %d\n",stream->streamSize);
            printf("            | stream_vir_Addr : 0x%" PRIx32 "\n", stream->phyAddr);
            printf("            | pack offset     : 0x%" PRIx32 "\n",stream->pack->offset);
            printf("            | pack length     : %d\n",stream->pack->length);
            printf("            |----------------------------------------------------------|\n");
            puts("                                                                  ");

            // 关键计算1：计算当前包在环形缓冲区中的剩余空间
            // stream->streamSize 是环形缓冲区总大小
            // pack->offset 是当前包在缓冲区中的起始偏移
            // remSize = 缓冲区末尾 - 当前包起始位置
            uint32_t remSize = stream->streamSize - pack->offset;
            // case1：当前包跨越了环形缓冲区末尾（需要分两次写入）
            if (remSize < pack->length) {
                /*******************************************************
                 * 关键写入操作1：写入从offset到缓冲区末尾的数据
                 * stream->virAddr + pack->offset → 数据起始内存地址
                 * remSize → 要写入的数据长度（从offset到缓冲区末尾）
                 *******************************************************/
                ret = write(fd, (void *) (stream->virAddr + pack->offset), remSize);
                if (ret != remSize) {
                    IMP_LOG_ERR(TAG, "	stream write ret(%d) != pack[%d].remSize(%d) error:%s\n", ret, i, remSize,
                                strerror(errno))                    ;
                    return -1;
                }
                /*******************************************************
                * 关键写入操作2：写入剩余部分（从缓冲区头部开始）
                * stream->virAddr → 缓冲区起始地址（绕回到头部）
                * pack->length - remSize → 剩余要写入的数据长度
                *******************************************************/
                ret = write(fd, (void *) stream->virAddr, pack->length - remSize);
                if (ret != (pack->length - remSize)) {
                    IMP_LOG_ERR(
                        TAG, "	stream->virAddr:%x stream write ret(%d) != pack[%d].(length-remSize)(%d) error:%s\n",
                        stream->virAddr, ret, i, (pack->length - remSize), strerror(errno));
                    return -1;
                }
            } else {
                // case 2：当前包完全在环形缓冲区内（一次性写入）
                ret = write(fd, (void *) (stream->virAddr + pack->offset), pack->length);
                if (ret != pack->length) {
                    IMP_LOG_ERR(TAG, "	stream write ret(%d) != pack[%d].length(%d) error:%s\n", ret, i, pack->length,
                                strerror(errno))                    ;
                    return -1;
                }
            }
        }
    }
    IMP_LOG_DBG(TAG, "	MU_SHI_ER---------->packCount=%d, stream->seq=%u end----------\n", stream->packCount,
                stream->seq)    ;
    return 0;
}

static int save_stream_by_name(char *stream_prefix, int idx, IMPEncoderStream *stream) {
    int stream_fd = -1;
    char stream_path[128];
    int ret, i, nr_pack = stream->packCount;

    sprintf(stream_path, "%s-%02d.jpg", stream_prefix, idx);

    IMP_LOG_DBG(TAG, "Open Stream file %s ", stream_path);
    stream_fd = open(stream_path, O_RDWR | O_CREAT | O_TRUNC, 0777);
    if (stream_fd < 0) {
        IMP_LOG_ERR(TAG, "failed: %s\n", strerror(errno));
        return -1;
    }
    IMP_LOG_DBG(TAG, "OK\n");

    for (i = 0; i < nr_pack; i++) {
        IMPEncoderPack *pack = &stream->pack[i];
        if (pack->length) {
            uint32_t remSize = stream->streamSize - pack->offset;
            if (remSize < pack->length) {
                ret = write(stream_fd, (void *) (stream->virAddr + pack->offset),
                            remSize);
                if (ret != remSize) {
                    IMP_LOG_ERR(TAG, "stream write ret(%d) != pack[%d].remSize(%d) error:%s\n", ret, i, remSize,
                                strerror(errno))                    ;
                    return -1;
                }
                ret = write(stream_fd, (void *) stream->virAddr, pack->length - remSize);
                if (ret != (pack->length - remSize)) {
                    IMP_LOG_ERR(TAG, "stream write ret(%d) != pack[%d].(length-remSize)(%d) error:%s\n", ret, i,
                                (pack->length - remSize), strerror(errno))                    ;
                    return -1;
                }
            } else {
                ret = write(stream_fd, (void *) (stream->virAddr + pack->offset), pack->length);
                if (ret != pack->length) {
                    IMP_LOG_ERR(TAG, "stream write ret(%d) != pack[%d].length(%d) error:%s\n", ret, i, pack->length,
                                strerror(errno))                    ;
                    return -1;
                }
            }
        }
    }

    close(stream_fd);

    return 0;
}


static void *get_frame_thread(void *args) {
    puts(__FUNCTION__);
    // channel index
    int index = (int) args;
    int chnNum = chn[index].index;
    int i = 0, ret = 0;
    int fd = -1;
    char framefilename[64];
    IMPFrameInfo *frame = NULL;

    if (PIX_FMT_NV12 == chn[index].fs_chn_attr.pixFmt) {
        sprintf(framefilename, "/tmp/frame%dx%d_%d.nv12", chn[index].fs_chn_attr.picWidth,
                chn[index].fs_chn_attr.picHeight, index);
    } else {
        sprintf(framefilename, "frame%dx%d_%d.raw", chn[index].fs_chn_attr.picWidth, chn[index].fs_chn_attr.picHeight,
                index);
    }

    printf("save_file_name : %s\n", framefilename);
    // 打开文件
    fd = open(framefilename, O_RDWR | O_CREAT, 0x644);
    if (fd < 0) {
        IMP_LOG_ERR(TAG, "open %s failed:%s\n", framefilename, strerror(errno));
        goto err_open_framefilename;
    }

    // 设置可获取的图像最大深度: 保存多少张video buffer
    ret = IMP_FrameSource_SetFrameDepth(chnNum, chn[index].fs_chn_attr.nrVBs * 2);
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "IMP_FrameSource_SetFrameDepth(%d,%d) failed\n", chnNum, chn[index].fs_chn_attr.nrVBs * 2);
        goto err_IMP_FrameSource_SetFrameDepth_1;
    }

    // 不断的从
    for (i = 0; i < NR_FRAMES_TO_SAVE; i++) {
        printf("IMP_FrameSource_GetFrame(%d) i=%d\n", chnNum, i);
        // 此接口可以获取指定通道的视频图像信息。图像信息主要包括：图像的宽度、高度、像素格式以及图片数据起始地址。
        ret = IMP_FrameSource_GetFrame(chnNum, &frame);
        if (ret < 0) {
            IMP_LOG_ERR(TAG, "IMP_FrameSource_GetFrame(%d) i=%d failed\n", chnNum, i);
            goto err_IMP_FrameSource_GetFrame_i;
        }

        printf("	|---------------------------------------------------------------\n");
        printf("	| index           : %d\n", frame->index);
        printf("	| pool_idx        : %d\n", frame->pool_idx);
        printf("	| width           : %d\n", frame->width);
        printf("	| height          : %d\n", frame->height);
        printf("	| pix_fmt         : %d\n", frame->pixfmt);
        printf("	| size            : %d\n", frame->size);
        printf("	| phyAddr         : %d\n", frame->phyAddr);
        printf("	| virAddr         : %d\n", frame->virAddr);
        printf("	| direct_phy_addr : %d\n", frame->direct_phyAddr);
        printf("	| timeStamp       : %lld\n", frame->timeStamp);
        printf("	|---------------------------------------------------------------\n");
        puts("                                                                      ");

        //
        if (NR_FRAMES_TO_SAVE / 2 == i) {
            // virAddr:帧的虚拟地址
            if (write(fd, (void *) frame->virAddr, frame->size) != frame->size) {
                IMP_LOG_ERR(TAG, "chnNum=%d write frame i=%d failed\n", chnNum, i);
                goto err_write_frame;
            }
        }
        // 释放获取的图像
        ret = IMP_FrameSource_ReleaseFrame(chnNum, frame);
        if (ret < 0) {
            IMP_LOG_ERR(TAG, "IMP_FrameSource_ReleaseFrame(%d) i=%d failed\n", chnNum, i);
            goto err_IMP_FrameSource_ReleaseFrame_i;
        }
    }

    IMP_FrameSource_SetFrameDepth(chnNum, 0);
    close(fd);

    return (void *) 0;

err_IMP_FrameSource_ReleaseFrame_i:
err_write_frame:
    IMP_FrameSource_ReleaseFrame(chnNum, frame);
err_IMP_FrameSource_GetFrame_i:
    goto err_IMP_FrameSource_SetFrameDepth_1;
    IMP_FrameSource_SetFrameDepth(chnNum, 0);
err_IMP_FrameSource_SetFrameDepth_1:
    close(fd);
err_open_framefilename:
    return (void *) -1;
}

int sample_get_frame() {
    int ret;
    unsigned int i;
    pthread_t tid[FS_CHN_NUM];

    // 遍历：文件系统的通道数量，开启了3个线程
    for (i = 0; i < FS_CHN_NUM; i++) {
        if (chn[i].enable) {
            ret = pthread_create(&tid[i], NULL, get_frame_thread, (void *) i);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "Create ChnNum%d get_frame_thread failed\n", chn[i].index);
                return -1;
            }
        }
    }

    for (i = 0; i < FS_CHN_NUM; i++) {
        if (chn[i].enable) {
            pthread_join(tid[i],NULL);
        }
    }

    return 0;
}

/**
 * 1. 环形缓冲区原理：
* 【a】stream->virAddr 是缓冲区的起始虚拟地址
* 【b】stream->streamSize 是缓冲区总大小
* 【c】当数据包跨越缓冲区末尾时，需要分两次写入
* 2. 偏移计算
* uint32_t remSize = stream->streamSize - pack->offset;
* 【a】计算从当前偏移(pack->offset)到缓冲区末尾的剩余空间
* 【b】如果pack->length > remSize，说明数据包需要"绕回"缓冲区头部
* 3. 两次写入的场景：
* 【a】write(fd, (void *)(stream->virAddr + pack->offset), remSize);  // 写入第一部分: 从offset到缓冲区末尾
* 【b】write(fd, (void *)stream->virAddr, pack->length - remSize);    // 写入第二部分: 从缓冲区头部开始，写入剩余数据
* 4. 内存地址计算：
* 【a】stream->virAddr + pack->offset：通过基地址+偏移量得到实际内存地址
* 【b】这种计算方式常见于DMA缓冲区或环形缓冲区管理
* 5. 错误处理：
* 【a】每次write()后都检查返回值是否等于预期写入大小
* 【b】如果不匹配则打印错误信息（包含errno对应的错误描述）
 * @param args
 * @return
 */
static void *get_video_stream_thread(void *args) {
    PRINT_CURR_FUNC("MU_SHI_ER-------->get_video_stream_thread()")
    PRINT_CURR_FUNC("	Note : this fun running in the sub thread.")
    int val, i, chnNum, ret;
    char stream_path[64];
    IMPEncoderEncType encType;
    int stream_fd = -1, totalSaveCnt = 0;

    val = (int) args;
    chnNum = val & 0xffff;
    // use encoder type : h264,h265 or jpge...
    encType = (val >> 16) & 0xffff;

    // 开启编码Channel接收图像
    ret = IMP_Encoder_StartRecvPic(chnNum);
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "IMP_Encoder_StartRecvPic(%d) failed\n", chnNum);
        return ((void *) -1);
    }
    PRINT_CURR_FUNC("	MU_SHI_ER-------->IMP_Encoder_StartRecvPic")
    sprintf(stream_path, "%s/stream-chn%d-%dx%d.%s", STREAM_FILE_PATH_PREFIX, chnNum,
            chn[chnNum].fs_chn_attr.picWidth,
            chn[chnNum].fs_chn_attr.picHeight,
            (encType == IMP_ENC_TYPE_AVC) ? "h264" : ((encType == IMP_ENC_TYPE_HEVC) ? "h265" : "jpeg"));
    printf("	MU_SHI_ER-------->stream_path  : %s \n", stream_path);

    if (encType == IMP_ENC_TYPE_JPEG) {
        totalSaveCnt = ((NR_FRAMES_TO_SAVE / 10) > 0) ? (NR_FRAMES_TO_SAVE / 10) : 1;
    } else {
        IMP_LOG_DBG(TAG, "Video ChnNum=%d Open Stream file %s ", chnNum, stream_path);

        stream_fd = open(stream_path, O_RDWR | O_CREAT | O_TRUNC, 0777);
        PRINT_CURR_FUNC("	MU_SHI_ER-------->open file io")
        if (stream_fd < 0) {
            IMP_LOG_ERR(TAG, "	failed: %s\n", strerror(errno));
            return ((void *) -1);
        }
        IMP_LOG_DBG(TAG, "OK\n");
        totalSaveCnt = NR_FRAMES_TO_SAVE;
    }
    printf("	MU_SHI_ER-------->totalSaveCnt : %d\n", totalSaveCnt);

    // 根据：totalSaveCnt ，保存帧的数量，执行获取码流次数，
    for (i = 0; i < totalSaveCnt; i++) {
        // Polling码流缓存
        // 在获取码流之前可以用过此API进行Polling，当码流缓存不为空时或超时时函数返回。
        ret = IMP_Encoder_PollingStream(chnNum, 1000);
        PRINT_CURR_FUNC("	MU_SHI_ER-------->IMP_Encoder_PollingStream()")
        if (ret < 0) {
            IMP_LOG_ERR(TAG, "ERROR_IMP_Encoder_PollingStream(%d) timeout\n", chnNum);
            continue;
        }

        // 编码帧码流类型 [IMPEncoderStream ----> IMPEncoderPack ----> IMPEncoderNalType and IMPEncoderSliceType]
        IMPEncoderStream stream;
        /* Get H264 or H265 Stream */
        // 获取编码的码流
        ret = IMP_Encoder_GetStream(chnNum, &stream, 1);
        PRINT_CURR_FUNC("	MU_SHI_ER-------->IMP_Encoder_GetStream()")

#ifdef SHOW_FRM_BITRATE
        int i, len = 0;
        for (i = 0; i < stream.packCount; i++) {
            len += stream.pack[i].length;
        }
        bitrate_sp[chnNum] += len;
        frmrate_sp[chnNum]++;

        int64_t now = IMP_System_GetTimeStamp() / 1000;
        if (((int) (now - statime_sp[chnNum]) / 1000) >= FRM_BIT_RATE_TIME) {
            double fps = (double) frmrate_sp[chnNum] / ((double) (now - statime_sp[chnNum]) / 1000);
            double kbr = (double) bitrate_sp[chnNum] * 8 / (double) (now - statime_sp[chnNum]);

            printf("streamNum[%d]:FPS: %0.2f,Bitrate: %0.2f(kbps)\n", chnNum, fps, kbr);
            //fflush(stdout);

            frmrate_sp[chnNum] = 0;
            bitrate_sp[chnNum] = 0;
            statime_sp[chnNum] = now;
        }
#endif

        if (ret < 0) {
            IMP_LOG_ERR(TAG, "	IMP_Encoder_GetStream(%d) failed\n", chnNum);
            return ((void *) -1);
        }

        // case 1: jpeg
        if (encType == IMP_ENC_TYPE_JPEG) {
            sprintf(stream_path, "%s/stream-chn%d-%dx%d", STREAM_FILE_PATH_PREFIX, chnNum,
                    chn[chnNum % 4].fs_chn_attr.picWidth,
                    chn[chnNum % 4].fs_chn_attr.picHeight);
            ret = save_stream_by_name(stream_path, i, &stream);
            if (ret < 0) {
                return ((void *) ret);
            }
        } else {
            // case 2: save h264 or h265 bit stream
            ret = save_stream(stream_fd, &stream);
            if (ret < 0) {
                close(stream_fd);
                return ((void *) ret);
            }
        }
        //
        PRINT_CURR_FUNC("	MU_SHI_ER-------->IMP_Encoder_ReleaseStream")
        IMP_Encoder_ReleaseStream(chnNum, &stream);
    }
    close(stream_fd);

    ret = IMP_Encoder_StopRecvPic(chnNum);
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "IMP_Encoder_StopRecvPic(%d) failed\n", chnNum);
        return ((void *) -1);
    }

    return ((void *) 0);
}

/**
 * core-->get_video_stream_thread()
 * @return
 */
int sample_get_video_stream() {
    PRINT_CURR_FUNC(__FUNCTION__)
    unsigned int i;
    int ret;
    pthread_t tid[FS_CHN_NUM];

    for (i = 0; i < FS_CHN_NUM; i++) {
        if (chn[i].enable) {
            int arg = 0;

            // case 1: jpeg
            if (chn[i].payloadType == IMP_ENC_PROFILE_JPEG) {
                arg = (((chn[i].payloadType >> 24) << 16) | (4 + chn[i].index));
            } else {
                // h264 or h265
                arg = (((chn[i].payloadType >> 24) << 16) | chn[i].index);
            }
            ret = pthread_create(&tid[i], NULL, get_video_stream_thread, (void *) arg);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "Create ChnNum%d get_video_stream_thread failed\n",
                            (chn[i].payloadType == IMP_ENC_PROFILE_JPEG) ? (4 + chn[i].index) : chn[i].index);
            }
        }
    }

    for (i = 0; i < FS_CHN_NUM; i++) {
        if (chn[i].enable) {
            pthread_join(tid[i],NULL);
        }
    }

    return 0;
}

int sample_get_video_stream_byfd() {
    int streamFd[FS_CHN_NUM], vencFd[FS_CHN_NUM], maxVencFd = 0;
    char stream_path[FS_CHN_NUM][128];
    fd_set readfds;
    struct timeval selectTimeout;
    int saveStreamCnt[FS_CHN_NUM], totalSaveStreamCnt[FS_CHN_NUM];
    int i = 0, ret = 0, chnNum = 0;
    memset(streamFd, 0, sizeof(streamFd));
    memset(vencFd, 0, sizeof(vencFd));
    memset(stream_path, 0, sizeof(stream_path));
    memset(saveStreamCnt, 0, sizeof(saveStreamCnt));
    memset(totalSaveStreamCnt, 0, sizeof(totalSaveStreamCnt));

    for (i = 0; i < FS_CHN_NUM; i++) {
        streamFd[i] = -1;
        vencFd[i] = -1;
        saveStreamCnt[i] = 0;
        if (chn[i].enable) {
            if (chn[i].payloadType == IMP_ENC_PROFILE_JPEG) {
                chnNum = 4 + chn[i].index;
                totalSaveStreamCnt[i] = (NR_FRAMES_TO_SAVE / 50 > 0) ? NR_FRAMES_TO_SAVE / 50 : NR_FRAMES_TO_SAVE;
            } else {
                chnNum = chn[i].index;
                totalSaveStreamCnt[i] = NR_FRAMES_TO_SAVE;
            }
            sprintf(stream_path[i], "%s/stream-%d.%s", STREAM_FILE_PATH_PREFIX, chnNum,
                    ((chn[i].payloadType >> 24) == IMP_ENC_TYPE_AVC)
                        ? "h264"
                        : (((chn[i].payloadType >> 24) == IMP_ENC_TYPE_HEVC) ? "h265" : "jpeg"));

            if (chn[i].payloadType != IMP_ENC_PROFILE_JPEG) {
                streamFd[i] = open(stream_path[i], O_RDWR | O_CREAT | O_TRUNC, 0777);
                if (streamFd[i] < 0) {
                    IMP_LOG_ERR(TAG, "open %s failed:%s\n", stream_path[i], strerror(errno));
                    return -1;
                }
            }

            vencFd[i] = IMP_Encoder_GetFd(chnNum);
            if (vencFd[i] < 0) {
                IMP_LOG_ERR(TAG, "IMP_Encoder_GetFd(%d) failed\n", chnNum);
                return -1;
            }

            if (maxVencFd < vencFd[i]) {
                maxVencFd = vencFd[i];
            }

            ret = IMP_Encoder_StartRecvPic(chnNum);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "IMP_Encoder_StartRecvPic(%d) failed\n", chnNum);
                return -1;
            }
        }
    }

    while (1) {
        int breakFlag = 1;
        for (i = 0; i < FS_CHN_NUM; i++) {
            breakFlag &= (saveStreamCnt[i] >= totalSaveStreamCnt[i]);
        }
        if (breakFlag) {
            break; // save frame enough
        }

        FD_ZERO(&readfds);
        for (i = 0; i < FS_CHN_NUM; i++) {
            if (chn[i].enable && saveStreamCnt[i] < totalSaveStreamCnt[i]) {
                FD_SET(vencFd[i], &readfds);
            }
        }
        selectTimeout.tv_sec = 2;
        selectTimeout.tv_usec = 0;

        ret = select(maxVencFd + 1, &readfds, NULL, NULL, &selectTimeout);
        if (ret < 0) {
            IMP_LOG_ERR(TAG, "select failed:%s\n", strerror(errno));
            return -1;
        } else if (ret == 0) {
            continue;
        } else {
            for (i = 0; i < FS_CHN_NUM; i++) {
                if (chn[i].enable && FD_ISSET(vencFd[i], &readfds)) {
                    IMPEncoderStream stream;

                    if (chn[i].payloadType == IMP_ENC_PROFILE_JPEG) {
                        chnNum = 4 + chn[i].index;
                    } else {
                        chnNum = chn[i].index;
                    }
                    /* Get H264 or H265 Stream */
                    ret = IMP_Encoder_GetStream(chnNum, &stream, 1);
                    if (ret < 0) {
                        IMP_LOG_ERR(TAG, "IMP_Encoder_GetStream(%d) failed\n", chnNum);
                        return -1;
                    }

                    if (chn[i].payloadType == IMP_ENC_PROFILE_JPEG) {
                        ret = save_stream_by_name(stream_path[i], saveStreamCnt[i], &stream);
                        if (ret < 0) {
                            return -1;
                        }
                    } else {
                        ret = save_stream(streamFd[i], &stream);
                        if (ret < 0) {
                            close(streamFd[i]);
                            return -1;
                        }
                    }

                    IMP_Encoder_ReleaseStream(chnNum, &stream);
                    saveStreamCnt[i]++;
                }
            }
        }
    }

    for (i = 0; i < FS_CHN_NUM; i++) {
        if (chn[i].enable) {
            if (chn[i].payloadType == IMP_ENC_PROFILE_JPEG) {
                chnNum = 4 + chn[i].index;
            } else {
                chnNum = chn[i].index;
            }
            IMP_Encoder_StopRecvPic(chnNum);
            close(streamFd[i]);
        }
    }

    return 0;
}

static void *get_h265_jpeg_stream_thread(void *args) {
    PRINT_CURR_FUNC(__FUNCTION__)
    int val, i, chnNum, ret;
    char stream_path[64];
    IMPEncoderEncType encType;
    int totalSaveCnt = 0;

    val = (int) args;
    chnNum = val & 0xffff;
    encType = (val >> 16) & 0xffff;

    totalSaveCnt = NR_FRAMES_TO_SAVE / 10;

    for (i = 0; i < totalSaveCnt; i++) {
        ret = IMP_Encoder_StartRecvPic(chnNum);
        if (ret < 0) {
            IMP_LOG_ERR(TAG, "IMP_Encoder_StartRecvPic(%d) failed\n", chnNum);
            return ((void *) -1);
        }

        ret = IMP_Encoder_PollingStream(chnNum, 1000);
        if (ret < 0) {
            IMP_LOG_ERR(TAG, "IMP_Encoder_PollingStream(%d) timeout\n", chnNum);
            continue;
        }

        IMPEncoderStream stream;
        /* Get H264 or H265 Stream */
        ret = IMP_Encoder_GetStream(chnNum, &stream, 1);
        if (ret < 0) {
            IMP_LOG_ERR(TAG, "IMP_Encoder_GetStream(%d) failed\n", chnNum);
            return ((void *) -1);
        }

        if (encType == IMP_ENC_TYPE_JPEG) {
            sprintf(stream_path, "%s/stream-chn%d-%dx%d", STREAM_FILE_PATH_PREFIX, chnNum,
                    chn[chnNum % 4].fs_chn_attr.picWidth,
                    chn[chnNum % 4].fs_chn_attr.picHeight);
            ret = save_stream_by_name(stream_path, i, &stream);
            if (ret < 0) {
                return ((void *) ret);
            }
        }
        ret = IMP_Encoder_ReleaseStream(chnNum, &stream);
        if (ret < 0) {
            IMP_LOG_ERR(TAG, "IMP_Encoder_ReleaseStream(%d) failed\n", chnNum);
            return ((void *) -1);
        }

        ret = IMP_Encoder_StopRecvPic(chnNum);
        if (ret < 0) {
            IMP_LOG_ERR(TAG, "IMP_Encoder_StopRecvPic(%d) failed\n", chnNum);
            return ((void *) -1);
        }
    }

    return ((void *) 0);
}

int sample_get_h265_jpeg_stream() {
    PRINT_CURR_FUNC(__FUNCTION__)
    unsigned int i;
    int arg = 0;
    int ret;
    pthread_t tid[FS_CHN_NUM];
    chn[4].payloadType = chn[5].payloadType = IMP_ENC_PROFILE_JPEG;
    for (i = 0; i < FS_CHN_NUM; i++) {
        if (chn[i].enable) {
            arg = (((chn[i].payloadType >> 24) << 16) | chn[i].index);
            ret = pthread_create(&tid[i], NULL, get_video_stream_thread, (void *) arg);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "Create ChnNum%d get_video_stream failed\n",
                            (chn[i].payloadType == IMP_ENC_PROFILE_JPEG) ? (4 + chn[i].index) : chn[i].index);
            }

            if (chn[i + 4].payloadType == IMP_ENC_PROFILE_JPEG) {
                arg = (((chn[i + 4].payloadType >> 24) << 16) | (4 + chn[i].index));
                ret = pthread_create(&tid[i + 4], NULL, get_h265_jpeg_stream_thread, (void *) arg);
                if (ret < 0) {
                    IMP_LOG_ERR(TAG, "Create ChnNum%d get_h265_jpeg_stream_thread failed\n",
                                (chn[i+4].payloadType == IMP_ENC_PROFILE_JPEG) ? (4 + chn[i].index) : chn[i].index);
                }
            }
        }
    }

    for (i = 0; i < FS_CHN_NUM; i++) {
        if (chn[i].enable) {
            pthread_join(tid[i],NULL);
            pthread_join(tid[i + 4],NULL);
        }
    }
    return 0;
}

int sample_jpeg_ivpu_init() {
    PRINT_CURR_FUNC(__FUNCTION__)
    int i, ret;
    IMPEncoderEncAttr *enc_attr;
    IMPEncoderChnAttr channel_attr;
    IMPFSChnAttr *imp_chn_attr_tmp;

    for (i = 0; i < FS_CHN_NUM; i++) {
        if (chn[i].enable) {
            imp_chn_attr_tmp = &chn[i].fs_chn_attr;
            memset(&channel_attr, 0, sizeof(IMPEncoderChnAttr));
            enc_attr = &channel_attr.encAttr;
            enc_attr->eProfile = IMP_ENC_PROFILE_JPEG;
            enc_attr->encVputype = IMP_ENC_TPYE_IVPU;
            enc_attr->bufSize = 0;
            enc_attr->uWidth = imp_chn_attr_tmp->picWidth;
            enc_attr->uHeight = imp_chn_attr_tmp->picHeight;

            /* Create Channel */
            ret = IMP_Encoder_CreateChn(4 + chn[i].index, &channel_attr);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "IMP_Encoder_CreateChn(%d) error: %d\n",
                            chn[i].index, ret);
                return -1;
            }

            /* Resigter Channel */
            ret = IMP_Encoder_RegisterChn(i, 4 + chn[i].index);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "IMP_Encoder_RegisterChn(0, %d) error: %d\n",
                            chn[i].index, ret);
                return -1;
            }
        }
    }

    return 0;
}

int sample_get_jpeg_snap(int count) {
    PRINT_CURR_FUNC(__FUNCTION__)
    int i, ret;
    int j;
    char snap_path[64];

    for (i = 0; i < FS_CHN_NUM; i++) {
        if (chn[i].enable) {
            ret = IMP_Encoder_StartRecvPic(4 + chn[i].index);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "IMP_Encoder_StartRecvPic(%d) failed\n", 4 + chn[i].index);
                return -1;
            }

            for (j = 0; j < count; j++) {
                sprintf(snap_path, "%s/snap-%d-%dx%d-chn%d.jpg",
                        SNAP_FILE_PATH_PREFIX, j, chn[i].fs_chn_attr.picWidth, chn[i].fs_chn_attr.picHeight,
                        4 + chn[i].index);

                IMP_LOG_ERR(TAG, "Open Snap file %s ", snap_path);
                int snap_fd = open(snap_path, O_RDWR | O_CREAT | O_TRUNC, 0777);
                if (snap_fd < 0) {
                    IMP_LOG_ERR(TAG, "failed: %s\n", strerror(errno));
                    return -1;
                }
                IMP_LOG_DBG(TAG, "OK\n");

                /* Polling JPEG Snap, set timeout as 1000msec */
                ret = IMP_Encoder_PollingStream(4 + chn[i].index, 1000);
                if (ret < 0) {
                    IMP_LOG_ERR(TAG, "Polling stream timeout\n");
                    continue;
                }

                IMPEncoderStream stream;
                /* Get JPEG Snap */
                ret = IMP_Encoder_GetStream(chn[i].index + 4, &stream, 1);
                if (ret < 0) {
                    IMP_LOG_ERR(TAG, "IMP_Encoder_GetStream() failed\n");
                    return -1;
                }

                ret = save_stream(snap_fd, &stream);
                if (ret < 0) {
                    close(snap_fd);
                    return ret;
                }

                IMP_Encoder_ReleaseStream(4 + chn[i].index, &stream);

                close(snap_fd);
            }

            ret = IMP_Encoder_StopRecvPic(4 + chn[i].index);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "IMP_Encoder_StopRecvPic() failed\n");
                return -1;
            }
        }
    }
    return 0;
}

int64_t sample_gettimeus(void) {
    PRINT_CURR_FUNC(__FUNCTION__)
    struct timeval sttime;
    gettimeofday(&sttime,NULL);
    return (sttime.tv_sec * 1000000 + (sttime.tv_usec));
}
