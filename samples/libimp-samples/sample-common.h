/*
 * sample-common.h
 *
 * Copyright (C) 2014 Ingenic Semiconductor Co.,Ltd
 */

#ifndef __SAMPLE_COMMON_H__
#define __SAMPLE_COMMON_H__

#include <imp/imp_common.h>
#include <imp/imp_osd.h>
#include <imp/imp_framesource.h>
#include <imp/imp_isp.h>
#include <imp/imp_encoder.h>
#include <unistd.h>

#ifdef __cplusplus
#if __cplusplus
extern "C"
{
#endif
#endif /* __cplusplus */

/******************************************** Sensor Attribute Table *********************************************/
/* 		NAME		I2C_ADDR		RESOLUTION		Default_Boot			        							*/
/* 		jxf23		0x40 			1920*1080		0:25fps_dvp 1:15fps_dvp 2:25fps_mipi						*/
/* 		jxf37		0x40 			1920*1080		0:25fps_dvp 1:25fps_mipi 2:	25fps_mipi						*/
/* 		imx327		0x1a 			1920*1080		0:25fps 1:25fps_2dol										*/
/* 		sc430ai		0x30 			2688*1520		0:20fps_mipi 1:30fps_mipi 2:25fps_mipi						*/
/* 		sc500ai		0x30 			2880*1620		0:30fps_mipi											    */
/* 		sc5235		0x30 			2592*1944		0:5fps_mipi													*/
/* 		gc4663		0x29 			2560*1440		0:25fps_mipi 1:30fps_mipi						            */
/* 		sc8238		0x30 			3840*2160		0:15fps 1:30fps 											*/
/******************************************** Sensor Attribute Table *********************************************/

/* first sensor */
#define FIRST_SNESOR_NAME           "gc5603"                        //sensor name (match with snesor driver name)
#define FIRST_I2C_ADDR              0x31                           //sensor i2c address
#define FIRST_I2C_ADAPTER_ID        0                               //sensor controller number used (0/1/2/3)
#define FIRST_SENSOR_WIDTH          2880                            //sensor width
#define FIRST_SENSOR_HEIGHT         1620                            //sensor height
#define FIRST_RST_GPIO              GPIO_PA(18)                     //sensor reset gpio
#define FIRST_PWDN_GPIO             GPIO_PA(19)                     //sensor pwdn gpio
#define FIRST_POWER_GPIO            -1                              //sensor power gpio
#define FIRST_SENSOR_ID             0                               //sensor index
#define FIRST_VIDEO_INTERFACE       IMPISP_SENSOR_VI_MIPI_CSI0      //sensor interface type (dvp/csi0/csi1)
#define FIRST_MCLK                  IMPISP_SENSOR_MCLK0             //sensor clk source (mclk0/mclk1/mclk2)
#define FIRST_DEFAULT_BOOT          0                               //sensor default mode(0/1/2/3/4)

#define CHN0_EN                 1
#define CHN1_EN                 1
#define CHN2_EN                 0

/* Crop_en Choose */
#define FIRST_CROP_EN					0      // 是否启用图像裁剪（0：禁用，1：启用）

// 	作用：配置摄像头传感器的输出帧率为 25fps（25/1 = 25）。
#define FIRST_SENSOR_FRAME_RATE_NUM			25// 帧率分子（25fps）
#define FIRST_SENSOR_FRAME_RATE_DEN			1  // 帧率分母

#define FIRST_SENSOR_WIDTH_SECOND			640// 第二路视频流的宽度
#define FIRST_SENSOR_HEIGHT_SECOND			360// 第二路视频流的高度

	// 支持多路视频流输出（如主码流 1080P + 子码流 720P + 低分辨率流 640x360）。
#define FIRST_SENSOR_WIDTH_THIRD			1280// 第三路视频流的宽度
#define FIRST_SENSOR_HEIGHT_THIRD			720// 第三路视频流的高度

#define BITRATE_720P_Kbs        1000 // 720P 视频的码率（1000 Kbps）
#define NR_FRAMES_TO_SAVE		2000 // 保存的帧数（用于测试）
#define STREAM_BUFFER_SIZE		(1 * 1024 * 1024) // 流缓冲区大小（1MB）,编码器输出流的缓冲区大小。


// 区分视频（H.264/H.265）和 JPEG（抓图）的编码通道。
// 君正芯片通常支持多通道并行编码（如一路录像 + 一路抓图）。
#define ENC_VIDEO_CHANNEL		0 // 视频编码通道号
#define ENC_JPEG_CHANNEL		1 // JPEG 编码通道号

	// 文件路径
#define STREAM_FILE_PATH_PREFIX		"./" // 视频流保存路径
#define SNAP_FILE_PATH_PREFIX		"./" // 抓图保存路径

	// 定义 OSD 叠加层的大小（如显示时间戳、水印等）
	// 不同分辨率可配置不同的 OSD 区域（如主码流和子码流独立配置）。
#define OSD_REGION_WIDTH		16 // OSD 区域的宽度（字符宽度）
#define OSD_REGION_HEIGHT		34 // OSD 区域的高度（字符高度）
#define OSD_REGION_WIDTH_SEC	8  // 第二路 OSD 的宽度
#define OSD_REGION_HEIGHT_SEC   18 // 第二路 OSD 的高度



#define SLEEP_TIME			1 // 循环延迟时间（秒）

#define FS_CHN_NUM			3 // 文件系统的通道数量
#define IVS_CHN_ID          1 // 智能分析（IVS）通道号 ,智能分析（如移动检测、人脸识别）绑定的通道。

#define CH0_INDEX  	0  // 通道 0 的索引
#define CH1_INDEX  	1  // 通道 1 的索引
#define CH2_INDEX  	2  // 通道 2 的索引

#define CHN_ENABLE 		1 // 通道使能标志
#define CHN_DISABLE 	0 // 通道禁用标志

/*#define SUPPORT_RGB555LE*/

#define PRINT_CURR_FUNC(method_name) puts((method_name));

struct chn_conf{
	unsigned int index;            //0 for main channel ,1 for second channel
	unsigned int enable;
	IMPEncoderProfile payloadType; // 画质类型
	IMPFSChnAttr fs_chn_attr;      // 通道属性
	IMPCell framesource_chn;
	IMPCell imp_encoder;
};

typedef struct {
	uint8_t *streamAddr;
	int streamLen;
}streamInfo;

typedef struct {
	IMPEncoderEncType type;// 编码器类型（如 H.264/H.265/JPEG）
	IMPEncoderRcMode mode; // 码率控制模式（固定码率、可变码率等）
	uint16_t frameRate; // 	目标帧率（帧/秒）
	uint16_t gopLength; // GOP（关键帧间隔），即两个 I 帧之间的帧数
	uint32_t targetBitrate; // 目标码率（单位：Kbps）
	uint32_t maxBitrate; // 	最大码率（单位：Kbps，VBR 模式下生效）
	int16_t initQp; // 初始量化参数（QP值，影响画质，值越小画质越高）	26
	int16_t minQp; // 最小量化参数（QP下限）允许的最佳画质
	int16_t maxQp; // 最大量化参数（QP上限） 允许的最低画质
	uint32_t maxPictureSize; // 单帧最大大小（单位：字节，用于限制突发帧体积）
} encInfo;

typedef struct sample_osd_param{
	int *phandles;
	uint32_t *ptimestamps;
}IMP_Sample_OsdParam;

#define  CHN_NUM  ARRAY_SIZE(chn)

int sample_system_init();
int sample_system_exit();

int sample_framesource_init();
int sample_framesource_exit();

int sample_encoder_init();
int sample_encoder_exit();

int sample_framesource_streamon();
int sample_framesource_streamoff();

int sample_jpeg_init();
int sample_jpeg_exit();

int sample_get_frame();
int sample_get_video_stream();
int sample_get_video_stream_byfd();
int sample_get_h265_jpeg_stream();

int sample_jpeg_ivpu_init();

int sample_get_jpeg_snap(int count);

int64_t sample_gettimeus(void);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif /* __SAMPLE_COMMON_H__ */
