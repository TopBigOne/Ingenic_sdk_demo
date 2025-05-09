/*
 * sample-Setfps.c
 *
 * Copyright (C) 2014 Ingenic Semiconductor Co.,Ltd
 */

#include <imp/imp_log.h>
#include <imp/imp_common.h>
#include <imp/imp_system.h>
#include <imp/imp_framesource.h>
#include <imp/imp_encoder.h>

#include "sample-common.h"


#define TAG "Sample-Setfps"

extern struct chn_conf chn[];

/**
 * invoke sample_get_video_stream() where in [sample-common.c]
 * @param m
 * @return
 */
static void *h264_stream_thread(void *m) {
    int ret;
    ret = sample_get_video_stream();
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "Get H264 stream failed\n");
    }

    return NULL;
}

/**
 * 设置摄像头帧率
 * @param argc params count
 * @param argv  real params content
 * @return
 */
int main(int argc, char *argv[]) {
    int i, ret;
    IMPISPSensorFps fps;

    /* Step.1 System init */
    ret = sample_system_init();
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "IMP_System_Init() failed\n");
        return -1;
    }

    /* Step.2 FrameSource init */
    /* frame per second */
    fps.num = 25;
    fps.den = 1;
    // 设置摄像头输出帧率:
    // note : 在使用这个函数之前，必须保证IMP_ISP_EnableSensor 和 IMP_ISP_EnableTuning 已被调用。
    IMP_ISP_Tuning_SetSensorFPS(0, &fps);

    ret = sample_framesource_init();
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "FrameSource init failed\n");
        return -1;
    }

    for (i = 0; i < FS_CHN_NUM; i++) {
        if (chn[i].enable) {
            ret = IMP_Encoder_CreateGroup(chn[i].index);
            if (ret < 0) {
                printf("%s %d\n","IMP_Encoder_CreateGroup() index : ",chn[i].index);
                IMP_LOG_ERR(TAG, "IMP_Encoder_CreateGroup(%d) error !\n", i);
                return -1;
            }
        }
    }

    /* Step.3 Encoder init */
    ret = sample_encoder_init();
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "Encoder init failed\n");
        return -1;
    }

    /* Step.4 Bind */
    for (i = 0; i < FS_CHN_NUM; i++) {
        if (chn[i].enable) {
            ret = IMP_System_Bind(&chn[i].framesource_chn, &chn[i].imp_encoder);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "Bind FrameSource channel%d and Encoder failed\n", i);
                return -1;
            }
        }
    }

    /* Step.5 Stream On */
    ret = sample_framesource_streamon();
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "ImpStreamOn failed\n");
        return -1;
    }

    /* Step.6 Get stream */
    pthread_t h264_stream_thread_tid; /* Stream capture in another thread */
    ret = pthread_create(&h264_stream_thread_tid, NULL, h264_stream_thread, NULL);
    if (ret) {
        IMP_LOG_ERR(TAG, "h264 stream create error\n");
        return -1;
    }

    /* keep fps=25 1second */
    sleep(1);

    // 注意，这个时候，切换帧率
    fps.num = 20;
    fps.den = 1;
    IMP_ISP_Tuning_SetSensorFPS(0, &fps);

    /* Exit sequence as follow */
    pthread_join(h264_stream_thread_tid, NULL);

    /* Step.a Stream Off */
    ret = sample_framesource_streamoff();
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "FrameSource StreamOff failed\n");
        return -1;
    }

    //----------------------流数据获取完毕-------------------------------------------
    /* Step.b UnBind */
    for (i = 0; i < FS_CHN_NUM; i++) {
        if (chn[i].enable) {
            ret = IMP_System_UnBind(&chn[i].framesource_chn, &chn[i].imp_encoder);
            if (ret < 0) {
                IMP_LOG_ERR(TAG, "UnBind FrameSource channel%d and Encoder failed\n", i);
                return -1;
            }
        }
    }

    /* Step.c Encoder exit */
    ret = sample_encoder_exit();
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "Encoder exit failed\n");
        return -1;
    }

    /* Step.d FrameSource exit */
    ret = sample_framesource_exit();
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "FrameSource exit failed\n");
        return -1;
    }

    /* Step.e System exit */
    ret = sample_system_exit();
    if (ret < 0) {
        IMP_LOG_ERR(TAG, "sample_system_exit() failed\n");
        return -1;
    }

    return 0;
}
