CONFIG_UCLIBC_BUILD=n

#CROSS_COMPILE ?= mips-linux-gnu-
CROSS_COMPILE ?=/home/dev/Documents/Android_work/Hai_si_work/hai_si/mips_gcc720_glibc229_r5_1_4/bin/mips-linux-gnu-

CC = $(CROSS_COMPILE)gcc
CPLUSPLUS = $(CROSS_COMPILE)g++
LD = $(CROSS_COMPILE)ld
AR = $(CROSS_COMPILE)ar cr
# 是否包含strip
#STRIP = $(CROSS_COMPILE)strip

CFLAGS = $(INCLUDES) -O2 -Wall -march=mips32r2

ifeq ($(CONFIG_UCLIBC_BUILD), y)
CFLAGS += -muclibc
LDFLAG += -muclibc
endif

ifeq ($(CONFIG_UCLIBC_BUILD), y)
SDK_LIB_DIR = ../../lib/uclibc
else
SDK_LIB_DIR = ../../lib/glibc
endif

SDK_INC_DIR = ../../include

INCLUDES = -I$(SDK_INC_DIR)

LIBS = $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a

LDFLAG += -Wl,-gc-sections

SAMPLES = sample-Encoder-video \
	sample-IspOsd \
	sample-Mu-OSD \
	sample-Audio \
	sample-Ai \
	sample-Ao \
	sample-Ai-AEC \
	sample-Ai-Ref \
	sample-dmic \
	sample-dmic-aec \
	sample-dmic-ref \
	sample-Setfps \
	sample-ISP-flip \
	sample-Encoder-jpeg \
	sample-Encoder-yuv \
	sample-Encoder-jpeg-ivpu \
	sample-Encoder-h265-ivpu-jpeg \
	sample-IVS-move \
	sample-IVS-base-move\
	sample-Change-Resolution \
	sample-Snap-Raw \
	sample-Snap-YUV \
	sample-Framesource \
	sample-LCD \
	sample-AutoZoom \
	sample-Fcrop \
	sample-IVS-unbind-move \
	sample-Soft-photosensitive \
	sample-Encoder-video-direct \
	sample-Encoder-OSD \

all: 	$(SAMPLES)

sample-Ai: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-Ai.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt -ldl
	$(STRIP) $@

sample-Ao: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-Ao.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt -ldl
	$(STRIP) $@

sample-Ai-AEC: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-Ai-AEC.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt -ldl
	$(STRIP) $@

sample-Ai-Ref: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-Ai-Ref.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt -ldl
	$(STRIP) $@

sample-dmic: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-dmic.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt -ldl
	$(STRIP) $@

sample-dmic-aec: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-dmic-aec.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt -ldl
	$(STRIP) $@

sample-dmic-ref: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-dmic-ref.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt -ldl
	$(STRIP) $@
# case 2:
sample-Encoder-video: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-Encoder-video.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@

sample-Encoder-jpeg: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-Encoder-jpeg.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@

sample-Encoder-yuv: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-Encoder-yuv.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@
# 抓 取 helix jpeg 图片
sample-Encoder-jpeg-ivpu: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-Encoder-jpeg-ivpu.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@
# 保存 H265 码流和抓取图片
sample-Encoder-h265-ivpu-jpeg: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-Encoder-h265-ivpu-jpeg.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@

sample-IspOsd: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-IspOsd.o
		$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
			$(STRIP) $@

# 覆盖物OSD
sample-Mu-OSD: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-OSD.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@

sample-Audio: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-Audio.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt -ldl
	$(STRIP) $@
# 设置帧率
sample-Setfps: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-Setfps.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@

sample-ISP-flip: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-ISP-flip.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@

sample-IVS-move: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-IVS-move.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@

sample-IVS-base-move: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-IVS-base-move.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@

sample-IVS-unbind-move: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-IVS-unbind-move.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@
# 改变分辨率流程
sample-Change-Resolution: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-Change-Resolution.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@
# 抓取 raw 图片
sample-Snap-Raw: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-Snap-Raw.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@
# 抓取 yuv 图片
sample-Snap-YUV: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-Snap-YUV.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@
# 保存对应格式的图片
sample-Framesource: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-Framesource.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@

sample-LCD: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-LCD.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@

sample-ISP-Bypass: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-ISP-Bypass.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@

sample-EXTFS-Encoder-h264: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-EXTFS-Encoder-h264.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@

sample-EXTFS-Encoder-h264-bgra: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-EXTFS-Encoder-h264-bgra.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@

sample-Framesource-algo: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-Framesource-algo.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@

sample-AutoZoom: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-AutoZoom.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@

sample-Fcrop: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-Fcrop.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt
	$(STRIP) $@

sample-Soft-photosensitive: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-Soft-photosensitive.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt -ldl
	$(STRIP) $@
# 主码流开启直通功能，并获取直通后的 h265 码流
sample-Encoder-video-direct: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-Encoder-video-direct.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt -ldl
	$(STRIP) $@
# case : Encoder覆盖物
sample-Encoder-OSD: $(SDK_LIB_DIR)/libimp.a $(SDK_LIB_DIR)/libalog.a sample-common.o sample-Encoder-OSD.o
	$(CPLUSPLUS) $(LDFLAG) -o $@ $^ $(LIBS) -lpthread -lm -lrt -ldl
	$(STRIP) $@

%.o:%.c sample-common.h
	$(CC) -c $(CFLAGS) $< -o $@

clean:
	rm -f *.o *~

distclean: clean
	rm -f $(SAMPLES)

#/home/dev/Documents/Android_work/Hai_si_work/hai_si/mips_gcc720_glibc229_r5_1_4/bin
#/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin
