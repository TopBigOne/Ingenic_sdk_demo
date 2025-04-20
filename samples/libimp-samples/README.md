
# 串口文件通信
* get
```shell
tftp -g -r sample-Mu-OSD 192.168.0.196
```


* push 
```shell

tftp -p -l stream-chn0-2880x1620.h264 192.168.0.196

```

* 启动tftp 
```shell

sudo systemctl restart tftpd-hpa

```

* 启动 minicom
```shell
sudo minicom -D /dev/ttyUSB0 -b 115200
```

* copy
```shell
rm -f /home/dev/Desktop/for_tftp/sample-Mu-OSD
rm -f /home/dev/Desktop/for_tftp/stream-chn0-2880x1620.h264
cp -a /home/dev/Documents/Android_work/Hai_si_work/hai_si/Ingenic_sdk_demo/samples/libimp-samples/sample-Mu-OSD /home/dev/Desktop/for_tftp/sample-Mu-OSD
```



* play with ffplay 
```shell

/home/dev/Documents/Android_work/main_ffmpeg/FFmpeg/ffplay_g -i /home/dev/Desktop/for_tftp/stream-chn0-2880x1620.h264
```




----------------------------------------------
```c





```









