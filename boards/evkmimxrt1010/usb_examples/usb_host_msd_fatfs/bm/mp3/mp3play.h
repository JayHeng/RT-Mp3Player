#ifndef __MP3PLAY_H__ 
#define __MP3PLAY_H__ 

//#include <sys.h>
#include ".\\helix\\mp3dec.h"
#include <stdint.h>


//////////////////////////////////////////////////////////////////////////////////	 
//本程序移植自helix MP3解码库
//ALIENTEK STM32F407开发板
//MP3 解码代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/6/29
//版本：V1.0
//********************************************************************************
//V1.0 说明
//1,支持16位单声道/立体声MP3的解码
//2,支持CBR/VBR格式MP3解码
//3,支持ID3V1和ID3V2标签解析
//4,支持所有比特率(MP3最高是320Kbps)解码
////////////////////////////////////////////////////////////////////////////////// 	
  
#define MP3_TITSIZE_MAX		40		//歌曲名字最大长度
#define MP3_ARTSIZE_MAX		40		//歌曲名字最大长度
#define MP3_FILE_BUF_SZ    5*1024	//MP3解码时,文件buf大小
 

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef  uint32_t  vu32;
typedef  uint16_t vu16;
typedef  uint8_t  vu8;


//ID3V1 标签 
typedef __packed struct 
{
    u8 id[3];		   	//ID,TAG三个字母
    u8 title[30];		//歌曲名字
    u8 artist[30];		//艺术家名字
	u8 year[4];			//年代
	u8 comment[30];		//备注
	u8 genre;			//流派 
}ID3V1_Tag;

//ID3V2 标签头 
typedef __packed struct 
{
    u8 id[3];		   	//ID
    u8 mversion;		//主版本号
    u8 sversion;		//子版本号
    u8 flags;			//标签头标志
    u8 size[4];			//标签信息大小(不包含标签头10字节).所以,标签大小=size+10.
}ID3V2_TagHead;

//ID3V2.3 版本帧头
typedef __packed struct 
{
    u8 id[4];		   	//帧ID
    u8 size[4];			//帧大小
    u16 flags;			//帧标志
}ID3V23_FrameHead;

//MP3 Xing帧信息(没有全部列出来,仅列出有用的部分)
typedef __packed struct 
{
    u8 id[4];		   	//帧ID,为Xing/Info
    u8 flags[4];		//存放标志
    u8 frames[4];		//总帧数
	u8 fsize[4];		//文件总大小(不包含ID3)
}MP3_FrameXing;
 
//MP3 VBRI帧信息(没有全部列出来,仅列出有用的部分)
typedef __packed struct 
{
    u8 id[4];		   	//帧ID,为Xing/Info
	u8 version[2];		//版本号
	u8 delay[2];		//延迟
	u8 quality[2];		//音频质量,0~100,越大质量越好
	u8 fsize[4];		//文件总大小
	u8 frames[4];		//文件总帧数 
}MP3_FrameVBRI;


//MP3控制结构体
typedef __packed struct 
{
    u8 title[MP3_TITSIZE_MAX];	//歌曲名字
    u8 artist[MP3_ARTSIZE_MAX];	//艺术家名字
    u32 totsec ;				//整首歌时长,单位:秒
    u32 cursec ;				//当前播放时长
	
    u32 bitrate;	   			//比特率
	u32 samplerate;				//采样率
	u16 outsamples;				//PCM输出数据量大小(以16位为单位),单声道MP3,则等于实际输出*2(方便DAC输出)
	
	u32 datastart;				//数据帧开始的位置(在文件里面的偏移)
}__mp3ctrl;


void mp3_i2s_dma_tx_callback(void) ;
void mp3_fill_buffer(u16* buf,u16 size,u8 nch);
u8 mp3_id3v1_decode(u8* buf,__mp3ctrl *pctrl);
u8 mp3_id3v2_decode(u8* buf,u32 size,__mp3ctrl *pctrl);
u8 mp3_play_song(u8* fname);
#endif




























