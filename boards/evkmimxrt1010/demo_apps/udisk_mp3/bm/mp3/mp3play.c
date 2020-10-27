#include "mp3play.h"
#include "mp3_config.h"
#include "ff.h"
#include "string.h"
#include "FSL_DEBUG_CONSOLE.h"

#define printf PRINTF
#define AUDIO_MIN(x,y)	((x)<(y)? (x):(y))

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
 

FIL f_wave;
void close_wave_file(void)
{
    f_close(&f_wave);
}
void open_wave_file(void)
{
    FRESULT error;
    error = f_open(&f_wave, _T(PCM_FILEPATH), (FA_WRITE | FA_CREATE_ALWAYS));
    if (error)
    {
        printf("open decoded pcm file fail.\r\n");
        while(1);
    }    
    else
    {
        printf("open decoded pcm file done.\r\n");
    }
}
void mp3_fill_buffer(u16* buf,u16 sample_cnt,u8 nch)
{
    // send to SAI
#if 1  
    void SAI_send_audio(char * buf, uint32_t size);
    //SAI_send_audio(buf, sample_cnt*nch);
#endif

#if 0
    // write to files
    static int size_total = 0;
    size_total += sample_cnt*nch;
	PRINTF("get pcm data: %d, sample_cnt: %d\r\n", size_total, sample_cnt);
    UINT bytesWritten;
    f_write(&f_wave, buf, sample_cnt*nch, &bytesWritten);
#endif
}


//解析ID3V1 
//buf:输入数据缓存区(大小固定是128字节)
//pctrl:MP3控制器
//返回值:0,获取正常
//    其他,获取失败
u8 mp3_id3v1_decode(u8* buf,__mp3ctrl *pctrl)
{
	ID3V1_Tag *id3v1tag;
	id3v1tag=(ID3V1_Tag*)buf;
	if (strncmp("TAG",(char*)id3v1tag->id,3)==0)//是MP3 ID3V1 TAG
	{
		if(id3v1tag->title[0])strncpy((char*)pctrl->title,(char*)id3v1tag->title,30);
		if(id3v1tag->artist[0])strncpy((char*)pctrl->artist,(char*)id3v1tag->artist,30); 
	}else return 1;
	return 0;
}
//解析ID3V2 
//buf:输入数据缓存区
//size:数据大小
//pctrl:MP3控制器
//返回值:0,获取正常
//    其他,获取失败
u8 mp3_id3v2_decode(u8* buf,u32 size,__mp3ctrl *pctrl)
{
	ID3V2_TagHead *taghead;
	ID3V23_FrameHead *framehead; 
	u32 t;
	u32 tagsize;	//tag大小
	u32 frame_size;	//帧大小 
	taghead=(ID3V2_TagHead*)buf; 
	if(strncmp("ID3",(const char*)taghead->id,3)==0)//存在ID3?
	{
		tagsize=((u32)taghead->size[0]<<21)|((u32)taghead->size[1]<<14)|((u16)taghead->size[2]<<7)|taghead->size[3];//得到tag 大小
		pctrl->datastart=tagsize;		//得到mp3数据开始的偏移量
		if(tagsize>size)tagsize=size;	//tagsize大于输入bufsize的时候,只处理输入size大小的数据
		if(taghead->mversion<3)
		{
			printf("not supported mversion!\r\n");
			return 1;
		}
		t=10;
		while(t<tagsize)
		{
			framehead=(ID3V23_FrameHead*)(buf+t);
			frame_size=((u32)framehead->size[0]<<24)|((u32)framehead->size[1]<<16)|((u32)framehead->size[2]<<8)|framehead->size[3];//得到帧大小
 			if (strncmp("TT2",(char*)framehead->id,3)==0||strncmp("TIT2",(char*)framehead->id,4)==0)//找到歌曲标题帧,不支持unicode格式!!
			{
				strncpy((char*)pctrl->title,(char*)(buf+t+sizeof(ID3V23_FrameHead)+1),AUDIO_MIN(frame_size-1,MP3_TITSIZE_MAX-1));
			}
 			if (strncmp("TP1",(char*)framehead->id,3)==0||strncmp("TPE1",(char*)framehead->id,4)==0)//找到歌曲艺术家帧
			{
				strncpy((char*)pctrl->artist,(char*)(buf+t+sizeof(ID3V23_FrameHead)+1),AUDIO_MIN(frame_size-1,MP3_ARTSIZE_MAX-1));
			}
			t+=frame_size+sizeof(ID3V23_FrameHead);
		} 
	}
    else 
        pctrl->datastart=0;//不存在ID3,mp3数据是从0开始
	return 0;
} 

//获取MP3基本信息
//pname:MP3文件路径
//pctrl:MP3控制信息结构体 
//返回值:0,成功
//    其他,失败
u8 mp3_get_info(u8 *pname,__mp3ctrl* pctrl)
{
    HMP3Decoder decoder;
    MP3FrameInfo frame_info;
	MP3_FrameXing* fxing;
	MP3_FrameVBRI* fvbri;
	FIL*fmp3;
	u8 *buf;
	u32 br;
	u8 res;
	int offset=0;
	u32 p;
	short samples_per_frame;	//一帧的采样个数
	u32 totframes;				//总帧数
	
    FIL f;
	fmp3=&f; //mymalloc(SRAMIN,sizeof(FIL)); 
    
    u8 buf_mp3[5*1024];
	buf = buf_mp3; //mymalloc(SRAMIN,5*1024);		//申请5K内存 

	if(fmp3&&buf)//内存申请成功
	{ 		
		res = f_open(fmp3,(MP3_FILEPATH),FA_READ);//打开文件
		res=f_read(fmp3,(char*)buf,5*1024,&br);
		if(res==0)//读取文件成功,开始解析ID3V2/ID3V1以及获取MP3信息
		{  
			mp3_id3v2_decode(buf,br,pctrl);	//解析ID3V2数据
			//f_lseek(fmp3,fmp3->fsize-128);	//偏移到倒数128的位置
            f_lseek(fmp3,f_size(fmp3)-128);	//偏移到倒数128的位置
            
			f_read(fmp3,(char*)buf,128,&br);//读取128字节
			mp3_id3v1_decode(buf,pctrl);	//解析ID3V1数据  
			decoder=MP3InitDecoder(); 		//MP3解码申请内存
			f_lseek(fmp3,pctrl->datastart);	//偏移到数据开始的地方
			f_read(fmp3,(char*)buf,5*1024,&br);	//读取5K字节mp3数据
 			offset=MP3FindSyncWord(buf,br);	//查找帧同步信息
			if(offset>=0&&MP3GetNextFrameInfo(decoder,&frame_info,&buf[offset])==0)//找到帧同步信息了,且下一阵信息获取正常	
			{ 
				p=offset+4+32;
				fvbri=(MP3_FrameVBRI*)(buf+p);
				if(strncmp("VBRI",(char*)fvbri->id,4)==0)//存在VBRI帧(VBR格式)
				{
					if (frame_info.version==MPEG1)samples_per_frame=1152;//MPEG1,layer3每帧采样数等于1152
					else samples_per_frame=576;//MPEG2/MPEG2.5,layer3每帧采样数等于576 
 					totframes=((u32)fvbri->frames[0]<<24)|((u32)fvbri->frames[1]<<16)|((u16)fvbri->frames[2]<<8)|fvbri->frames[3];//得到总帧数
					pctrl->totsec=totframes*samples_per_frame/frame_info.samprate;//得到文件总长度
				}else	//不是VBRI帧,尝试是不是Xing帧(VBR格式)
				{  
					if (frame_info.version==MPEG1)	//MPEG1 
					{
						p=frame_info.nChans==2?32:17;
						samples_per_frame = 1152;	//MPEG1,layer3每帧采样数等于1152
					}else
					{
						p=frame_info.nChans==2?17:9;
						samples_per_frame=576;		//MPEG2/MPEG2.5,layer3每帧采样数等于576
					}
					p+=offset+4;
					fxing=(MP3_FrameXing*)(buf+p);
					if(strncmp("Xing",(char*)fxing->id,4)==0||strncmp("Info",(char*)fxing->id,4)==0)//是Xng帧
					{
						if(fxing->flags[3]&0X01)//存在总frame字段
						{
							totframes=((u32)fxing->frames[0]<<24)|((u32)fxing->frames[1]<<16)|((u16)fxing->frames[2]<<8)|fxing->frames[3];//得到总帧数
							pctrl->totsec=totframes*samples_per_frame/frame_info.samprate;//得到文件总长度
						}
                        else	//不存在总frames字段
						{
							//pctrl->totsec=fmp3->fsize/(frame_info.bitrate/8);
                            pctrl->totsec=f_size(fmp3)/(frame_info.bitrate/8);
						} 
					}
                    else 		//CBR格式,直接计算总播放时间
					{
						//pctrl->totsec=fmp3->fsize/(frame_info.bitrate/8);
                        pctrl->totsec = f_size(fmp3)/(frame_info.bitrate/8);
					}
				} 
				pctrl->bitrate=frame_info.bitrate;			//得到当前帧的码率
				pctrl->samplerate=frame_info.samprate; 	//得到采样率. 
				if(frame_info.nChans==2)pctrl->outsamples=frame_info.outputSamps; //输出PCM数据量大小 
				else pctrl->outsamples=frame_info.outputSamps*2; //输出PCM数据量大小,对于单声道MP3,直接*2,补齐为双声道输出
			}else res=0XFE;//未找到同步帧	
			MP3FreeDecoder(decoder);//释放内存		
		} 
		f_close(fmp3);
	}
    else 
        res=0XFF;
    
	return res;	
}  






FIL audioFile;

#include "gpt.h"
static u8* readptr;	//MP3解码读指针
static int offset=0;	//偏移量
static int bytesleft=0;//buffer还剩余的有效数据
u8 mp3_buf[MP3_FILE_BUF_SZ];
//u8 buft[2304*2];
HMP3Decoder mp3decoder;
__mp3ctrl my_mp3_ctrl;

#define DECODE_END 0
#define DECODE_OK  1

u8 mp3_decode_one_frame(u8 * buf_out)
{
    u8 res; 
    u32 br=0; 
    int err=0; 
    MP3FrameInfo mp3frameinfo;
    
    // PRINTF("mp3_decode_one_frame");

    offset=MP3FindSyncWord(readptr,bytesleft);//在readptr位置,开始查找同步字符
    if(offset<0)
    { 
        //没有找到同步字符，重新load数据块
        readptr=mp3_buf;	// MP3读指针指向buffer
        offset=0;		    // 偏移量为0
        bytesleft=0;
        
        res=f_read(&audioFile,mp3_buf,MP3_FILE_BUF_SZ,&br);//一次读取MP3_FILE_BUF_SZ字节
        if(res)  //读数据出错了
        {
            return DECODE_END;
        }
        if(br==0) //读数为0,说明解码完成了.
        {
            return DECODE_END;
        }

        bytesleft+=br;	//buffer里面有多少有效MP3数据?
        err=0;                  
    }
    else	
    {
        //找到同步字符了
        readptr+=offset;		//MP3读指针偏移到同步字符处.
        bytesleft-=offset;		//buffer里面的有效数据个数,必须减去偏移量
        
        //gp_timer_measure_begin();
        err=MP3Decode(mp3decoder,&readptr,&bytesleft,(short*)buf_out,0);//解码一帧MP3数据
        //int us = gp_timer_measure_end();
        
        // PRINTF("CPU loading: %d\r\n", us/(2304/48000));
        // PRINTF("CPU loading: %d\r\n", us*48000*100/(2304*1000*1000) );
        // PRINTF("CPU loading: %d\r\n", us*48*100/(2304*1000) );

        if(err!=0)
        {
            printf("decode error:%d\r\n",err);
            return DECODE_END;
        }
        else
        {
            MP3GetLastFrameInfo(mp3decoder,&mp3frameinfo);	//得到刚刚解码的MP3帧信息
            if(my_mp3_ctrl.bitrate!=mp3frameinfo.bitrate)	//更新码率
            {
                my_mp3_ctrl.bitrate = mp3frameinfo.bitrate;
            }
            
            
            // ********************************************
            // fill decoded buffer.
            // ********************************************                        
            //mp3_fill_buffer((u16*)buft,mp3frameinfo.outputSamps,mp3frameinfo.nChans);//填充pcm数据 
        }
        
        
        // read new data
        if(bytesleft<MAINBUF_SIZE*2)//当数组内容小于2倍MAINBUF_SIZE的时候,必须补充新的数据进来.
        { 
            memmove(mp3_buf,readptr,bytesleft);//移动readptr所指向的数据到buffer里面,数据量大小为:bytesleft
            f_read(&audioFile,mp3_buf+bytesleft,MP3_FILE_BUF_SZ-bytesleft,&br);//补充余下的数据
            if(br<MP3_FILE_BUF_SZ-bytesleft)
            {
                memset(mp3_buf+bytesleft+br,0,MP3_FILE_BUF_SZ-bytesleft-br); 
            }
            bytesleft=MP3_FILE_BUF_SZ;  
            readptr=mp3_buf; 
        }
    }
    
    return DECODE_OK;
}

void mp3_play_clean(void);

u8 mp3_play_song(u8* fname)
{ 

	u8 res;
	u32 br=0; 
    
	memset(&my_mp3_ctrl,0,sizeof(__mp3ctrl));//数据清零 
        open_wave_file();
	res=mp3_get_info(fname,&my_mp3_ctrl);  
	if(res==0)
	{ 
		printf("     title:%s\r\n",   my_mp3_ctrl.title); 
		printf("    artist:%s\r\n",   my_mp3_ctrl.artist); 
		printf("   bitrate:%dbps\r\n",my_mp3_ctrl.bitrate);	
		printf("samplerate:%d\r\n",   my_mp3_ctrl.samplerate);	
		printf("  totalsec:%d\r\n",   my_mp3_ctrl.totsec); 		
		mp3decoder=MP3InitDecoder(); 					//MP3解码申请内存
		res=f_open(&audioFile,(MP3_FILEPATH),FA_READ);	//打开文件
	}
    else
    {
        printf("get mp3 information error\r\n");
        while(1)
            ;
    }
	if(res==0&&mp3decoder!=0)//打开文件成功
	{ 
		f_lseek(&audioFile,my_mp3_ctrl.datastart);	//跳过文件头中tag信息							//开始播放 
		//while(1) 
		{
          
            // *** now begin to decode MP3 file ***
          
			readptr=mp3_buf;	// MP3读指针指向buffer
			offset=0;		    // 偏移量为0
			bytesleft=0;
            
			res=f_read(&audioFile,mp3_buf,MP3_FILE_BUF_SZ,&br);//一次读取MP3_FILE_BUF_SZ字节
			if(res)  //读数据出错了
			{
				return 0;
			}
			if(br==0) //读数为0,说明解码完成了.
			{
				return 0;
			}

			bytesleft+=br;	//buffer里面有多少有效MP3数据?

/*            
			while(1)//没有出现数据异常(即可否找到帧同步字符)
			{
                if( mp3_decode_one_frame() == DECODE_END)
                {
                    mp3_play_clean();
                    break;
                }
			}  
*/
		}
	}
    return 0;
}


void mp3_play_clean(void)
{
	f_close(&audioFile);
    close_wave_file();
	MP3FreeDecoder(mp3decoder);		//释放内存	
}

















