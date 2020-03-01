#include "rw_udisk.h"
#include "tlsf.h"

_udiskfs_t udiskfs;

// mount/unmount udisk
void MountUDISK(u8 src)
{
	FRESULT res;	
	
	switch(src)	{
		case APPLICATION_READY:
			{
				BSP_PRINTF("UDISK connected");
				res = f_mount(&udiskfs.fs,USBHPath,1);
				if(res != FR_OK)	{
					BSP_PRINTF("mount udisk failed");
				}
				else {					
//					FILINFO fn;
					DIR dir;
					BSP_PRINTF("mount udisk ok");
					res = f_opendir(&dir,USBHPath);
					if(res==FR_OK)	{
//						for(;;)	{
//							res=f_readdir(&dir,&fn);
//							if(res!=FR_OK||fn.fname[0]==0)	break;
//							BSP_PRINTF("udisk filename: %s",fn.fname);
//						}
						f_closedir(&dir);
						udiskfs.flag |= UDISKFLAG_MOUNTED;
					}
				}
				break;
			}
		case APPLICATION_DISCONNECT:
			{
				BSP_PRINTF("UDISK disconnect");
				res=f_mount(NULL,USBHPath,1);//(NULL:unmount)
				if(res != FR_OK)	{
					BSP_PRINTF("unmount udisk failed");
				}else	{
					BSP_PRINTF("unmount udisk ok");
				}
				udiskfs.flag &= ~UDISKFLAG_MOUNTED;
				break;
			}
	}
}
#define	 UDISK_COPY_SIZE		512
static 	FIL srcfile, destfile;
u8 CopyFile(char *psrc_path, char *pdest_path)
{
	u8 ret=0;
	FRESULT res;
	char *pbuf;
	UINT rsize,wsize;
	//�򿪵�һ���ļ�����flash�Ѵ��ڵ��ļ�
	res = f_open(&srcfile,psrc_path, FA_OPEN_EXISTING|FA_READ);//��֪�����ı����Ƕ����ƣ�����ͳһ�ö�����
	if(res!=FR_OK) {
		return 0;
	}
	//�򿪵ڶ����ļ��������ڵ��ļ�
	res =  f_open(&destfile,pdest_path, FA_CREATE_ALWAYS|FA_READ|FA_WRITE);//�Զ����ƴ򿪣�������a��ʽ
	if(res!=FR_OK) {
		f_close(&srcfile);//����Ŀ���ļ�����ʱ��ԭʼ�ļ��Ѿ����ˣ������ֱ�ӽ�����������Ӧ�ý���ԭʼ�ļ�
		return 0;
	}
	pbuf = (char *)tlsf_malloc(UserMem, UDISK_COPY_SIZE);
	for(;;) {
		res =  f_read(&srcfile, pbuf, UDISK_COPY_SIZE, &rsize);//һ�δ洢100�ֽ�������������ԭĿ��С��һ�£������ڴ��ʱ����size	
		if(res != FR_OK||rsize==0)	break;
		res =  f_write(&destfile, pbuf, rsize, &wsize);
		if(res != FR_OK)	break;
	}
	tlsf_free(UserMem,pbuf);
	pbuf = NULL;
	if(f_size(&srcfile)==f_size(&destfile))	{//�жϴ�С�Ƿ�һ�� �����Ƿ�ɹ�
		BSP_PRINTF("copy %s ok", pdest_path);
		ret = 1;
	}	
	//f_sync(&destfile);
	f_close(&srcfile);
	f_close(&destfile);
	return ret;
}

//�����ļ� ��srcfile -- spi flash ������ destfile -- u��
u8 CopyFileToUDISK(char *psrc_name, char *pdest_name)
{
//	FRESULT res;     /* FatFs return code */		
	char src_name[FILE_NAME_LEN],dest_name[FILE_NAME_LEN];
	
	sprintf(dest_name, "%s%s", USBHPath, pdest_name);//u���豸·��
	sprintf(src_name, "%s%s", USERPath, psrc_name);//flash����·��
	return CopyFile(src_name, dest_name);
}

u8 CopyFileToSpiflash(char *psrc_name, char *pdest_name)
{
//	FRESULT res;     /* FatFs return code */		
	char src_name[FILE_NAME_LEN],dest_name[FILE_NAME_LEN];
	
	sprintf(dest_name, "%s%s", USERPath, psrc_name);//flash����·��
	sprintf(src_name, "%s%s", USBHPath, pdest_name);//u���豸·��
	return CopyFile(src_name, dest_name);
}

