#include <stdio.h>

/*****************************************************************************
 * ��һ���ַ���UTF8����ת����Unicode(UCS-2��UCS-4)����.
 *
 * ����:
 *    pInput      ָ�����뻺����, ��UTF-8����
 *    Unic        ָ�����������, �䱣������ݼ���Unicode����ֵ,
 *                ����Ϊunsigned long .
 *
 * ����ֵ:
 *    �ɹ��򷵻ظ��ַ���UTF8������ռ�õ��ֽ���; ʧ���򷵻�0.
 *
 * ע��:
 *     1. UTF8û���ֽ�������, ����Unicode���ֽ���Ҫ��;
 *        �ֽ����Ϊ���(Big Endian)��С��(Little Endian)����;
 *        ��Intel�������в���С�˷���ʾ, �ڴ˲���С�˷���ʾ. (�͵�ַ���λ)
 ****************************************************************************/
 
/*
   |  Unicode���ŷ�Χ      |  UTF-8���뷽ʽ
 n |  (ʮ������)           | (������)
---+-----------------------+------------------------------------------------------
 1 | 0000 0000 - 0000 007F |                                              0xxxxxxx
 2 | 0000 0080 - 0000 07FF |                                     110xxxxx 10xxxxxx
 3 | 0000 0800 - 0000 FFFF |                            1110xxxx 10xxxxxx 10xxxxxx
 4 | 0001 0000 - 0010 FFFF |                   11110xxx 10xxxxxx 10xxxxxx 10xxxxxx
 5 | 0020 0000 - 03FF FFFF |          111110xx 10xxxxxx 10xxxxxx 10xxxxxx 10xxxxxx
 6 | 0400 0000 - 7FFF FFFF | 1111110x 10xxxxxx 10xxxxxx 10xxxxxx 10xxxxxx 10xxxxxx
 
                    �� 1. UTF-8�ı������
  */ 
static int enc_get_utf8_size(const char* utf8)
{
	int i =0,ret =0;
	if(NULL == utf8)
		return 0;
	for(i = 0; i<8; i++)
	{
		if(((*utf8)&(0x80>>i)))
			ret++;
		else
		{
            if(0==i)
                ret = 1;
			break;
		}
	}
	return ret;
}

int enc_utf8_to_unicode(const char* utf8, unsigned long *Unic)
{
	//  assert(pInput != NULL && Unic != NULL);
    // b1 ��ʾUTF-8�����pInput�еĸ��ֽ�, b2 ��ʾ�θ��ֽ�, ...
    char b1, b2, b3, b4, b5, b6;
    char uncstr[4];
    char *pOutput = uncstr;
    int utfbytes = enc_get_utf8_size(utf8);
    *Unic = 0x0; // �� *Unic ��ʼ�Ϊȫ�?
   
    switch (utfbytes)
    {
        case 1:
            *Unic = *utf8;
            break;
        case 2:
            b1 = *utf8;
            b2 = *(utf8 + 1);
            if ( (b2 & 0xE0) != 0x80 )
                return 0;
            *pOutput     = (b1 << 6) + (b2 & 0x3F);
			*(pOutput+1) = (b1 >> 2) & 0x07;
			*Unic = *(pOutput+1) <<8|*pOutput;
            break;
        case 3:
            b1 = *utf8;
            b2 = *(utf8 + 1);
            b3 = *(utf8 + 2);
            if ( ((b2 & 0xC0) != 0x80) || ((b3 & 0xC0) != 0x80) )
                return 0;
            *pOutput     = (b2 << 6) + (b3 & 0x3F);
            *(pOutput+1) = (b1 << 4) + ((b2 >> 2) & 0x0F);
            *Unic = *(pOutput+1) <<8|*pOutput;
            break;
        case 4:
            b1 = *utf8;
            b2 = *(utf8 + 1);
            b3 = *(utf8 + 2);
            b4 = *(utf8 + 3);
            if ( ((b2 & 0xC0) != 0x80) || ((b3 & 0xC0) != 0x80) || ((b4 & 0xC0) != 0x80) )
                return 0;
            *pOutput     = (b3 << 6) + (b4 & 0x3F);
			*(pOutput+1) = (b2 << 4) + ((b3 >> 2) & 0x0F);
			*(pOutput+2) = ((b1 << 2) & 0x1C)  + ((b2 >> 4) & 0x03);
			*Unic = *(pOutput+2) <<16|*(pOutput+1) <<8|*pOutput;
            break;
        case 5:
            b1 = *utf8;
            b2 = *(utf8 + 1);
            b3 = *(utf8 + 2);
            b4 = *(utf8 + 3);
            b5 = *(utf8 + 4);
            if ( ((b2 & 0xC0) != 0x80) || ((b3 & 0xC0) != 0x80)
            || ((b4 & 0xC0) != 0x80) || ((b5 & 0xC0) != 0x80) )
                return 0;
            *pOutput     = (b4 << 6) + (b5 & 0x3F);
			*(pOutput+1) = (b3 << 4) + ((b4 >> 2) & 0x0F);
			*(pOutput+2) = (b2 << 2) + ((b3 >> 4) & 0x03);
			*(pOutput+3) = (b1 << 6);
			*Unic = (*(pOutput+3)<<24)|(*(pOutput+2) <<16)|*(pOutput+1) <<8|*pOutput;
            break;
        case 6:
            b1 = *utf8;
            b2 = *(utf8 + 1);
            b3 = *(utf8 + 2);
            b4 = *(utf8 + 3);
            b5 = *(utf8 + 4);
            b6 = *(utf8 + 5);
            if ( ((b2 & 0xC0) != 0x80) || ((b3 & 0xC0) != 0x80)
                || ((b4 & 0xC0) != 0x80) || ((b5 & 0xC0) != 0x80)
                || ((b6 & 0xC0) != 0x80) )
                return 0;
            *pOutput     = (b5 << 6) + (b6 & 0x3F);
			*(pOutput+1) = (b5 << 4) + ((b6 >> 2) & 0x0F);
			*(pOutput+2) = (b3 << 2) + ((b4 >> 4) & 0x03);
			*(pOutput+3) = ((b1 << 6) & 0x40) + (b2 & 0x3F);
			*Unic = (*(pOutput+3)<<24)|(*(pOutput+2) <<16)|(*(pOutput+1) <<8)|*pOutput;
            break;
        default:
            return 0;
            break;
    }
    return utfbytes;
}
