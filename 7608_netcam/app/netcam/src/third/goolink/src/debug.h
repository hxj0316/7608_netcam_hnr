#ifndef _DEBUG_H_
#define _DEBUG_H_

/*
\e �� \033 �����Esc����
������ɫ�ĸ�ʽ�� \e[����ɫ;ǰ��ɫ;����m
\033[����ɫ;ǰ��ɫ;����m
�ָ�Ĭ��Ϊ       \e[0m
���б���ɫ���Ա����������滻

��һ��������
0 ͸����ʹ���ն���ɫ��,1 ���� 40 ��, 41 ��, 42 ��, 43 ��, 44 �� 45 ��, 46 �� ; ����ѡ������� 1 ����
��, 47�ף��ң�

�ڶ���������
ǰ��ɫ��Ҳ�����ı�����ɫ�����Ա����������滻
30 �� 31 ��, 32 ��, 33 ��, 34 ��, 35 ��, 36 ����, 37 �ף��ң�

������������
������1����������0

���ĸ�����Ϊm:
ע��m��������ַ�����

echo -e "\e[1;30m skyapp exist \e[0m" //��ɫ
echo -e "\e[1;31m skyapp exist \e[0m" //��ɫ
echo -e "\e[1;32m skyapp exist \e[0m" //��ɫ
echo -e "\e[1;33m skyapp exist \e[0m" //��ɫ
echo -e "\e[1;34m skyapp exist \e[0m" //��ɫ
echo -e "\e[1;35m skyapp exist \e[0m" //��ɫ
echo -e "\e[1;36m skyapp exist \e[0m" //����
echo -e "\e[1;37m skyapp exist \e[0m" //�ף��ң�
*/

#define NONE                 "\e[0m"
#define BLACK                "\e[0;30m"
#define L_BLACK              "\e[1;30m"
#define RED                  "\e[0;31m"
#define L_RED                "\e[1;31m"
#define GREEN                "\e[0;32m"
#define L_GREEN              "\e[1;32m"
#define BROWN                "\e[0;33m"
#define YELLOW               "\e[1;33m"
#define BLUE                 "\e[0;34m"
#define L_BLUE               "\e[1;34m"
#define PURPLE               "\e[0;35m"
#define L_PURPLE             "\e[1;35m" //1��ʾ����, �������BOLDЧ����һ����
#define CYAN                 "\e[0;36m"
#define L_CYAN               "\e[1;36m"
#define GRAY                 "\e[0;37m"
#define WHITE                "\e[1;37m"

#define COLOR_TAIL           "\e[0m" //�ָ�

#define BOLD                 "\e[1m"
#define UNDERLINE            "\e[4m"
#define BLINK                "\e[5m"
#define REVERSE              "\e[7m"
#define HIDE                 "\e[8m"
#define CLEAR                "\e[2J"
#define CLRLINE              "\r\e[K" //or "\e[1K\r"

//������glnk��صĴ�ӡ
#define __GLNK_DEBUG__  
#ifdef  __GLNK_DEBUG__  
#define GDEBUG(format,...) printf(YELLOW  "%s:%03d " format "\n" COLOR_TAIL, __FUNCTION__,  __LINE__, ##__VA_ARGS__)  
#else  
#define GDEBUG(format,...)  
#endif  

//������cc�Ĵ�ӡ
#define __DEBUG__  
#ifdef  __DEBUG__
#define DEBUG(format,...) printf(L_PURPLE "Fnx_CCProcess, %s:%03d " format "\n" COLOR_TAIL, __FUNCTION__,  __LINE__, ##__VA_ARGS__)  
#else  
#define DEBUG(format,...)  
#endif  

#endif