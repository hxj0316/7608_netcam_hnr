
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <ctype.h>
#include <unistd.h>
#include <pthread.h>

#include "sdk_def.h"
#include "netcam_api.h"
#include "net_dhcp.h"
#include "net_arp.h"
#include "net_utils.h"
#include "sdk_network.h"
#include "sdk_debug.h"



/*
typedef struct device_ip
{
	char ip[16];
}dev_ip_t;
*/
int g_ip_conflict_flag = 0;

static void *arp_check_ip_proc(void *param)
{
    sdk_sys_thread_set_name("arp_check_ip_proc");
	char *sz_query_ip = (char *)param;
	char query_mac[32][18];
	int *ret;
	ret = (int *)malloc(sizeof(int));
	//*ret= ARP_query(sz_query_ip, query_mac);
	*ret= ARP_send_request(sz_query_ip, query_mac, 32, 2);
	if (*ret < 0) {
		printf("arp query ip: %s failed", sz_query_ip);
	} else if (*ret == 0) {
		printf("arp query ip: %s not found", sz_query_ip);
	} else {
		printf("arp query ip: %s found : %d(%s)", sz_query_ip, *ret, query_mac[0]);
	}
	pthread_exit(ret);
}




static void multi_work_start(int nworks, int *result, pthread_t *pid, void *(*thread_proc)(void *), void **param_list)
{
#define MAX_WORK			(128)
#define DEFAULT_WORK_NUM	(32)
	int i;

	if (nworks == 0)
		nworks = DEFAULT_WORK_NUM;
	else if (nworks < 0 || nworks > MAX_WORK) {
		nworks = MAX_WORK;
	}

	for ( i  = 0; i < nworks; i++) {
		if (pthread_create(&pid[i], NULL, thread_proc, param_list[i]) == 0) {
			result[i] = 0;
		} else {
			result[i] = -1;
		}
	}
}

static void multi_work_stop(int nworks, int *result, pthread_t *pid)
{
	int i;
	int *ret;
	for ( i  = 0; i < nworks; i++) {
		if (result[i] == 0) {
			if (pthread_join(pid[i], (void *)&ret) == 0) {
				result[i] = *ret;
				free(ret);
			} else {
				result[i] = -1;
			}
		}
	}
}

static int check_ip_conflict(char *ip)
{
	int result = 1;
	pthread_t pid = 0;
	int *ret;
	if(ip){
		if(pthread_create(&pid, NULL, arp_check_ip_proc, (void *)ip) == 0){
			if (pthread_join(pid, (void *)&ret) == 0){
				result = *ret;
				free(ret);
			}
		}
	}
	return result;
}

void *get_ip_conflict_proc(void *args)
{
    sdk_sys_thread_set_name("get_ip_conflict_proc");
	char ip_addr[32] = {0};
	char mac_addr[36] = {0};

	pthread_detach(pthread_self());
	if(net_utils_get_ifaddr(LAN, ip_addr, mac_addr))
		pthread_exit(0);

	g_ip_conflict_flag = check_ip_conflict(ip_addr);
	printf("IP conflict:%d\r\n", g_ip_conflict_flag);
	pthread_exit(0);
}

void NET_get_ip_conflict(void)
{
	ST_SDK_NETWORK_ATTR n_interface;
	netcam_net_get(&n_interface);
	if(n_interface.dhcp != 0){
		pthread_t pid;
		pthread_create(&pid, NULL, get_ip_conflict_proc, NULL);
	}
}


static int NET_find_avai_ip(char *sz_l_ip, char *sz_l_netmask, char *ip_start, char *ip_end,
	int need_n, int multi_tasks, int timeout_s, f_ip_found hook)
{
#define NUM_IP_CHECK_TRHEAD		(16)
#define IP_FOUND_DEF_TIMEOUT		(3)
#define DEF_SEARCH_TIMES			(5)
	int n_found = 0;
	int n_threads;
	int irevert=0;
	bool trigger = true;
	unsigned char query_ip[4];
	unsigned char local_ip[4];
	unsigned char u_start[4], u_end[4];
	unsigned char network[4], net_tmp[4];
	unsigned int i_network, i_mask, i_queryip, i_start, i_end;
	unsigned char netmask[4];
	char sz_query_ip[18];
//	char query_mac[128][18];
	int index;
	time_t __tt;
	int i;
	pthread_t check_ip_pids[NUM_IP_CHECK_TRHEAD];
	int check_ip_result[NUM_IP_CHECK_TRHEAD];
	char *check_ip_args[NUM_IP_CHECK_TRHEAD];

	//pthread_detach(pthread_self());

	if (multi_tasks > 0 && multi_tasks < NUM_IP_CHECK_TRHEAD) {
		n_threads =  multi_tasks;
	} else
		n_threads = NUM_IP_CHECK_TRHEAD;

	if (timeout_s == 0)
		timeout_s = (need_n + multi_tasks - 1)/multi_tasks * DEF_SEARCH_TIMES;
	else if (timeout_s <= IP_FOUND_DEF_TIMEOUT) {
		timeout_s = IP_FOUND_DEF_TIMEOUT;
	}

	memset(check_ip_args, 0, sizeof(check_ip_args));
	net_utils_ipstr2byte(local_ip, sz_l_ip);
	net_utils_ipstr2byte(netmask, sz_l_netmask);

	network[0] = local_ip[0] & netmask[0];
	network[1] = local_ip[1] & netmask[1];
	network[2] = local_ip[2] & netmask[2];
	network[3] = local_ip[3] & netmask[3];
	i_network =  (network[0] << 24) | (network[1] << 16) | (network[2] << 8) | network[3];
	i_mask =  (netmask[0] << 24) | (netmask[1] << 16) | (netmask[2] << 8) | netmask[3];
	if (ip_start && ip_end) {
		net_utils_ipstr2byte(u_start, ip_start);
		net_utils_ipstr2byte(u_end, ip_end);
		i_start =  (u_start[0] << 24) | (u_start[1] << 16) | (u_start[2] << 8) | u_start[3];
		i_end =  (u_end[0] << 24) | (u_end[1] << 16) | (u_end[2] << 8) | u_end[3];
		net_tmp[0] = u_start[0] & netmask[0];
		net_tmp[1] = u_start[1] & netmask[1];
		net_tmp[2] = u_start[2] & netmask[2];
		net_tmp[3] = u_start[3] & netmask[3];
		if (i_network != ((net_tmp[0] << 24) | (net_tmp[1] << 16) | (net_tmp[2] << 8) | net_tmp[3])) {
			printf("invalid start ip: %s\n", ip_start);
			return -1;
		}
		net_tmp[0] = u_end[0] & netmask[0];
		net_tmp[1] = u_end[1] & netmask[1];
		net_tmp[2] = u_end[2] & netmask[2];
		net_tmp[3] = u_end[3] & netmask[3];
		if (i_network != ((net_tmp[0] << 24) | (net_tmp[1] << 16) | (net_tmp[2] << 8) | net_tmp[3])) {
			printf("invalid end ip: %s\n", ip_end);
			return -1;
		}
		if (!((i_start < i_end) && (i_start > i_network) && (i_end > i_network))) {
			printf("ERROR: start ip(%s) must < end ip(%s)\n", ip_start, ip_end);
			return -1;
		}
		i_queryip = i_start;
	}else {
		struct timespec timetic;
		unsigned char random;
		do{
			clock_gettime(CLOCK_MONOTONIC, &timetic);
			srand((unsigned) timetic.tv_nsec);
			random = rand();
		}while(random <=2 || random > 254);
		printf("random = %d", random);
		i_start = (i_network & 0xffffff00)|random;
		i_end = i_network & 0xffffffff;
		i_queryip = i_start;
		/*i_start = i_network + 15;
		i_end = i_network & 0xffffffff;
		i_queryip = i_network + 15;*/
	}

	time(&__tt);
	index = 0;
	trigger = true;
	while(trigger)
	{
		char *thread_arg = NULL;
		int ret;
//		dev_ip_t item;

		query_ip[0] = (i_queryip >> 24) & 0xff;
		query_ip[1] = (i_queryip >> 16) & 0xff;
		query_ip[2] = (i_queryip >> 8) & 0xff;
		query_ip[3] = (i_queryip >> 0) & 0xff;
		net_utils_ipbyte2str(query_ip, sz_query_ip);

		if (index < n_threads) {
			thread_arg = (char *)malloc(20);
			strcpy(thread_arg, sz_query_ip);
			check_ip_args[index] = thread_arg;
			check_ip_result[index] = -1;
			check_ip_pids[index] = 0;

			i_queryip++;

			if ((i_queryip & 0xff) == 0 || (i_queryip & 0xff) == 1 || (i_queryip & 0xff) == 255) {
				i_queryip += 3;
			}
			index++;
		} else if (index == n_threads) {
			bool found = false;
			// threads start/stop
			multi_work_start(n_threads, check_ip_result, check_ip_pids,arp_check_ip_proc, (void **)check_ip_args);
			usleep(100*1000);
			multi_work_stop(n_threads, check_ip_result, check_ip_pids);
			//check results
			for ( i = 0; i < n_threads; i++) {
				ret = check_ip_result[i];
				//printf("thread %d args:%s ret:%d\n", i, check_ip_args[i], check_ip_result[i]);
				if (ret == 0) {
					found = true;
					if (hook)
						hook(sz_l_ip, sz_l_netmask, check_ip_args[i], NULL);
					n_found++;
				}
				if (check_ip_args[i]) {
					free(check_ip_args[i]);
					check_ip_args[i] = NULL;
				}
				if (n_found >= need_n) {
					trigger = false;
					break;
				}
			}
			if (found == false) {
				if (i_mask >= 0xffffff00) {
					if (( i_end - need_n -1)> i_start){
						i_queryip = i_end - need_n - irevert++;
						if (i_queryip <= i_start)
							break;
					}else
						i_queryip += 3;
				} else if (i_mask >= 0xffff0000) {
					i_queryip += 0xff;
				} else if (i_mask >= 0xff000000){
					i_queryip += 0xffff;
				}
			}

			// reset index
			index = 0;
		}
		if ((time(NULL) - __tt) > timeout_s)  {
			printf("NET_find_avai_ip timeout!\n");
			break;
		}
	}
	printf("NET_find_avai_ip %d/%d, spent %ld s (timeout: %ds)\n", n_found, need_n, time(NULL) - __tt , timeout_s);

	for ( i = 0; i < n_threads; i++) {
		if (check_ip_args[i]) {
			free(check_ip_args[i]);
			check_ip_args[i] = NULL;
		}
	}

	return n_found;
}

int NET_adapt_ip(char *shelf_ip, char *client_ip, char *ip_mask, f_ip_found hook)
{
	unsigned int shelf, client;
	if((NULL == shelf_ip) || (NULL == client_ip)){
		return -1;
	}
	//do IP adapted
	shelf = inet_addr(shelf_ip);
	client= inet_addr(client_ip);
	printf("%s/%s\r\n", shelf_ip, client_ip);
	if((shelf & 0xffffff) != (client & 0xffffff)
		|| (g_ip_conflict_flag > 0) ){
		NET_find_avai_ip(client_ip, ip_mask, NULL, NULL, 1, 5, 3, hook);
	}
	return 0;
}

