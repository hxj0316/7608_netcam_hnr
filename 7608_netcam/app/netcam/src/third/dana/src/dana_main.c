#include <errno.h>
#include "netcam_api.h"
#include "dana_callback.h"
#include "eventalarm.h"
#include "ipcsearch.h"
#include "flash_ctl.h"

volatile bool lib_danavideo_started = false;
volatile bool lib_danacloud_inited = false;
volatile bool danaairlink_set_wifi_cb_called = false;
static void* dana_pt = 0;



static int app_dana_motion_event_upload(int nChannel, int nAlarmType, int nAction, void* pParam)
{

	if(nAction == 0)
	{
		return 0;
	}

    switch(nAlarmType)
    {
        case GK_ALARM_TYPE_ALARMIN:     //0:�ź���������ʼ
			break;

        case GK_ALARM_TYPE_DISK_FULL:       //1:Ӳ����
			break;

        case GK_ALARM_TYPE_VLOST:            //2:�źŶ�ʧ

			break;

        case GK_ALARM_TYPE_VMOTION:          //3:�ƶ����

            /*dana�����ϴ�*/
        	danavideo_motion();
            break;
        case GK_ALARM_TYPE_DISK_UNFORMAT:    //4:Ӳ��δ��ʽ��
			break;

        case GK_ALARM_TYPE_DISK_RWERR:       //5:��дӲ�̳���,
			break;

        case GK_ALARM_TYPE_VSHELTER:         //6:�ڵ�����
			break;

        case GK_ALARM_TYPE_ALARMIN_RESUME:         //9:�ź��������ָ�
			break;

        case GK_ALARM_TYPE_VLOST_RESUME:         //10:��Ƶ��ʧ�����ָ�
			break;

        case GK_ALARM_TYPE_VMOTION_RESUME:         //11:��Ƶ�ƶ���ⱨ���ָ�
			break;

        case GK_ALARM_TYPE_NET_BROKEN:       //12:����Ͽ�
			break;

        case GK_ALARM_TYPE_IP_CONFLICT:      //13:IP��ͻ
			break;

        default:
			PRINT_INFO("Alarm: Type = %d", nAlarmType);
        	break;
    }

    //PRINT_INFO("alarm_type:%s action:%s\n", eventType, strAction);

    return 0;


}



dana_video_feature_t feature_list[] = {
	DANA_VIDEO_HAVE_CLOUD_STORAGE,
    DANA_VIDEO_HAVE_PERSONAL_STORAGE
};

static void danavideo_hb_is_ok(void)
{
	PRINT_INFO("[danalib_debug] THE CONN TO P2P SERVER IS OK\n");
}

static void danavideo_hb_is_not_ok(void)
{
	PRINT_INFO("[danalib_debug] THE CONN TO P2P SERVER IS NOT OK\n");
}

static void danavideo_upgrade_rom(const char* rom_path,
	const char *rom_md5, const uint32_t rom_size)
{

}

static void danavideo_autoset(const uint32_t power_freq,
	const int64_t now_time, const char *time_zone,
	const char *ntp_server1, const char *ntp_server2)
{
	int zone;
	char ch;
	if (strlen(time_zone) < strlen("GMT+08:00"))
		zone = 0;
	else
	{
		ch = time_zone[3];
		zone = 10 * (time_zone[4] - '0');
		zone += time_zone[5] - '0';

		zone *= 60;
		zone += 10 * (time_zone[7] - '0');
		zone += time_zone[8] - '0';

		zone = ch == '+' ? zone : 0 - zone;
	}

	netcam_sys_set_time_zone_by_utc_second(now_time - zone * 60, zone);
}

static void danavideo_setwifiap(const uint32_t ch_no,
	const uint32_t ip_type, const char *ipaddr, const char *netmask,
	const char *gateway, const char *dns_name1, const char *dns_name2,
	const char *essid, const char *auth_key, const uint32_t enc_type)
{
	PRINT_INFO("SETWIFIAP\n\tch_no: %d\n\tip_type: %d\n\tipaddr: %s\n\tnetmask: %s\n\tgateway: %s\n\tdns_name1: %s\n\tdns_name2: %s\n\tessid: %s\n\tauth_key: %s\n\tenc_type: %d\n", ch_no, ip_type, ipaddr, netmask, gateway, dns_name1, dns_name2, essid, auth_key, enc_type);
    switch (enc_type) {
        case DANAVIDEO_WIFI_ENCTYPE_NONE:
            PRINT_INFO("DANAVIDEO_WIFI_ENCTYPE_NONE\n");
            break;
        case DANAVIDEO_WIFI_ENCTYPE_WEP:
            PRINT_INFO("DANAVIDEO_WIFI_ENCTYPE_WEP\n");
            break;
        case DANAVIDEO_WIFI_ENCTYPE_WPA_TKIP:
            PRINT_INFO("DANAVIDEO_WIFI_ENCTYPE_WPA_TKIP\n");
            break;
        case DANAVIDEO_WIFI_ENCTYPE_WPA_AES:
            PRINT_INFO("DANAVIDEO_WIFI_ENCTYPE_WPA_AES\n");
            break;
        case DANAVIDEO_WIFI_ENCTYPE_WPA2_TKIP:
            PRINT_INFO("DANAVIDEO_WIFI_ENCTYPE_WPA2_TKIP\n");
            break;
        case DANAVIDEO_WIFI_ENCTYPE_WPA2_AES:
            PRINT_INFO("DANAVIDEO_WIFI_ENCTYPE_WPA2_AES\n");
            break;
        case DANAVIDEO_WIFI_ENCTYPE_INVALID:
        default:
            PRINT_INFO("DANAVIDEO_WIFI_ENCTYPE_INVALID\n");
            break;
    }

	WIFI_LINK_INFO_t linkInfo;
	ST_SDK_NETWORK_ATTR net_attr;

	strcpy(net_attr.name,netcam_net_wifi_get_devname());
	netcam_net_get(&net_attr);

	memset(&linkInfo,0,sizeof(WIFI_LINK_INFO_t));
	linkInfo.isConnect = 1;
	strcpy(linkInfo.linkEssid,essid);
	if(auth_key != 0)
		strncpy(linkInfo.linkPsd, auth_key, sizeof(linkInfo.linkPsd));
	if(WIFI_CONNECT_OK == netcam_net_wifi_set_connect_info(&linkInfo))
	{
		PRINT_INFO("wifi connected succedd!\n");
		danaairlink_set_wifi_cb_called = true;
	}

    PRINT_INFO("����WiFi...\n");
    sleep(3);
    //danaairlink_set_wifi_cb_called = true; // ���̻߳��ٴν�������״̬

}

static uint32_t danavideo_local_auth(const char *user_name,
	const char *user_pass)
{
    return 0;
}

static void danavideo_conf_create_or_updated(const char *conf_absolute_pathname)
{

}

static uint32_t danavideo_get_connected_wifi_quality()
{
    uint32_t wifi_quality = 45;
    return wifi_quality;
}

static void danavideo_productsetdeviceinfo(const char *model, const char *sn, const char *hw_mac)
{

}


static void* dana_start_thread(void* arg)
{
	int i,ret = 0;
	#if 1
	volatile bool danaairlink_inited = false, danaairlink_started = false;
	#endif
	PRINT_INFO("[dana_start_thread] dana lib version %s\n",
		lib_danavideo_linked_version_str(lib_danavideo_linked_version()));

	lib_danavideo_set_hb_is_ok(danavideo_hb_is_ok);
    lib_danavideo_set_hb_is_not_ok(danavideo_hb_is_not_ok);

    lib_danavideo_set_upgrade_rom(danavideo_upgrade_rom);

    lib_danavideo_set_autoset(danavideo_autoset);
    lib_danavideo_set_local_setwifiap(danavideo_setwifiap);

    lib_danavideo_set_local_auth(danavideo_local_auth);

    lib_danavideo_set_conf_created_or_updated(danavideo_conf_create_or_updated);

    lib_danavideo_set_get_connected_wifi_quality(danavideo_get_connected_wifi_quality);

    lib_danavideo_set_productsetdeviceinfo(danavideo_productsetdeviceinfo);

	lib_danavideo_set_maximum_buffering_data_size(DANALIB_MAX_BUFFER_SIZE);
	lib_danavideo_set_startup_timeout(DANALIB_STARTUP_TIMEOUT);


	dana_pt = event_alarm_open(app_dana_motion_event_upload);

	#if 0 // ����DanaAirLink
    // DanaAirLink
    // Ŀǰ֧��MT7601, RTL8188, RT3070��оƬ,������õ�������оƬ������ϵDanale
    // ĳЩоƬ��Ҫ���ô����ṩ������
    // ��Ҫlib_danavideo_set_local_setwifiapע������Wi-Fi�ص�����
    char *if_name = "ra0";
    if (!danaairlink_set_danalepath(DANA_DEVICEID_PATH)) {
        PRINT_INFO("testdanavideo danaairlink_set_danalepath failed\n");
        return -1;
    }
    if (!danaairlink_init(DANAAIRLINK_CHIP_TYPE_MT7601, if_name)) {
        PRINT_INFO("testdanavideo danaairlink_set_danalepath failed\n");
        return -1;
    }
    danaairlink_inited = true;
    PRINT_INFO("testdanavideo danaairlink_init succeeded\n");
    if (!danaairlink_start_once()) {
        PRINT_INFO("testdanavideo danaairlink_start_once failed\n");
        danaairlink_deinit();
        return -1;
    }
    danaairlink_started = true;
    PRINT_INFO("testdanavideo danaairlink_start_once succeeded\n");
	PRINT_INFO("################ while ##############\n");


    while (1) {
        if (danaairlink_set_wifi_cb_called) {
            // ��ʾ�ٴν�������״̬
            danaairlink_set_wifi_cb_called = false;
            if (danaairlink_start_once()) {
                PRINT_INFO("testdanavideo danaairlink_start_once succeeded\n");
            } else {
                PRINT_INFO("testdanavideo WARING danaairlink_start_once failed\n");
            }
        } else {
            sleep(2);
			PRINT_INFO("################ sleep ##############\n");
        }
    }


    // ������ʱ����danaairlink_stopֹͣ����
    // ������danaairlink_deinit������Դ

#endif
	//init p2p
	PRINT_INFO("[dana_start_thread] device ID: %s from conf\n",
		lib_danavideo_deviceid_from_conf(DANA_DEVICEID_PATH));
	lib_danavideo_set_listen_port(true, 12349);

	i = 1;
	while (!lib_danavideo_init(DANA_DEVICEID_PATH, NULL, NULL,
			"TEST_chip", "TEST_schema", "TEST_product", &danavideocallbackfuns))
	{
		printf("[dana_start_thread] lib_danavideo_init failed, try again %d times\n", i);
		i++;
		sleep(2);
	}
	PRINT_INFO("[dana_start_thread] lib_danavideo_init succeeded\n");

	i = 1;
	while (!lib_danavideo_start())
	{
		PRINT_ERR("[dana_start_thread] lib_danavideo_start failed, try again %d times\n", i);
		i++;
		sleep(2);
	}

	PRINT_INFO("[dana_start_thread] listen port: %d, device ID: %s\n",
		lib_danavideo_get_listen_port(), lib_danavideo_deviceid());
	PRINT_INFO("[dana_start_thread] lib_danavideo_start succeeded\n");

	lib_danavideo_started = true;

#if 1
	//D����a?��lib_danavideo_init3��1|o���̡¨�?
	PRINT_INFO("[dana_start_thread] dana cloud version: %s\n",
		lib_danavideo_cloud_linked_version_str(lib_danavideo_cloud_linked_version()));
    lib_danavideo_cloud_set_cloud_mode_changed(danacloud_mode_changed);

	i = 1;
	while (!lib_danavideo_cloud_init(DANACLOUD_CHAN_NO, DANACLOUD_MAX_BUFFER_SIZE,
			DANACLOUD_MAX_PACKET_SIZE, DANAVIDEO_CLOUD_MODE_REALTIME))
	{
        PRINT_ERR("[dana_start_thread]init dana cloud failed %d times\n", i);
		i++;
        sleep(2);

    }

    PRINT_INFO("[dana_cloud_thread]init dana cloud succeeded\n");
    lib_danacloud_inited = true;


#if 1
	dana_video_feature_t feature_list[] = {
				DANAVIDEO_FEATURE_HAVE_BATTERY,
				DANAVIDEO_FEATURE_HAVE_GPS,
				DANAVIDEO_FEATURE_HAVE_PTZ_L_R_U_D,
				DANAVIDEO_FEATURE_HAVE_WIPER,
				DANAVIDEO_FEATURE_HAVE_ZOOM_LENS,
				DANAVIDEO_FEATURE_HAVE_SD,
				DANAVIDEO_FEATURE_HAVE_MIC,
				DANAVIDEO_FEATURE_HAVE_SPEAKER,
				DANA_VIDEO_HAVE_CLOUD_STORAGE
	};

#endif

    if (!lib_danavideo_util_setdevicefeature
		(sizeof(feature_list)/sizeof(dana_video_feature_t), feature_list))
	{
        PRINT_ERR("[dana_cloud_thread]set device feature failed\n");
		return 0;
    }

	PRINT_INFO("[dana_cloud_thread]set device feature succeeded\n");


	if (danaairlink_started) {
		danaairlink_stop();
		danaairlink_started = false;
	}
	if (danaairlink_inited) {
		danaairlink_deinit();
	}
	danaairlink_inited = false;
#endif

	return 0;
}

int danalib_start()
{
    char tmPath[64];
    char defaultPath[64];
    struct stat fileInfo;
	pthread_t thr_dana;
    sprintf(tmPath,"%s/danale.conf",DANA_DEVICEID_PATH);
    sprintf(defaultPath,"%s/danale.conf",RESOURCE_DANA_DEFAULT_DIR);
    if(stat(tmPath,&fileInfo) != 0)
    {
        PRINT_INFO("Dana %s not exist,use default:%s",tmPath,defaultPath);
        //system("mkdir -p /tmp/data");
		if (symlink(RESOURCE_DANA_DEFAULT_DIR, DANA_DEVICEID_PATH))
		{
			PRINT_INFO("Fail to link %s to %s",
			RESOURCE_DANA_DEFAULT_DIR, DANA_DEVICEID_PATH);
			return -1;
		}
    }

	if (pthread_create(&thr_dana, NULL, dana_start_thread, NULL))
	{
		PRINT_ERR("[danalib_start] fail to start dana thread.\n");
		return -1;
	}
    P2PName_add(MTD_DANA_P2P);
    return 0;
}


void danalib_exit()
{
	if (lib_danavideo_started)
	{
        event_alarm_close(dana_pt);
        lib_danavideo_stop();
		usleep(800*1000);
		lib_danavideo_deinit();
		usleep(800*1000);

		lib_danavideo_started = false;
	}

	if (lib_danacloud_inited)
	{
		lib_danavideo_cloud_deinit();
        lib_danacloud_inited = false;
	}

	PRINT_INFO("[danalib_exit] dana video exit\n");
}


