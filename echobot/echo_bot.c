/*
 * 
 * Zoff <zoff@zoff.cc>
 * in 2016
 *
 * dirty hack (echobot and toxic were used as blueprint)
 *
 *
 * compile on linux (dynamic):
 *  gcc -O2 -fPIC -Wall -Wpedantic -o echo_bot echo_bot.c -std=gnu99 -lsodium -I/usr/local/include/ -ltoxcore -ltoxav -lpthread
 * compile for debugging (dynamic):
 *  gcc -O0 -g -fPIC -Wall -Wpedantic -o echo_bot echo_bot.c -std=gnu99 -lsodium -I/usr/local/include/ -ltoxcore -ltoxav -lpthread
 *
 * compile on linux (static):
 *  gcc -O2 -Wall -Wpedantic -o echo_bot_static echo_bot.c -static -std=gnu99 -L/usr/local/lib -I/usr/local/include/ \
    -lsodium -ltoxcore -ltoxav -ltoxgroup -ltoxmessenger -ltoxfriends -ltoxnetcrypto \
    -ltoxdht -ltoxnetwork -ltoxcrypto -lsodium -lpthread -static-libgcc -static-libstdc++ \
    -lopus -lvpx -lm -lpthread
 *
 *
 *
 */


#include <ctype.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <dirent.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <pthread.h>

#include <sodium/utils.h>
#include <tox/tox.h>
#include <tox/toxav.h>

// ----------- version -----------
// ----------- version -----------
#define VERSION_MAJOR 0
#define VERSION_MINOR 99
#define VERSION_PATCH 0
static const char global_version_string[] = "0.99.0";
// ----------- version -----------
// ----------- version -----------

typedef struct DHT_node {
    const char *ip;
    uint16_t port;
    const char key_hex[TOX_PUBLIC_KEY_SIZE*2 + 1];
    unsigned char key_bin[TOX_PUBLIC_KEY_SIZE];
} DHT_node;

#define MAX_AVATAR_FILE_SIZE 65536
#define TOXIC_MAX_NAME_LENGTH 32   /* Must be <= TOX_MAX_NAME_LENGTH */
// #define PATH_MAX 255
#define TIME_STR_SIZE 32
#define MAX_STR_SIZE 200

#define CURRENT_LOG_LEVEL 9 // 0 -> error, 1 -> warn, 2 -> info, 9 -> debug

#define KiB 1024
#define MiB 1048576       /* 1024^2 */
#define GiB 1073741824    /* 1024^3 */

#define seconds_since_last_mod 1 // how long to wait before we process image files in seconds
#define MAX_FILES 3 // how many filetransfers to/from 1 friend at the same time?
#define MAX_RESEND_FILE_BEFORE_ASK 4

#define c_sleep(x) usleep(1000*x)

typedef enum FILE_TRANSFER_STATE {
    FILE_TRANSFER_INACTIVE, // == 0 , this is important
    FILE_TRANSFER_PAUSED,
    FILE_TRANSFER_PENDING,
    FILE_TRANSFER_STARTED,
} FILE_TRANSFER_STATE;

typedef enum FILE_TRANSFER_DIRECTION {
    FILE_TRANSFER_SEND,
    FILE_TRANSFER_RECV
} FILE_TRANSFER_DIRECTION;

struct FileTransfer {
    FILE *file;
    FILE_TRANSFER_STATE state;
    FILE_TRANSFER_DIRECTION direction;
    uint8_t file_type;
    char file_name[TOX_MAX_FILENAME_LENGTH + 1];
    char file_path[PATH_MAX + 1];    /* Not used by senders */
    double   bps;
    uint32_t filenum;
    uint32_t friendnum;
    size_t   index;
    uint64_t file_size;
    uint64_t position;
    time_t   last_line_progress;   /* The last time we updated the progress bar */
    time_t   last_keep_alive;  /* The last time we sent or received data */
    uint32_t line_id;
    uint8_t  file_id[TOX_FILE_ID_LENGTH];
};


struct LastOnline {
    uint64_t last_on;
    struct tm tm;
    char hour_min_str[TIME_STR_SIZE];    /* holds 24-hour time string e.g. "15:43:24" */
};

struct GroupChatInvite {
    char *key;
    uint16_t length;
    uint8_t type;
    bool pending;
};

typedef struct {
    char name[TOXIC_MAX_NAME_LENGTH + 1];
    int namelength;
    char statusmsg[TOX_MAX_STATUS_MESSAGE_LENGTH + 1];
    size_t statusmsg_len;
    char pub_key[TOX_PUBLIC_KEY_SIZE];
    char pubkey_string[(TOX_ADDRESS_SIZE * 2 + 1)];
    char worksubdir[MAX_STR_SIZE];
    uint32_t num;
    int chatwin;
    bool active;
    TOX_CONNECTION connection_status;
    bool is_typing;
    bool logging_on;    /* saves preference for friend irrespective of global settings */
    uint8_t status;
    struct LastOnline last_online;
    struct FileTransfer file_receiver[MAX_FILES];
    struct FileTransfer file_sender[MAX_FILES];
	char last_answer[100];
	int waiting_for_answer; // 0 -> no, 1 -> waiting for answer, 2 -> got answer
} ToxicFriend;

typedef struct {
    char name[TOXIC_MAX_NAME_LENGTH + 1];
    int namelength;
    char pub_key[TOX_PUBLIC_KEY_SIZE];
    uint32_t num;
    bool active;
    uint64_t last_on;
} BlockedFriend;

typedef struct {
    int num_selected;
    size_t num_friends;
    size_t num_online;
    size_t max_idx;    /* 1 + the index of the last friend in list */
    uint32_t *index;
    ToxicFriend *list;
} FriendsList;


static struct Avatar {
    char name[TOX_MAX_FILENAME_LENGTH + 1];
    size_t name_len;
    char path[PATH_MAX + 1];
    size_t path_len;
    off_t size;
} Avatar;

typedef struct {
    bool incoming;
    uint32_t state;
    pthread_mutex_t arb_mutex[1];
} CallControl;

void on_avatar_chunk_request(Tox *m, struct FileTransfer *ft, uint64_t position, size_t length);
int avatar_send(Tox *m, uint32_t friendnum);
struct FileTransfer *new_file_transfer(uint32_t friendnum, uint32_t filenum, FILE_TRANSFER_DIRECTION direction, uint8_t type);
void kill_all_file_transfers_friend(Tox *m, uint32_t friendnum);
int has_reached_max_file_transfer_for_friend(uint32_t num);
static int find_friend_in_friendlist(uint32_t friendnum);
int is_friend_online(Tox *tox, uint32_t num);

const char *savedata_filename = "savedata.tox";
const char *savedata_tmp_filename = "savedata.tox.tmp";
const char *log_filename = "echobot.log";
const char *my_avatar_filename = "avatar.png";
const char *motion_pics_dir = "./m/";
const char *motion_pics_work_dir = "./work/";
const char *motion_capture_file_extension = ".jpg";
const char *motion_capture_file_extension_mov = ".avi";

const char *shell_cmd__single_shot = "/home/pi/inst_/single_shot.sh";
int global_want_restart = 0;
const char *global_timestamp_format = "%H:%M:%S";
const char *global_long_timestamp_format = "%Y-%m-%d %H:%M:%S";
uint64_t global_start_time;


TOX_CONNECTION my_connection_status = TOX_CONNECTION_NONE;
FILE *logfile = NULL;
FriendsList Friends;

void dbg(int level, const char *fmt, ...)
{
	char *level_and_format = NULL;
	char *fmt_copy = NULL;

	if (fmt == NULL)
	{
		return;
	}

	if (strlen(fmt) < 1)
	{
		return;
	}

	if (!logfile)
	{
		return;
	}

	if ((level < 0) || (level > 9))
	{
		level = 0;
	}

	level_and_format = malloc(strlen(fmt) + 3);

	if (!level_and_format)
	{
		// fprintf(stderr, "free:000a\n");
		return;
	}

	fmt_copy = level_and_format + 2;
	strcpy(fmt_copy, fmt);
	level_and_format[1] = ':';
	if (level == 0)
	{
		level_and_format[0] = 'E';
	}
	else if (level == 1)
	{
		level_and_format[0] = 'W';
	}
	else if (level == 2)
	{
		level_and_format[0] = 'I';
	}
	else
	{
		level_and_format[0] = 'D';
	}

	if (level <= CURRENT_LOG_LEVEL)
	{
		va_list ap;
		va_start(ap, fmt);
		vfprintf(logfile, level_and_format, ap);
		va_end(ap);
	}

	// fprintf(stderr, "free:001\n");
	if (level_and_format)
	{
		// fprintf(stderr, "free:001.a\n");
		free(level_and_format);
	}
	// fprintf(stderr, "free:002\n");
}


time_t get_unix_time(void)
{
    return time(NULL);
}


Tox *create_tox()
{
    Tox *tox;

    struct Tox_Options options;
    tox_options_default(&options);

	options.ipv6_enabled = false;

    FILE *f = fopen(savedata_filename, "rb");
    if (f)
	{
        fseek(f, 0, SEEK_END);
        long fsize = ftell(f);
        fseek(f, 0, SEEK_SET);

        uint8_t *savedata = malloc(fsize);

        size_t dummy = fread(savedata, fsize, 1, f);
		if (dummy < 1)
		{
			dbg(0, "reading savedata failed\n");
		}
        fclose(f);

        options.savedata_type = TOX_SAVEDATA_TYPE_TOX_SAVE;
        options.savedata_data = savedata;
        options.savedata_length = fsize;

        tox = tox_new(&options, NULL);

        free((void *)savedata);
    }
	else
	{
        tox = tox_new(&options, NULL);
    }

    return tox;
}

void update_savedata_file(const Tox *tox)
{
    size_t size = tox_get_savedata_size(tox);
    char *savedata = malloc(size);
    tox_get_savedata(tox, (uint8_t *)savedata);

    FILE *f = fopen(savedata_tmp_filename, "wb");
    fwrite(savedata, size, 1, f);
    fclose(f);

    rename(savedata_tmp_filename, savedata_filename);

    free(savedata);
}

off_t file_size(const char *path)
{
    struct stat st;

    if (stat(path, &st) == -1)
    {
        return 0;
    }

    return st.st_size;
}

int bin_id_to_string(const char *bin_id, size_t bin_id_size, char *output, size_t output_size)
{
    if (bin_id_size != TOX_ADDRESS_SIZE || output_size < (TOX_ADDRESS_SIZE * 2 + 1))
    {
        return -1;
    }

    size_t i;
    for (i = 0; i < TOX_ADDRESS_SIZE; ++i)
    {
        snprintf(&output[i * 2], output_size - (i * 2), "%02X", bin_id[i] & 0xff);
    }

	return 0;
}

size_t get_file_name(char *namebuf, size_t bufsize, const char *pathname)
{
    int len = strlen(pathname) - 1;
    char *path = strdup(pathname);

    if (path == NULL)
    {
        // TODO
    }

    while (len >= 0 && pathname[len] == '/')
    {
        path[len--] = '\0';
    }

    char *finalname = strdup(path);

    if (finalname == NULL)
    {
        // TODO
    }

    const char *basenm = strrchr(path, '/');
    if (basenm != NULL)
    {
        if (basenm[1])
        {
            strcpy(finalname, &basenm[1]);
        }
    }

    snprintf(namebuf, bufsize, "%s", finalname);
    free(finalname);
    free(path);

    return strlen(namebuf);
}

void bootstrap(Tox *tox)
{
    DHT_node nodes[] =
    {
        {"178.62.250.138",             33445, "788236D34978D1D5BD822F0A5BEBD2C53C64CC31CD3149350EE27D4D9A2F9B6B", {0}},
        {"2a03:b0c0:2:d0::16:1",       33445, "788236D34978D1D5BD822F0A5BEBD2C53C64CC31CD3149350EE27D4D9A2F9B6B", {0}},
        {"tox.zodiaclabs.org",         33445, "A09162D68618E742FFBCA1C2C70385E6679604B2D80EA6E84AD0996A1AC8A074", {0}},
        {"163.172.136.118",            33445, "2C289F9F37C20D09DA83565588BF496FAB3764853FA38141817A72E3F18ACA0B", {0}},
        {"2001:bc8:4400:2100::1c:50f", 33445, "2C289F9F37C20D09DA83565588BF496FAB3764853FA38141817A72E3F18ACA0B", {0}},
        {"128.199.199.197",            33445, "B05C8869DBB4EDDD308F43C1A974A20A725A36EACCA123862FDE9945BF9D3E09", {0}},
        {"2400:6180:0:d0::17a:a001",   33445, "B05C8869DBB4EDDD308F43C1A974A20A725A36EACCA123862FDE9945BF9D3E09", {0}},
        {"biribiri.org",               33445, "F404ABAA1C99A9D37D61AB54898F56793E1DEF8BD46B1038B9D822E8460FAB67", {0}}
    };

    for (size_t i = 0; i < sizeof(nodes)/sizeof(DHT_node); i ++) {
        sodium_hex2bin(nodes[i].key_bin, sizeof(nodes[i].key_bin),
                       nodes[i].key_hex, sizeof(nodes[i].key_hex)-1, NULL, NULL, NULL);
        tox_bootstrap(tox, nodes[i].ip, nodes[i].port, nodes[i].key_bin, NULL);
    }
}

// fill string with toxid in upper case hex.
// size of toxid_str needs to be: [TOX_ADDRESS_SIZE*2 + 1] !!
void get_my_toxid(Tox *tox, char *toxid_str)
{
    uint8_t tox_id_bin[TOX_ADDRESS_SIZE];
    tox_self_get_address(tox, tox_id_bin);

	char tox_id_hex_local[TOX_ADDRESS_SIZE*2 + 1];
    sodium_bin2hex(tox_id_hex_local, sizeof(tox_id_hex_local), tox_id_bin, sizeof(tox_id_bin));

    for (size_t i = 0; i < sizeof(tox_id_hex_local)-1; i ++)
	{
        tox_id_hex_local[i] = toupper(tox_id_hex_local[i]);
    }

	snprintf(toxid_str, (size_t)(TOX_ADDRESS_SIZE*2 + 1), "%s", (const char*)tox_id_hex_local);
}

void print_tox_id(Tox *tox)
{
    char tox_id_hex[TOX_ADDRESS_SIZE*2 + 1];
	get_my_toxid(tox, tox_id_hex);

    if (logfile)
    {
        // printf("--MyToxID--:%s\n", tox_id_hex);
        // fprintf(logfile, "--MyToxID--:%s\n", tox_id_hex);
		dbg(2, "--MyToxID--:%s\n", tox_id_hex);
        int fd = fileno(logfile);
        fsync(fd);
    }
}

int is_friend_online(Tox *tox, uint32_t num)
{
	int j = find_friend_in_friendlist(num);
	switch (Friends.list[j].connection_status)
	{
		case TOX_CONNECTION_NONE:
			return 0;
			break;
		case TOX_CONNECTION_TCP:
			return 1;
			break;
		case TOX_CONNECTION_UDP:
			return 1;
			break;
		default:
			return 0;
			break;
	}
}

static int find_friend_in_friendlist(uint32_t friendnum)
{
	int i;

	for (i = 0; i <= Friends.max_idx; ++i)
	{
        if (Friends.list[i].num == friendnum)
		{
			return i;
		}
	}

	return -1;
}

static void update_friend_last_online(uint32_t num, time_t timestamp)
{
	int friendlistnum = find_friend_in_friendlist(num);

    Friends.list[friendlistnum].last_online.last_on = timestamp;
    Friends.list[friendlistnum].last_online.tm = *localtime((const time_t *)&timestamp);

    /* if the format changes make sure TIME_STR_SIZE is the correct size !! */
    strftime(Friends.list[friendlistnum].last_online.hour_min_str, TIME_STR_SIZE, global_timestamp_format, &Friends.list[friendlistnum].last_online.tm);
}

void send_file_to_friend(Tox *m, uint32_t num, const char* filename)
{
    // ------- hack to send file --------
    // ------- hack to send file --------
    const char *errmsg = NULL;
    char path[MAX_STR_SIZE];

    snprintf(path, sizeof(path), "%s", filename);
    dbg(2, "send_file_to_friend:path=%s\n", path);

    FILE *file_to_send = fopen(path, "r");

    if (file_to_send == NULL)
    {
		dbg(0, "error opening file\n");
		return;
    }

    off_t filesize = file_size(path);

    if (filesize == 0)
    {
		dbg(0, "filesize 0\n");
		fclose(file_to_send);
		return;
    }

    char file_name[TOX_MAX_FILENAME_LENGTH];
    size_t namelen = get_file_name(file_name, sizeof(file_name), path);

    TOX_ERR_FILE_SEND err;
    uint32_t filenum = tox_file_send(m, num, TOX_FILE_KIND_DATA, (uint64_t) filesize, NULL,
		(uint8_t *) file_name, namelen, &err);

    if (err != TOX_ERR_FILE_SEND_OK)
    {
		dbg(0, "! TOX_ERR_FILE_SEND_OK\n");
		goto on_send_error;
    }

    struct FileTransfer *ft = new_file_transfer(num, filenum, FILE_TRANSFER_SEND, TOX_FILE_KIND_DATA);

    if (!ft)
    {
		dbg(0, "ft=NULL\n");
		err = TOX_ERR_FILE_SEND_TOO_MANY;
		goto on_send_error;
    }

    memcpy(ft->file_name, file_name, namelen + 1);
    ft->file = file_to_send;
    ft->file_size = filesize;
    tox_file_get_file_id(m, num, filenum, ft->file_id, NULL);

    return;

on_send_error:

    switch (err)
	{
        case TOX_ERR_FILE_SEND_FRIEND_NOT_FOUND:
            errmsg = "File transfer failed: Invalid friend.";
            break;

        case TOX_ERR_FILE_SEND_FRIEND_NOT_CONNECTED:
            errmsg = "File transfer failed: Friend is offline.";

            break;

        case TOX_ERR_FILE_SEND_NAME_TOO_LONG:
            errmsg = "File transfer failed: Filename is too long.";
            break;

        case TOX_ERR_FILE_SEND_TOO_MANY:
            errmsg = "File transfer failed: Too many concurrent file transfers.";

            break;

        default:
            errmsg = "File transfer failed.";
            break;
    }

    dbg(0, "ft error=%s\n", errmsg);
    tox_file_control(m, num, filenum, TOX_FILE_CONTROL_CANCEL, NULL);
    fclose(file_to_send);

    // ------- hack to send file --------
    // ------- hack to send file --------
}


int copy_file(const char *from, const char *to)
{
    int fd_to, fd_from;
    char buf[4096];
    ssize_t nread;
    int saved_errno;

    fd_from = open(from, O_RDONLY);

    if (fd_from < 0)
	{
		dbg(0, "copy_file:002\n");
        return -1;
	}

    fd_to = open(to, O_WRONLY | O_CREAT | O_EXCL, 0666);
    if (fd_to < 0)
	{
		dbg(0, "copy_file:003\n");
        goto out_error;
	}

    while (nread = read(fd_from, buf, sizeof buf), nread > 0)
    {
        char *out_ptr = buf;
        ssize_t nwritten;

        do
		{
            nwritten = write(fd_to, out_ptr, nread);

            if (nwritten >= 0)
            {
                nread -= nwritten;
                out_ptr += nwritten;
            }
            else if (errno != EINTR)
            {
				dbg(0, "copy_file:004\n");
                goto out_error;
            }

        } while (nread > 0);
    }

    if (nread == 0)
    {
        if (close(fd_to) < 0)
        {
            fd_to = -1;
            dbg(0, "copy_file:005\n");
            goto out_error;
        }

        close(fd_from);

        /* Success! */
        return 0;
    }


  out_error:
    saved_errno = errno;

    close(fd_from);
    if (fd_to >= 0)
	{
        close(fd_to);
	}

	dbg(0, "copy_file:009\n");

    errno = saved_errno;
    return -1;
}



char* copy_file_to_friend_subdir(int friendlistnum, const char* file_with_path, const char* filename)
{
	dbg(2, "newpath=%s %s\n", file_with_path, filename);
	int errcode;

	char *newname = NULL;
	newname = malloc(300);
	snprintf(newname, 299, "%s/%s", (const char*)Friends.list[friendlistnum].worksubdir, filename);
	dbg(2, "copy_file %s -> %s\n", file_with_path, newname);
	dbg(2, "copy_file friend:workdir=%s\n", (const char*)Friends.list[friendlistnum].worksubdir);
	errcode = copy_file(file_with_path, newname);
	dbg(9, "copy_file:ready:res=%d\n", errcode);

	dbg(9, "newpath=%s\n", newname);

	return newname;
}

void send_file_to_all_friends(Tox *m, const char* file_with_path, const char* filename)
{
    size_t i;
    size_t numfriends = tox_self_get_friend_list_size(m);
	char *newname = NULL;
	int j = -1;

    for (i = 0; i < numfriends; ++i)
    {
        dbg(2, "sending file (%s) to friendnum=%d\n", file_with_path, (int)i);

		j = find_friend_in_friendlist((uint32_t) i);
		if (j > -1)
		{
			newname = copy_file_to_friend_subdir((int) j, file_with_path, filename);

			// see if we have reached max filetransfers
			if (has_reached_max_file_transfer_for_friend((uint32_t) i) == 0)
			{
				if (is_friend_online(m, (uint32_t) i) == 1)
				{
					send_file_to_friend(m, i, newname);
				}
			}
			free(newname);
			newname = NULL;
		}
    }

    unlink(file_with_path);
}

void friendlist_onConnectionChange(Tox *m, uint32_t num, TOX_CONNECTION connection_status, void *user_data)
{
	int friendlistnum = find_friend_in_friendlist(num);
    dbg(2, "friendlist_onConnectionChange:friendnum=%d %d\n", (int)num, (int)connection_status);

    if (avatar_send(m, num) == -1)
    {
        dbg(0, "avatar_send failed for friend %d\n", num);
    }
    Friends.list[friendlistnum].connection_status = connection_status;
    update_friend_last_online(num, get_unix_time());
}



void friendlist_onFriendAdded(Tox *m, uint32_t num, bool sort)
{
    dbg(9, "friendlist_onFriendAdded:001\n");

    if (Friends.max_idx == 0)
    {
		dbg(9, "friendlist_onFriendAdded:001.a malloc 1 friend struct, max_id=%d, num=%d\n", (int)Friends.max_idx, (int)num);
        Friends.list = malloc(sizeof(ToxicFriend));
    }
    else
    {
		dbg(9, "friendlist_onFriendAdded:001.b realloc %d friend struct, max_id=%d, num=%d\n", (int)(Friends.max_idx + 1), (int)Friends.max_idx, (int)num);
        Friends.list = realloc(Friends.list, ((Friends.max_idx + 1) * sizeof(ToxicFriend)));
    }

	dbg(9, "friendlist_onFriendAdded:001.c set friend to all 0 values\n");
    memset(&Friends.list[Friends.max_idx], 0, sizeof(ToxicFriend)); // fill friend with "0" bytes


	dbg(2, "friendlist_onFriendAdded:003:%d\n", (int)Friends.max_idx);
	Friends.list[Friends.max_idx].num = num;
	Friends.list[Friends.max_idx].active = true;
	Friends.list[Friends.max_idx].connection_status = TOX_CONNECTION_NONE;
	Friends.list[Friends.max_idx].status = TOX_USER_STATUS_NONE;
	Friends.list[Friends.max_idx].waiting_for_answer = 0;
	// Friends.list[i].logging_on = (bool) user_settings->autolog == AUTOLOG_ON;

	TOX_ERR_FRIEND_GET_PUBLIC_KEY pkerr;
	tox_friend_get_public_key(m, num, (uint8_t *) Friends.list[Friends.max_idx].pub_key, &pkerr);

	if (pkerr != TOX_ERR_FRIEND_GET_PUBLIC_KEY_OK)
	{
		dbg(0, "tox_friend_get_public_key failed (error %d)\n", pkerr);
	}

	bin_id_to_string(Friends.list[Friends.max_idx].pub_key, (size_t) TOX_ADDRESS_SIZE, Friends.list[Friends.max_idx].pubkey_string, (size_t) (TOX_ADDRESS_SIZE * 2 + 1));
	dbg(2, "friend pubkey=%s\n", Friends.list[Friends.max_idx].pubkey_string);

	// mkdir subdir of workdir for this friend
	snprintf(Friends.list[Friends.max_idx].worksubdir, sizeof(Friends.list[Friends.max_idx].worksubdir), "%s/%s/", motion_pics_work_dir, (const char*)Friends.list[Friends.max_idx].pubkey_string);
	dbg(2, "friend subdir=%s\n", Friends.list[Friends.max_idx].worksubdir);
	mkdir(Friends.list[Friends.max_idx].worksubdir, S_IRWXU | S_IRWXG); // og+rwx

	TOX_ERR_FRIEND_GET_LAST_ONLINE loerr;
	time_t t = tox_friend_get_last_online(m, num, &loerr);

	if (loerr != TOX_ERR_FRIEND_GET_LAST_ONLINE_OK)
	{
	    t = 0;
	}

	update_friend_last_online(num, t);

	Friends.max_idx++;

}



static void load_friendlist(Tox *m)
{
    size_t i;
    size_t numfriends = tox_self_get_friend_list_size(m);

    for (i = 0; i < numfriends; ++i)
    {
        friendlist_onFriendAdded(m, i, false);
        dbg(2, "loading friend num:%d pubkey=%s\n", (int)i, Friends.list[Friends.max_idx - 1].pubkey_string);
    }
}




void close_file_transfer(Tox *m, struct FileTransfer *ft, int CTRL)
{
    dbg(9, "close_file_transfer:001\n");

    if (!ft)
	{
        return;
	}

    if (ft->state == FILE_TRANSFER_INACTIVE)
	{
        return;
	}

    if (ft->file)
	{
        fclose(ft->file);
	}

    if (CTRL >= 0)
	{
        tox_file_control(m, ft->friendnum, ft->filenum, (TOX_FILE_CONTROL) CTRL, NULL);
	}

    memset(ft, 0, sizeof(struct FileTransfer));
	ft->state = FILE_TRANSFER_INACTIVE; // == 0

}

int has_reached_max_file_transfer_for_friend(uint32_t num)
{
	int active_ft = 0;
	int friendlistnum = find_friend_in_friendlist(num);
	int i;

    for (i = 0; i < MAX_FILES; ++i)
    {
        struct FileTransfer *ft_send = &Friends.list[friendlistnum].file_sender[i];

        if (ft_send->state != FILE_TRANSFER_INACTIVE)
        {
			if (ft_send->file_name != NULL)
			{
				active_ft++;
			}
		}
	}

	if (active_ft < MAX_FILES)
	{
		return 0;
	}
	else
	{
		// have reached max filetransfers already
		return 1;
	}
}

struct FileTransfer *get_file_transfer_from_filename_struct(int friendlistnum, const char* filename)
{
    size_t i;

    for (i = 0; i < MAX_FILES; ++i)
    {
        struct FileTransfer *ft_send = &Friends.list[friendlistnum].file_sender[i];

        if (ft_send->state != FILE_TRANSFER_INACTIVE)
        {
			if (ft_send->file_name != NULL)
			{
				if ((strlen(ft_send->file_name) > 0) && (filename != NULL) && (strlen(filename) > 0))
				{
					if (strncmp((char*)ft_send->file_name, filename, strlen(ft_send->file_name)) == 0)
					{
						// dbg(9, "found ft by filename:%s\n", ft_send->file_name);
						return ft_send;
					}
				}
			}
        }
    }

    return NULL;
}


struct FileTransfer *get_file_transfer_struct(uint32_t friendnum, uint32_t filenum)
{
    size_t i;

	int friendlistnum = find_friend_in_friendlist(friendnum);

    for (i = 0; i < MAX_FILES; ++i)
    {
        struct FileTransfer *ft_send = &Friends.list[friendlistnum].file_sender[i];

        if (ft_send->state != FILE_TRANSFER_INACTIVE && ft_send->filenum == filenum)
        {
            return ft_send;
        }

        struct FileTransfer *ft_recv = &Friends.list[friendlistnum].file_receiver[i];

        if (ft_recv->state != FILE_TRANSFER_INACTIVE && ft_recv->filenum == filenum)
        {
            return ft_recv;
        }
    }

    return NULL;
}

void send_text_message_to_friend(Tox *tox, uint32_t friend_number, const char *fmt, ...)
{
	char msg2[1000];
	size_t length = 0;

	if (fmt == NULL)
	{
		dbg(9, "send_text_message_to_friend:no message to send\n");
		return;
	}

	va_list ap;
	va_start(ap, fmt);
	vsnprintf(msg2, 999, fmt, ap);
	va_end(ap);

	length = (size_t)strlen(msg2);
	tox_friend_send_message(tox, friend_number, TOX_MESSAGE_TYPE_NORMAL, (uint8_t *)msg2, length, NULL);
}


void friend_request_cb(Tox *tox, const uint8_t *public_key, const uint8_t *message, size_t length,
                                   void *user_data)
{
    uint32_t friendnum = tox_friend_add_norequest(tox, public_key, NULL);
    dbg(2, "add friend:002:friendnum=%d max_id=%d\n", friendnum, (int)Friends.max_idx);
    friendlist_onFriendAdded(tox, friendnum, 0);

    update_savedata_file(tox);
}

/* ssssshhh I stole this from ToxBot, don't tell anyone.. */
/* ssssshhh and I stole this from EchoBot, don't tell anyone.. */
static void get_elapsed_time_str(char *buf, int bufsize, uint64_t secs)
{
	long unsigned int minutes = (secs % 3600) / 60;
	long unsigned int hours = (secs / 3600) % 24;
	long unsigned int days = (secs / 3600) / 24;

	snprintf(buf, bufsize, "%lud %luh %lum", days, hours, minutes);
}

void cmd_stats(Tox *tox, uint32_t friend_number)
{
	switch (my_connection_status)
	{
		case TOX_CONNECTION_NONE:
			send_text_message_to_friend(tox, friend_number, "doorspy status:offline");
			break;
		case TOX_CONNECTION_TCP:
			send_text_message_to_friend(tox, friend_number, "doorspy status:Online, using TCP");
			break;
		case TOX_CONNECTION_UDP:
			send_text_message_to_friend(tox, friend_number, "doorspy status:Online, using UDP");
			break;
		default:
			send_text_message_to_friend(tox, friend_number, "doorspy status:*unknown*");
			break;
	}

	// ----- uptime -----
	char time_str[200];
	uint64_t cur_time = time(NULL);
	get_elapsed_time_str(time_str, sizeof(time_str), cur_time - global_start_time);
	send_text_message_to_friend(tox, friend_number, "Uptime: %s", time_str);
	// ----- uptime -----

    char tox_id_hex[TOX_ADDRESS_SIZE*2 + 1];
	get_my_toxid(tox, tox_id_hex);

	send_text_message_to_friend(tox, friend_number, "tox:%s", tox_id_hex);
}

void cmd_kamft(Tox *tox, uint32_t friend_number)
{
	send_text_message_to_friend(tox, friend_number, "killing all filetransfers to you ...");
	kill_all_file_transfers_friend(tox, friend_number);
}

void cmd_snap(Tox *tox, uint32_t friend_number)
{
	send_text_message_to_friend(tox, friend_number, "capture single shot, and send to all friends ...");
	system(shell_cmd__single_shot);
}

void cmd_friends(Tox *tox, uint32_t friend_number)
{
	size_t i;
    size_t numfriends = tox_self_get_friend_list_size(tox);
	int j = -1;

    for (i = 0; i < numfriends; ++i)
    {
		j = find_friend_in_friendlist((uint32_t) i);
		if (j > -1)
		{
			send_text_message_to_friend(tox, friend_number, "%d:friend", j);
			send_text_message_to_friend(tox, friend_number, "%d:key:%s", j, (const char*)Friends.list[j].pubkey_string);
			send_text_message_to_friend(tox, friend_number, "%d:last online (in client local time):%s", j, (const char*)Friends.list[j].last_online.hour_min_str);

			switch (Friends.list[j].connection_status)
			{
				case TOX_CONNECTION_NONE:
					send_text_message_to_friend(tox, friend_number, "%d:%s", j, "status:offline");
					break;
				case TOX_CONNECTION_TCP:
					send_text_message_to_friend(tox, friend_number, "%d:%s", j, "status:Online, using TCP");
					break;
				case TOX_CONNECTION_UDP:
					send_text_message_to_friend(tox, friend_number, "%d:%s", j, "status:Online, using UDP");
					break;
				default:
					send_text_message_to_friend(tox, friend_number, "%d:%s", j, "status:*unknown*");
					break;
			}
		}
    }
}

void cmd_restart(Tox *tox, uint32_t friend_number)
{
	send_text_message_to_friend(tox, friend_number, "doorspy services will restart ...");

	global_want_restart = 1;
}

void cmd_vcm(Tox *tox, uint32_t friend_number)
{
	send_text_message_to_friend(tox, friend_number, "video-call-me not yet implemented!");
}

void send_help_to_friend(Tox *tox, uint32_t friend_number)
{
	send_text_message_to_friend(tox, friend_number, "=========================\nDoorSpy version:%s\n=========================", global_version_string);
	// send_text_message_to_friend(tox, friend_number, " commands are:");
	send_text_message_to_friend(tox, friend_number, " .stats    --> show DoorSpy status");
	send_text_message_to_friend(tox, friend_number, " .friends  --> show DoorSpy Friends");
	send_text_message_to_friend(tox, friend_number, " .kamft    --> kill all my filetransfers");
	send_text_message_to_friend(tox, friend_number, " .snap     --> snap a single still image");
	send_text_message_to_friend(tox, friend_number, " .restart  --> restart DoorSpy system");
	send_text_message_to_friend(tox, friend_number, " .vcm      --> videocall me");
}

void friend_message_cb(Tox *tox, uint32_t friend_number, TOX_MESSAGE_TYPE type, const uint8_t *message,
                                   size_t length, void *user_data)
{
	int j;
	int send_back = 0;

    if (type == TOX_MESSAGE_TYPE_NORMAL)
    {
		if (message != NULL)
		{
			dbg(2, "waiting for answer from friend:%d msg:%s\n", (int)friend_number, (char*)message);

			j = find_friend_in_friendlist(friend_number);
			if (Friends.list[j].waiting_for_answer == 1)
			{
				// we want to get user feedback
				snprintf(Friends.list[j].last_answer, 99, (char*)message);
				Friends.list[j].waiting_for_answer = 2;

				dbg(2, "got answer from friend:%d answer:%s\n", (int)friend_number, Friends.list[j].last_answer);
			}
			else
			{
				dbg(2, "message from friend:%d msg:%s\n", (int)friend_number, (char*)message);

				if (strncmp((char*)message, ".help", strlen((char*)message)) == 0)
				{
					send_help_to_friend(tox, friend_number);
				}
				else if (strncmp((char*)message, ".stats", strlen((char*)message)) == 0)
				{
					cmd_stats(tox, friend_number);
				}
				else if (strncmp((char*)message, ".friends", strlen((char*)message)) == 0)
				{
					cmd_friends(tox, friend_number);
				}
				else if (strncmp((char*)message, ".kamft", strlen((char*)message)) == 0)
				{
					cmd_kamft(tox, friend_number);
				}
				else if (strncmp((char*)message, ".snap", strlen((char*)message)) == 0)
				{
					cmd_snap(tox, friend_number);
				}
				else if (strncmp((char*)message, ".restart", strlen((char*)message)) == 0) // restart doorspy processes (no reboot)
				{
					cmd_restart(tox, friend_number);
				}
				else if (strncmp((char*)message, ".vcm", strlen((char*)message)) == 0) // video call me!
				{
					cmd_vcm(tox, friend_number);
				}
				else
				{
					// send_back = 1;
					// unknown command, just send "help / usage"
					send_help_to_friend(tox, friend_number);
				}
			}
		}
		else
		{
			dbg(2, "message from friend:%d msg:NULL\n", (int)friend_number);
		}
    }
    else
    {
		dbg(2, "message from friend:%d\n", (int)friend_number);
    }

	if (send_back == 1)
	{
		tox_friend_send_message(tox, friend_number, type, message, length, NULL);
	}
}



void on_file_recv_chunk(Tox *m, uint32_t friendnumber, uint32_t filenumber, uint64_t position,
                        const uint8_t *data, size_t length, void *user_data)
{
    struct FileTransfer *ft = get_file_transfer_struct(friendnumber, filenumber);

    if (!ft)
	{
        return;
	}
}


void on_file_recv(Tox *m, uint32_t friendnumber, uint32_t filenumber, uint32_t kind, uint64_t file_size,
                  const uint8_t *filename, size_t filename_length, void *userdata)
{
    /* We don't care about receiving avatars */
    if (kind != TOX_FILE_KIND_DATA)
    {
        tox_file_control(m, friendnumber, filenumber, TOX_FILE_CONTROL_CANCEL, NULL);
        dbg(9, "on_file_recv:002:cancel incoming avatar\n");
        return;
    }
    else
    {
        // cancel all filetransfers. we don't want to receive files
        tox_file_control(m, friendnumber, filenumber, TOX_FILE_CONTROL_CANCEL, NULL);
        dbg(9, "on_file_recv:003:cancel incoming file\n");
        return;
    }
}



void on_file_chunk_request(Tox *tox, uint32_t friendnumber, uint32_t filenumber, uint64_t position,
                           size_t length, void *userdata)
{
    // dbg(9, "on_file_chunk_request:001:friendnum=%d filenum=%d position=%ld len=%d\n", (int)friendnumber, (int)filenumber, (long)position, (int)length);
    struct FileTransfer *ft = get_file_transfer_struct(friendnumber, filenumber);

    if (!ft)
    {
        dbg(0, "on_file_chunk_request:003 ft=NULL\n");
        return;
    }

    if (ft->file_type == TOX_FILE_KIND_AVATAR)
    {
        on_avatar_chunk_request(tox, ft, position, length);
        return;
    }


    if (ft->state != FILE_TRANSFER_STARTED)
    {
        dbg(0, "on_file_chunk_request:005 !FILE_TRANSFER_STARTED\n");
        return;
    }

    if (length == 0)
    {
        dbg(2, "File '%s' successfully sent\n", ft->file_name);

        char origname[300];
        snprintf(origname, 299, "%s", (const char*)ft->file_name);

        close_file_transfer(tox, ft, -1);
		// also remove the file from disk

		int friendlist_num = find_friend_in_friendlist(friendnumber);
        char longname[300];
        snprintf(longname, 299, "%s/%s", (const char*)Friends.list[friendlist_num].worksubdir, origname);
        dbg(2, "delete file %s\n", longname);
		unlink(longname);

        return;
    }

    if (ft->file == NULL)
    {
        dbg(0, "File transfer for '%s' failed: Null file pointer\n", ft->file_name);
        close_file_transfer(tox, ft, TOX_FILE_CONTROL_CANCEL);
        return;
    }

    if (ft->position != position)
    {
        if (fseek(ft->file, position, SEEK_SET) == -1)
        {
            dbg(0, "File transfer for '%s' failed: Seek fail\n", ft->file_name);
            close_file_transfer(tox, ft, TOX_FILE_CONTROL_CANCEL);
            return;
        }

        ft->position = position;
    }

    uint8_t send_data[length];
    size_t send_length = fread(send_data, 1, sizeof(send_data), ft->file);

    if (send_length != length)
    {
        dbg(0, "File transfer for '%s' failed: Read fail\n", ft->file_name);
        close_file_transfer(tox, ft, TOX_FILE_CONTROL_CANCEL);
        return;
    }

    TOX_ERR_FILE_SEND_CHUNK err;
    tox_file_send_chunk(tox, friendnumber, filenumber, position, send_data, send_length, &err);

    if (err != TOX_ERR_FILE_SEND_CHUNK_OK)
    {
        dbg(0, "tox_file_send_chunk failed in chat callback (error %d)\n", err);
    }

    ft->position += send_length;
    ft->bps += send_length;
    ft->last_keep_alive = get_unix_time();

}


void on_avatar_file_control(Tox *m, struct FileTransfer *ft, TOX_FILE_CONTROL control)
{
    switch (control)
	{
        case TOX_FILE_CONTROL_RESUME:
            if (ft->state == FILE_TRANSFER_PENDING)
			{
                ft->state = FILE_TRANSFER_STARTED;
            }
			else if (ft->state == FILE_TRANSFER_PAUSED)
			{
                ft->state = FILE_TRANSFER_STARTED;
            }

            break;

        case TOX_FILE_CONTROL_PAUSE:
            ft->state = FILE_TRANSFER_PAUSED;
            break;

        case TOX_FILE_CONTROL_CANCEL:
            close_file_transfer(m, ft, -1);
            break;
    }
}


void on_file_control(Tox *m, uint32_t friendnumber, uint32_t filenumber, TOX_FILE_CONTROL control,
                     void *userdata)
{
    struct FileTransfer *ft = get_file_transfer_struct(friendnumber, filenumber);

    if (!ft)
    {
        return;
    }

    if (ft->file_type == TOX_FILE_KIND_AVATAR)
    {
        on_avatar_file_control(m, ft, control);
        return;
    }

    dbg(9, "on_file_control:002:file in/out\n");



	switch (control)
	{
		case TOX_FILE_CONTROL_RESUME:
		{
			dbg(9, "on_file_control:003:TOX_FILE_CONTROL_RESUME\n");

			ft->last_keep_alive = get_unix_time();

			/* transfer is accepted */
			if (ft->state == FILE_TRANSFER_PENDING)
			{
				ft->state = FILE_TRANSFER_STARTED;
				dbg(9, "on_file_control:004:pending -> started\n");
			}
			else if (ft->state == FILE_TRANSFER_PAUSED)
			{    /* transfer is resumed */
				ft->state = FILE_TRANSFER_STARTED;
				dbg(9, "on_file_control:005:paused -> started\n");
			}

			break;
		}

		case TOX_FILE_CONTROL_PAUSE:
		{
			dbg(9, "on_file_control:006:TOX_FILE_CONTROL_PAUSE\n");
			ft->state = FILE_TRANSFER_PAUSED;
			break;
		}

		case TOX_FILE_CONTROL_CANCEL:
		{
			dbg(1, "File transfer for '%s' was aborted\n", ft->file_name);
			close_file_transfer(m, ft, -1);
			break;
		}
	}

}



void on_avatar_chunk_request(Tox *m, struct FileTransfer *ft, uint64_t position, size_t length)
{
    dbg(9, "on_avatar_chunk_request:001\n");

    if (ft->state != FILE_TRANSFER_STARTED)
    {
        dbg(0, "on_avatar_chunk_request:001a:!FILE_TRANSFER_STARTED\n");
        return;
    }

    if (length == 0)
    {
        close_file_transfer(m, ft, -1);
        return;
    }

    if (ft->file == NULL)
	{
        close_file_transfer(m, ft, TOX_FILE_CONTROL_CANCEL);
        return;
    }

    if (ft->position != position)
	{
        if (fseek(ft->file, position, SEEK_SET) == -1)
		{
            close_file_transfer(m, ft, TOX_FILE_CONTROL_CANCEL);
            return;
        }

        ft->position = position;
    }

    uint8_t send_data[length];
    size_t send_length = fread(send_data, 1, sizeof(send_data), ft->file);

    if (send_length != length)
    {
        close_file_transfer(m, ft, TOX_FILE_CONTROL_CANCEL);
        return;
    }

    TOX_ERR_FILE_SEND_CHUNK err;
    tox_file_send_chunk(m, ft->friendnum, ft->filenum, position, send_data, send_length, &err);

    if (err != TOX_ERR_FILE_SEND_CHUNK_OK)
    {
        dbg(0, "tox_file_send_chunk failed in avatar callback (error %d)\n", err);
    }

    ft->position += send_length;
    ft->last_keep_alive = get_unix_time();
}


void self_connection_status_cb(Tox *tox, TOX_CONNECTION connection_status, void *user_data)
{
    switch (connection_status)
	{
        case TOX_CONNECTION_NONE:
            dbg(2, "Offline\n");
			my_connection_status = TOX_CONNECTION_NONE;
            break;
        case TOX_CONNECTION_TCP:
            dbg(2, "Online, using TCP\n");
			my_connection_status = TOX_CONNECTION_TCP;
            break;
        case TOX_CONNECTION_UDP:
            dbg(2, "Online, using UDP\n");
			my_connection_status = TOX_CONNECTION_UDP;
            break;
    }
}


static struct FileTransfer *new_file_sender(uint32_t friendnum, uint32_t filenum, uint8_t type)
{
    size_t i;

    dbg(9, "new_file_sender:001 friendnum=%d filenum=%d type=%d\n", (int)friendnum, (int) filenum, (int) type);
	int friendlistnum = find_friend_in_friendlist(friendnum);

    for (i = 0; i < MAX_FILES; ++i)
    {
        struct FileTransfer *ft = &Friends.list[friendlistnum].file_sender[i];
        dbg(9, "new_file_sender:002 i=%d\n", (int)i);

        if (ft->state == FILE_TRANSFER_INACTIVE)
        {
			dbg(9, "new_file_sender:003:reusing sender i=%d\n", (int)i);

            memset(ft, 0, sizeof(struct FileTransfer));
			// ft->state = FILE_TRANSFER_INACTIVE; // == 0

            ft->index = i;
            ft->friendnum = friendnum;
            ft->filenum = filenum;
            ft->file_type = type;
            ft->last_keep_alive = get_unix_time();
            ft->state = FILE_TRANSFER_PENDING;
            ft->direction = FILE_TRANSFER_SEND;

            dbg(9, "new_file_sender:003 i=%d\n", (int)i);

            return ft;
        }
    }

    return NULL;
}



static struct FileTransfer *new_file_receiver(uint32_t friendnum, uint32_t filenum, uint8_t type)
{
    size_t i;
	int friendlistnum = find_friend_in_friendlist(friendnum);

    for (i = 0; i < MAX_FILES; ++i)
    {
        struct FileTransfer *ft = &Friends.list[friendlistnum].file_receiver[i];

        if (ft->state == FILE_TRANSFER_INACTIVE) {
            memset(ft, 0, sizeof(struct FileTransfer));
			// ft->state = FILE_TRANSFER_INACTIVE; // == 0

            ft->index = i;
            ft->friendnum = friendnum;
            ft->filenum = filenum;
            ft->file_type = type;
            ft->last_keep_alive = get_unix_time();
            ft->state = FILE_TRANSFER_PENDING;
            ft->direction = FILE_TRANSFER_RECV;
            return ft;
        }
    }

    return NULL;
}


struct FileTransfer *new_file_transfer(uint32_t friendnum, uint32_t filenum,
                                       FILE_TRANSFER_DIRECTION direction, uint8_t type)
{
    if (direction == FILE_TRANSFER_RECV)
    {
        return new_file_receiver(friendnum, filenum, type);
    }

    if (direction == FILE_TRANSFER_SEND)
    {
        return new_file_sender(friendnum, filenum, type);
    }

    return NULL;
}


int avatar_send(Tox *m, uint32_t friendnum)
{
    dbg(2, "avatar_send:001 friendnum=%d\n", (int)friendnum);
    dbg(2, "avatar_send:002 %d %s %d\n", (int)Avatar.size, Avatar.name, (int)Avatar.name_len);

    TOX_ERR_FILE_SEND err;
    uint32_t filenum = tox_file_send(m, friendnum, TOX_FILE_KIND_AVATAR, (size_t) Avatar.size,
                                     NULL, (uint8_t *) Avatar.name, Avatar.name_len, &err);

    if (Avatar.size == 0)
    {
        return 0;
    }

    if (err != TOX_ERR_FILE_SEND_OK)
    {
        dbg(0, "tox_file_send failed for _friendnumber %d (error %d)\n", friendnum, err);
        return -1;
    }

    struct FileTransfer *ft = new_file_transfer(friendnum, filenum, FILE_TRANSFER_SEND, TOX_FILE_KIND_AVATAR);

    if (!ft)
    {
        dbg(0, "avatar_send:003:ft=NULL\n");
        return -1;
    }

    ft->file = fopen(Avatar.path, "r");

    if (ft->file == NULL)
    {
        dbg(0, "avatar_send:004:ft->file=NULL\n");
        return -1;
    }

    snprintf(ft->file_name, sizeof(ft->file_name), "%s", Avatar.name);
    ft->file_size = Avatar.size;

    return 0;
}


int check_file_signature(const char *signature, size_t size, FILE *fp)
{
    char buf[size];
    if (fread(buf, size, 1, fp) != 1)
    {
        return -1;
    }
    int ret = memcmp(signature, buf, size);
    if (fseek(fp, 0L, SEEK_SET) == -1)
    {
        return -1;
    }
    return ret == 0 ? 0 : 1;
}


void kill_all_file_transfers_friend(Tox *m, uint32_t friendnum)
{
    size_t i;
	int friendlistnum = find_friend_in_friendlist(friendnum);

    for (i = 0; i < MAX_FILES; ++i)
    {
        close_file_transfer(m, &Friends.list[friendlistnum].file_sender[i], TOX_FILE_CONTROL_CANCEL);
        close_file_transfer(m, &Friends.list[friendlistnum].file_receiver[i], TOX_FILE_CONTROL_CANCEL);
    }
}

void kill_all_file_transfers(Tox *m)
{
    size_t i;

    for (i = 0; i < Friends.max_idx; ++i)
    {
        kill_all_file_transfers_friend(m, Friends.list[i].num);
    }
}



int avatar_set(Tox *m, const char *path, size_t path_len)
{
    dbg(2, "avatar_set:001\n");

    if (path_len == 0 || path_len >= sizeof(Avatar.path))
    {
        return -1;
    }

    dbg(9, "avatar_set:002\n");
    FILE *fp = fopen(path, "rb");

    if (fp == NULL)
    {
        return -1;
    }

    char PNG_signature[8] = {0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A};

    if (check_file_signature(PNG_signature, sizeof(PNG_signature), fp) != 0)
    {
        fclose(fp);
        return -1;
    }
    fclose(fp);

    dbg(9, "avatar_set:003\n");

    off_t size = file_size(path);

    if (size == 0 || size > MAX_AVATAR_FILE_SIZE)
    {
        return -1;
    }

    dbg(9, "avatar_set:004\n");

    get_file_name(Avatar.name, sizeof(Avatar.name), path);
    Avatar.name_len = strlen(Avatar.name);
    snprintf(Avatar.path, sizeof(Avatar.path), "%s", path);
    Avatar.path_len = path_len;
    Avatar.size = size;

    dbg(9, "avatar_set:099\n");

    return 0;
}

static void avatar_clear(void)
{
    memset(&Avatar, 0, sizeof(struct Avatar));
}

void avatar_unset(Tox *m)
{
    avatar_clear();
}

int check_number_of_files_to_resend_to_friend(Tox *m, uint32_t friendnum, int friendlistnum)
{
	int ret = 0;

	DIR *d;
	struct dirent *dir;
	// dbg(2, "checking friend subdir=%s\n", Friends.list[friendlistnum].worksubdir);

	d = opendir(Friends.list[friendlistnum].worksubdir);
	if (d)
	{
		while ((dir = readdir(d)) != NULL)
		{
			if (dir->d_type == DT_REG)
			{
				const char *ext = strrchr(dir->d_name,'.');
				if((!ext) || (ext == dir->d_name))
				{
						// wrong fileextension
				}
				else
				{
					if (strcmp(ext, motion_capture_file_extension) == 0)
					{
						// pictures
						if (get_file_transfer_from_filename_struct(friendnum, dir->d_name) == NULL)
						{
							ret++;
						}
					}
					else if (strcmp(ext, motion_capture_file_extension_mov) == 0)
					{
						// videos
						if (get_file_transfer_from_filename_struct(friendnum, dir->d_name) == NULL)
						{
							ret++;
						}
					}
				}
			}
		}
	}

	return ret;
}

void process_friends_dir(Tox *m, uint32_t friendnum, int friendlistnum)
{
	int resend_files_once = 0;

	// now check if friend is online
	if (is_friend_online(m, friendnum) == 1)
	{

		int number_of_files_to_resend = check_number_of_files_to_resend_to_friend(m, friendnum, friendlistnum);
		if (number_of_files_to_resend > MAX_RESEND_FILE_BEFORE_ASK)
		{
			// dbg(9, "number_of_files_to_resend > MAX_RESEND_FILE_BEFORE_ASK\n");

			if (Friends.list[friendlistnum].waiting_for_answer == 2)
			{
				dbg(9, "waiting_for_answer == 2\n");

				if (strncmp(Friends.list[friendlistnum].last_answer, "y", 1) == 0)
				{
					dbg(9, "process_friends_dir:resend:got answer:y\n");

					Friends.list[friendlistnum].last_answer[0] = '\0';
					Friends.list[friendlistnum].waiting_for_answer = 0;
					resend_files_once = 1;
				}
				else if (strncmp(Friends.list[friendlistnum].last_answer, "n", 1) == 0)
				{
					Friends.list[friendlistnum].last_answer[0] = '\0';
					Friends.list[friendlistnum].waiting_for_answer = 0;
					resend_files_once = 0;

					dbg(9, "process_friends_dir:resend:got answer:*n*\n");
				}
				else
				{
					send_text_message_to_friend(m, friendnum, "resend %d files? [y/n]", number_of_files_to_resend);
					Friends.list[friendlistnum].last_answer[0] = '\0';
					Friends.list[friendlistnum].waiting_for_answer = 1; // set to "waiting for answer"
					return;
				}
			}
			else
			{
				if (Friends.list[friendlistnum].waiting_for_answer != 1)
				{
					send_text_message_to_friend(m, friendnum, "resend %d files? [y/n]", number_of_files_to_resend);
					Friends.list[friendlistnum].waiting_for_answer = 1; // set to "waiting for answer"
				}
				return;
			}

			if (resend_files_once != 1)
			{
				return;
			}
		}
		else
		{
			resend_files_once = 1;
		}

		// dbg(9, "resending ...\n");

		DIR *d;
		struct dirent *dir;
		// dbg(2, "checking friend subdir=%s\n", Friends.list[friendlistnum].worksubdir);

		d = opendir(Friends.list[friendlistnum].worksubdir);
		if (d)
		{
			while ((dir = readdir(d)) != NULL)
			{
				if (dir->d_type == DT_REG)
				{
					const char *ext = strrchr(dir->d_name,'.');
					if((!ext) || (ext == dir->d_name))
					{
							// wrong fileextension
					}
					else
					{
						if (strcmp(ext, motion_capture_file_extension) == 0)
						{
							// images only
							// dbg(9, "checking image:%s\n", dir->d_name);

							struct stat foo;
							time_t mtime;
							time_t time_now = time(NULL);

							char newname[300];
							snprintf(newname, sizeof(newname), "%s/%s", Friends.list[friendlistnum].worksubdir, dir->d_name);
							// dbg(9, "subdir=%s\n", newname);

							stat(newname, &foo);
							mtime = foo.st_mtime; /* seconds since the epoch */

							// see if we have reached max filetransfers
							if (has_reached_max_file_transfer_for_friend(friendnum) == 0)
							{
								// see if file is in use
								if ((mtime + seconds_since_last_mod) < time_now)
								{
									// now see if this file is somewhere in outgoing ft
									if (get_file_transfer_from_filename_struct(friendnum, dir->d_name) == NULL)
									{
										dbg(2, "resending file %s to friend %d\n", newname, friendnum);
										send_file_to_friend(m, friendnum, newname);
									}
								}
								else
								{
									// printf("new image:%s (still in use ...)\n", dir->d_name);
								}

							}
						}
						else if (strcmp(ext, motion_capture_file_extension_mov) == 0)
						{
							// avi videos only
							// dbg(9, "checking video:%s\n", dir->d_name);

							struct stat foo;
							time_t mtime;
							time_t time_now = time(NULL);

							char newname[300];
							snprintf(newname, sizeof(newname), "%s/%s", Friends.list[friendlistnum].worksubdir, dir->d_name);
							// dbg(9, "subdir=%s\n", newname);

							stat(newname, &foo);
							mtime = foo.st_mtime; /* seconds since the epoch */

							// see if we have reached max filetransfers
							if (has_reached_max_file_transfer_for_friend(friendnum) == 0)
							{
								// see if file is in use
								if ((mtime + seconds_since_last_mod) < time_now)
								{
									// now see if this file is somewhere in outgoing ft
									if (get_file_transfer_from_filename_struct(friendnum, dir->d_name) == NULL)
									{
										dbg(2, "resending file %s to friend %d\n", newname, friendnum);
										send_file_to_friend(m, friendnum, newname);
									}
								}
								else
								{
									// printf("new image:%s (still in use ...)\n", dir->d_name);
								}

							}
						}

					}
				}
			}

			closedir(d);
		}
	}
}

void check_friends_dir(Tox *m)
{
    size_t i;
    size_t numfriends = tox_self_get_friend_list_size(m);
	int j = -1;

    for (i = 0; i < numfriends; ++i)
    {
        // dbg(2, "checking friend %d subdir\n", (int)i);

		j = find_friend_in_friendlist((uint32_t) i);
		if (j > -1)
		{
			process_friends_dir(m, i, j);
		}
	}
}


void check_dir(Tox *m)
{
	DIR *d;
	struct dirent *dir;
	d = opendir(motion_pics_dir);
	if (d)
	{
		while ((dir = readdir(d)) != NULL)
		{
			if (dir->d_type == DT_REG)
			{
				const char *ext = strrchr(dir->d_name,'.');
				if((!ext) || (ext == dir->d_name))
				{
						// wrong fileextension
				}
				else
				{
					if(strcmp(ext, motion_capture_file_extension) == 0)
					{
						// dbg(9, "new image:%s\n", dir->d_name);

						// move file to work dir
						char oldname[300];
						snprintf(oldname, sizeof(oldname), "%s/%s", motion_pics_dir, dir->d_name);


						struct stat foo;
						time_t mtime;
						time_t time_now = time(NULL);

						stat(oldname, &foo);
						mtime = foo.st_mtime; /* seconds since the epoch */

						if ((mtime + seconds_since_last_mod) < time_now)
						{
								char newname[300];
								snprintf(newname, sizeof(newname), "%s/%s", motion_pics_work_dir, dir->d_name);

								dbg(2, "new image:%s\n", dir->d_name);
								dbg(2, "move %s -> %s\n", oldname, newname);
								int renname_err = rename(oldname, newname);
								dbg(2, "res=%d\n", renname_err);

								send_file_to_all_friends(m, newname, dir->d_name);
						}
						else
						{
								// printf("new image:%s (still in use ...)\n", dir->d_name);
						}
					}
					else if(strcmp(ext, motion_capture_file_extension_mov) == 0)
					{
						// printf("new image:%s\n", dir->d_name);

						// move file to work dir
						char oldname[300];
						snprintf(oldname, sizeof(oldname), "%s/%s", motion_pics_dir, dir->d_name);


						struct stat foo;
						time_t mtime;
						time_t time_now = time(NULL);

						stat(oldname, &foo);
						mtime = foo.st_mtime; /* seconds since the epoch */

						if ((mtime + seconds_since_last_mod) < time_now)
						{
								char newname[300];
								snprintf(newname, sizeof(newname), "%s/%s", motion_pics_work_dir, dir->d_name);

								dbg(2, "new movie:%s\n", dir->d_name);
								dbg(2, "move %s -> %s\n", oldname, newname);
								int renname_err = rename(oldname, newname);
								dbg(2, "res=%d\n", renname_err);

								send_file_to_all_friends(m, newname, dir->d_name);
						}
						else
						{
								// printf("new image:%s (still in use ...)\n", dir->d_name);
						}
					}
				}
			}
		}

		closedir(d);
	}
}


// ------------------ Tox AV stuff --------------------
// ------------------ Tox AV stuff --------------------
// ------------------ Tox AV stuff --------------------

static void t_toxav_call_cb(ToxAV *av, uint32_t friend_number, bool audio_enabled, bool video_enabled, void *user_data)
{
    dbg(9, "Handling CALL callback\n");
    ((CallControl *)user_data)->incoming = true;
}

static void t_toxav_call_state_cb(ToxAV *av, uint32_t friend_number, uint32_t state, void *user_data)
{
    dbg(9, "Handling CALL STATE callback: %d\n", state);
    ((CallControl *)user_data)->state = state;
}

static void t_toxav_bit_rate_status_cb(ToxAV *av, uint32_t friend_number,
                                       uint32_t audio_bit_rate, uint32_t video_bit_rate,
                                       void *user_data)
{
    dbg(2, "Suggested bit rates: audio: %d video: %d\n", audio_bit_rate, video_bit_rate);
}


static void t_toxav_receive_audio_frame_cb(ToxAV *av, uint32_t friend_number,
        int16_t const *pcm,
        size_t sample_count,
        uint8_t channels,
        uint32_t sampling_rate,
        void *user_data)
{
    // CallControl *cc = (CallControl *)user_data;
    // frame *f = (frame *)malloc(sizeof(uint16_t) + sample_count * sizeof(int16_t) * channels);
    // memcpy(f->data, pcm, sample_count * sizeof(int16_t) * channels);
    // f->size = sample_count;

    // pthread_mutex_lock(cc->arb_mutex);
    // free(rb_write(cc->arb, f));
    // pthread_mutex_unlock(cc->arb_mutex);
}


static void t_toxav_receive_video_frame_cb(ToxAV *av, uint32_t friend_number,
        uint16_t width, uint16_t height,
        uint8_t const *y, uint8_t const *u, uint8_t const *v,
        int32_t ystride, int32_t ustride, int32_t vstride,
        void *user_data)
{
    // ystride = abs(ystride);
    // ustride = abs(ustride);
    // vstride = abs(vstride);

    // uint16_t *img_data = (uint16_t *)malloc(height * width * 6);

    // unsigned long int i, j;

    // for (i = 0; i < height; ++i)
	// {
        // for (j = 0; j < width; ++j)
		// {
            // uint8_t *point = (uint8_t *) img_data + 3 * ((i * width) + j);
            // int yx = y[(i * ystride) + j];
            // int ux = u[((i / 2) * ustride) + (j / 2)];
            // int vx = v[((i / 2) * vstride) + (j / 2)];

            // point[0] = YUV2R(yx, ux, vx);
            // point[1] = YUV2G(yx, ux, vx);
            // point[2] = YUV2B(yx, ux, vx);
        // }
    // }


    // CvMat mat = cvMat(height, width, CV_8UC3, img_data);

    // CvSize sz;
    // sz.height = height;
    // sz.width = width;

    // IplImage *header = cvCreateImageHeader(sz, 1, 3);
    // IplImage *img = cvGetImage(&mat, header);
    // cvShowImage(vdout, img);
    // free(img_data);
}

void *thread_av(void *data)
{
    ToxAV *av = (ToxAV *) data;
	pthread_t id = pthread_self();
	pthread_mutex_t av_thread_lock;
	if (pthread_mutex_init(&av_thread_lock, NULL) != 0)
	{
		dbg(0, "Error creating av_thread_lock\n");
	}
	else
	{
		dbg(2, "av_thread_lock created successfully\n");
	}

	dbg(2, "AV Thread #%d starting\n", (int) id);

    while (true)
	{
        pthread_mutex_lock(&av_thread_lock);
        toxav_iterate(av);
		// printf("AV Thread #%d running ...\n", (int) id);
        pthread_mutex_unlock(&av_thread_lock);

        usleep(toxav_iteration_interval(av) * 1000);
    }
}


// ------------------ Tox AV stuff --------------------
// ------------------ Tox AV stuff --------------------
// ------------------ Tox AV stuff --------------------



int main()
{
	global_want_restart = 0;

    logfile = fopen(log_filename, "wb");
    setvbuf(logfile, NULL, _IONBF, 0);

    Tox *tox = create_tox();
	global_start_time = time(NULL);

    // create motion-capture-dir of not already there
    mkdir(motion_pics_dir, S_IRWXU | S_IRWXG); // og+rwx

    // create workdir of not already there
    mkdir(motion_pics_work_dir, S_IRWXU | S_IRWXG); // og+rwx

    const char *name = "Door";
    tox_self_set_name(tox, (uint8_t *)name, strlen(name), NULL);

    const char *status_message = "This is your Door";
    tox_self_set_status_message(tox, (uint8_t *)status_message, strlen(status_message), NULL);

    Friends.max_idx = 0;

    bootstrap(tox);

    print_tox_id(tox);

    // init callbacks ----------------------------------
    tox_callback_friend_request(tox, friend_request_cb);
    tox_callback_friend_message(tox, friend_message_cb);
    tox_callback_file_chunk_request(tox, on_file_chunk_request);
    tox_callback_self_connection_status(tox, self_connection_status_cb);
    tox_callback_friend_connection_status(tox, friendlist_onConnectionChange);
    tox_callback_file_recv_control(tox, on_file_control);
    tox_callback_file_recv(tox, on_file_recv);
    tox_callback_file_recv_chunk(tox, on_file_recv_chunk);
    // init callbacks ----------------------------------


    update_savedata_file(tox);
    load_friendlist(tox);

    char path[300];
    snprintf(path, sizeof(path), "%s", my_avatar_filename);
    int len = strlen(path) - 1;
    avatar_set(tox, path, len);

	long long unsigned int cur_time = time(NULL);
	uint8_t off = 1;
	while (1)
	{
        tox_iterate(tox, NULL);
        usleep(tox_iteration_interval(tox) * 1000);
        if (tox_self_get_connection_status(tox) && off)
		{
            dbg(2, "Tox online, took %llu seconds\n", time(NULL) - cur_time);
            off = 0;
			break;
        }
        c_sleep(20);
    }


    TOXAV_ERR_NEW rc;
    ToxAV *mytox_av = toxav_new(tox, &rc);
    assert(rc == TOXAV_ERR_NEW_OK);
	dbg(2, "new Tox AV\n");
	CallControl mytox_CC;
	memset(&mytox_CC, 0, sizeof(CallControl));

    // init AV callbacks -------------------------------
    toxav_callback_call(mytox_av, t_toxav_call_cb, &mytox_CC);
    toxav_callback_call_state(mytox_av, t_toxav_call_state_cb, &mytox_CC);
    toxav_callback_bit_rate_status(mytox_av, t_toxav_bit_rate_status_cb, &mytox_CC);
    toxav_callback_video_receive_frame(mytox_av, t_toxav_receive_video_frame_cb, &mytox_CC);
    toxav_callback_audio_receive_frame(mytox_av, t_toxav_receive_audio_frame_cb, &mytox_CC);
    // init AV callbacks -------------------------------


	// start toxav thread ------------------------------
	pthread_t tid[1];
    if (pthread_create(&(tid[0]), NULL, thread_av, (void *) mytox_av) != 0)
	{
        dbg(0, "AV Thread create failed\n");
	}
	else
	{
        dbg(2, "AV Thread successfully created\n");
	}

	// start toxav thread ------------------------------

    while (1)
    {
        tox_iterate(tox, NULL);
        usleep(tox_iteration_interval(tox) * 1000);

		if (global_want_restart == 1)
		{
			// need to restart me!
			break;
		}
		else
		{
			check_dir(tox);
			check_friends_dir(tox);
		}
    }


    kill_all_file_transfers(tox);
	toxav_kill(mytox_av);
    tox_kill(tox);

    if (logfile)
    {
        fclose(logfile);
        logfile = NULL;
    }

    return 0;
}
