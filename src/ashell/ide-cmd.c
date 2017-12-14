// Copyright (c) 2017, Intel Corporation.

// C includes
#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <malloc.h>

// Zephyr includes
#include <atomic.h>
#include <misc/printk.h>
#include <misc/reboot.h>
#include <zephyr/types.h>

#ifdef CONFIG_BOARD_ARDUINO_101
  #include <flash.h>
#endif

// JerryScript includes
#include "jerry-code.h"

// ZJS includes
#include "../zjs_util.h"
// #include "../zjs_util.h"

// Local includes
#include "ashell.h"
#include "file-utils.h"
#include "jerryscript-port.h"
#include "ide-comms.h"

/* ************************** Parsing **************************************
   A stream parser for arbitrarily long input sequences provided in chunks
   over successive invocation of the parser, through which parser state is
   maintained.
*/

typedef enum {
    CMD_INIT = 0,
    CMD_SAVE,
    CMD_RUN,
    CMD_STOP,
    CMD_LIST,
    CMD_CAT,
    CMD_MOVE,
    CMD_REMOVE,
    CMD_BOOT,
    CMD_REBOOT,
    CMD_MAX
} ide_cmd_t;

typedef void (*ide_cmd_handler_t)(char *buf, size_t len);

#define DECLARE_HANDLER(name) static void name (char *buf, size_t len)

DECLARE_HANDLER(ide_cmd_init);
DECLARE_HANDLER(ide_cmd_save);
DECLARE_HANDLER(ide_cmd_run);
DECLARE_HANDLER(ide_cmd_stop);
DECLARE_HANDLER(ide_cmd_list);
DECLARE_HANDLER(ide_cmd_cat);
DECLARE_HANDLER(ide_cmd_move);
DECLARE_HANDLER(ide_cmd_remove);
DECLARE_HANDLER(ide_cmd_boot);
DECLARE_HANDLER(ide_cmd_reboot);

static const struct {
    ide_cmd_t   id;
    char *cmd;
    ide_cmd_handler_t handler;
    size_t argc;
} cmd_arg_map[CMD_MAX + 1] =
{ //  index        command    handler        argc       length
    { CMD_INIT,    "init",    ide_cmd_init,    0 },  // 7
    { CMD_SAVE,    "save",    ide_cmd_save,    2 },  // 20 + data
    { CMD_RUN,     "run",     ide_cmd_run,     1 },  // 19
    { CMD_STOP,    "stop",    ide_cmd_stop,    0 },  // 7
    { CMD_LIST,    "ls",      ide_cmd_list,    0 },  // 5
    { CMD_CAT,     "cat",     ide_cmd_cat,     1 },  // 18
    { CMD_MOVE,    "mv",      ide_cmd_move,    2 },  // 31
    { CMD_REMOVE,  "rm",      ide_cmd_remove,  1 },  // 22
    { CMD_BOOT,    "boot",    ide_cmd_boot,    1 },  // 19
    { CMD_REBOOT,  "reboot",  ide_cmd_reboot,  0 },  // 9
    { CMD_MAX,     "none",    NULL,            0 }
};

#define CMD_MAX_LEN 6

// Parsing requires at least 10, preferably 34 bytes or larger rx buffer.
// Introducing parser states to handle re-enter parsing with succcessive
// input buffers.
typedef enum {
    IDLE = 0,
    PARSE_CMD,
    PARSE_ARG_NONE,
    PARSE_ARG_FILENAME,
    PARSE_ARG_STREAM
} parser_state_t;

// Parser context: state, command, open stream.
static struct {
    parser_state_t state;
    ide_cmd_t      cmd_id;  // command index held as state while parsing args
    fs_file_t     *stream_fp;  // kept here to be reset on error
} parser;

#if 0
#define IDE_REPLY(status, ...) do {\
    ide_send_message(cmd_arg_map[parser.cmd_id].cmd, status, __VA_ARGS__ );\
    if (!(status == NO_ERROR && parser.state == PARSE_ARG_STREAM)) \
        parser_reset();\
} while (0)
#endif

static void parser_reset()
{
    parser.state = IDLE;
    parser.cmd_id = CMD_MAX;
    if (parser.stream_fp != NULL) {
        fs_close_alloc(parser.stream_fp);
    }
    parser.stream_fp = NULL;
}

void parser_init()
{
    parser_reset();
    IDE_DBG("parser.state: %u\r\n", parser.state);
    IDE_DBG("parser.cmd_id: %u\r\n", parser.cmd_id);
    IDE_DBG("parser.cmd: %s\r\n", cmd_arg_map[parser.cmd_id].cmd);
}

bool match_preamble(char *buf)
{
    return parser.state == IDLE  && *buf == '{';
}

inline void skip_preamble(char **pbuf, size_t *plen)
{
    if (**pbuf == '{') {
        (*pbuf)++;
        (*plen)--;
    }
}

inline bool match_postamble(char *buf)
{
    return *buf == '}';
}

inline bool match_separator(char *buf)
{
    return *buf == ' ';
}

inline void skip_separator(char **pbuf, size_t *plen)
{
    while (**pbuf == ' ') {
        (*pbuf)++;
        (*plen)--;
    }
}

inline bool match_stream_start(char *buf)
{
    return *buf == '$';
}

inline void skip_stream_start(char **pbuf, size_t *plen)
{
    if (**pbuf == '$') {
        (*pbuf)++;
        (*plen)--;
    }
}

inline void skip_size(char **pbuf, size_t *plen, size_t size)
{
    (*pbuf) += size;
    (*plen) -= size;
}

inline bool match_stream_end(char *buf)
{
    return *buf == '#';
}

int ide_start_message(char *cmd)
{
    return ide_spool("{ \"reply\": \"%s\", \"data\": ", cmd);
}

int ide_end_message(int status)
{
    ide_spool(", \"status\": %d }", status);
    return ide_spool_flush();
}

int ide_reply(int status, char *message)
{
    ide_spool("{\"reply\": %s, \"status\":%d, \"data\": %s }",
              cmd_arg_map[parser.cmd_id].cmd, status, message);
    ide_spool_flush();
    if (!(status == NO_ERROR && parser.state == PARSE_ARG_STREAM))
        parser_reset();
    return -status;
}

static ide_cmd_t match_cmd(char *buf, size_t len) {
    int i;
    for (i = 0; i < CMD_MAX && strncmp(cmd_arg_map[i].cmd, buf, len); i++);
    return i;  // CMD_MAX means no match
}

// Consume a command from the input and write to the destination buffer.
static int parse_command(char *buf, size_t len)
{
    size_t size, id;
    char cmd_buf[CMD_MAX_LEN + 1];

    IDE_DBG("\r\nParsing command.");
    for (size = 0; len > 0 && size < CMD_MAX_LEN; len--) {
        if(match_separator(buf) || match_postamble(buf)) {
            cmd_buf[size] = '\0';
            id = match_cmd(cmd_buf, size);
            IDE_DBG("\r\nCommand %d: %s", id, cmd_buf);
            if (id == CMD_MAX) {
                return ide_reply(ERROR_INVALID_CMD, "\"Invalid command.\"");
            }
            parser.cmd_id = id;
            return size;
        }
        cmd_buf[size++] = *buf++;
    }
    return ide_reply(ERROR_INVALID_MSG, "\"Invalid message.\"");
}

// Check if the number of arguments left to be parsed matches with buffer.
static int check_argc(size_t argc, char *buf, size_t len)
{
    IDE_DBG("\r\nChecking %d argument(s).", argc);
    if (argc == 0 && !match_postamble(buf)) {
        return ide_reply(ERROR_UNEXPECTED_CHAR, "\"Postamble expected.\"");
    } else if (argc > 0) {
        if (match_postamble(buf)) {
          return ide_reply(ERROR_MISSING_ARG, "\"Missing argument.\"");
        } else if (!match_separator(buf)) {
            return ide_reply(ERROR_UNEXPECTED_CHAR, "\"Separator expected.\"");
        }
    }
    return NO_ERROR;
}

// Process a buffer received from WebUSB.
// May be called multiple times until a full message is received.
// Fetch the command, then invoke its handler that will do arg parsing.
void ide_parse(char *buf, size_t len) {
    IDE_DBG("\r\nEntering ide_parse...");

    if (len == 0 || buf == NULL) {
        IDE_DBG("\r\nParser received empty buffer.");
        return;
    } else if (buf[len] != '\0') {
        IDE_DBG("\r\nParser received invalid buffer.");
        return;
    }

    IDE_DBG("\r\nReceived %u bytes.", len);

    //IDE_DBG("\r\nParser state: %d\r\nParser cmd:%s", parser.state, cmd_arg_map[parser.cmd_id].cmd);

    int ret = 0;
    // Parser states allow resuming parsing with successive input buffers.
    // Since all commands fit in one input buffer, except save, only need to
    // handle idle and stream states as possible entry points.
    switch (parser.state) {
        case IDLE:  // entry point
            IDE_DBG("\r\nParser state: idle.");
            for(; len > 0 && !match_preamble(buf); len--, buf++);

            if (len == 0) {
                ide_reply(ERROR_INVALID_MSG, "\"No preamble found.\"");
                return;
            }
            skip_preamble(&buf, &len);
            parser.state = PARSE_CMD;  // doesn't need entrance point
            IDE_DBG("\r\nParser state: cmd.");
            if ((ret = parse_command(buf, len)) < 0) {
                return;  // errors already reported and parser already reset
            }
            skip_size(&buf, &len, ret);
            //skip_separator(&buf, &len);
            IDE_DBG("\r\nParser state: arg(s).");
            size_t argc = cmd_arg_map[parser.cmd_id].argc;
            if (argc == 0) {
                parser.state = PARSE_ARG_NONE;
                if ((ret = check_argc(argc, buf, len)) < 0) {
                    return;  // errors already reported and parser already reset
                }
            } else {
                parser.state = PARSE_ARG_FILENAME;
            }
            skip_separator(&buf, &len);
            // fall through
        case PARSE_ARG_NONE:
        case PARSE_ARG_FILENAME:
        case PARSE_ARG_STREAM:    // from save handler, re-entry point
            // Arguments parsing is deferred to the command handlers.
            IDE_DBG("\r\nDispatching command...");
            cmd_arg_map[parser.cmd_id].handler(buf, len);
        default:
            return;
    }
}

// Try to consume a filename from the input.
static int parse_filename(char *buf, size_t len, size_t pos)
{
    bool dot = false;
    char *cur = buf;
    int index, ext, ret = NO_ERROR;

    IDE_DBG("\r\nParsing filename len = %d >", len);
    for (index = 0, ext = 0; *cur && (index < len); cur++) {
        if (!match_separator(cur) && !match_postamble(cur)) {
            index++;
            // check 8.3 format
            if (dot) {
                ext++;
                if (ext > 3 || *cur == '.')
                    ret = -ERROR_INVALID_FILENAME;
            } else if (*cur == '.') {
                dot = true;
                if (index > 8)
                    ret = -ERROR_INVALID_FILENAME;
            }
            if (ret < 0 || !isprint((int)(*cur)))
            {
                *cur = '\0';
                break;
            }
        } else {  // separator or postamble
            if (!dot || (dot && ext < 1)) {
                break;
            }
            size_t argc = cmd_arg_map[parser.cmd_id].argc - pos;
            ret = check_argc(argc, cur, len - index);
            if (ret < 0)
                return ret;
            *cur++ = '\0';
            index++;
            IDE_DBG("Found filename >%s\r\n", buf);
            return index;
        }
    }
    return ide_reply(ERROR_INVALID_FILENAME, "\"Invalid filename.\"");
}

/* ************************** Commands ************************************* */

// Signal to the Web IDE the device is ready and supports which protocol.
static void ide_cmd_init(char *buf, size_t len)
{
    IDE_DBG("\r\nInvoking init...\r\n");
    ide_reply(NO_ERROR, "{ \"mode\": \"ide\", \"version\": \"0.0.1\"}");
}

// Save a [portion of a] program in a file with a given name.
// May be invoked multiple times. Leaves file open until end of stream received.
int save_stream(char *filename, char *buffer, size_t len)
{
    #define FILE_SPOOL_SIZE P_SPOOL_SIZE
    int ret = NO_ERROR;
    bool end = false;

    if (parser.state == PARSE_ARG_FILENAME) {  // first invocation
        IDE_DBG("\r\nSaving stream to %s\r\n", filename);
        parser.state = PARSE_ARG_STREAM;
        if (!match_stream_start(buffer)) {
            return ide_reply(ERROR_INVALID_STREAM, "\"Invalid stream start.\"");
        }
        parser.stream_fp = fs_open_alloc(filename, fs_exist(filename) ? "u" : "w+");
    }

    if (!parser.stream_fp || parser.state != PARSE_ARG_STREAM) {
        return ide_reply(ERROR_INVALID_STREAM, "\"Not in streaming mode.\"");
    }

    if(match_stream_end(buffer + len - 2) &&
       match_postamble(buffer + len - 1)) {
        len -= 2;  // TODO: use stream_end_size() + postamble_size()
        end = true;
    }

    if (len > 0) {
        skip_stream_start(&buffer, &len);
        if (fs_seek(parser.stream_fp, 0, SEEK_END) < 0) {
            return ide_reply(ERROR_FILE_WRITE, "\"Error appending to the file.\"");
        }
        ret = fs_write(parser.stream_fp, buffer, len);
        if (ret < 0) {
            return ide_reply(ERROR_FILE_WRITE, "\"Error writing file.\"");
        } else if (ret < len) {
            return ide_reply(ERROR_FILE_WRITE, "\"File system full.\"");
        }
    }

    if (end) {
        parser.state = IDLE;  // trigger parser state reset
        return ide_reply(NO_ERROR, "\"ok\"");
    }

    return ret;
}

// The handler for the save command, may be called multiple times.
static void ide_cmd_save(char *buf, size_t len)
{
    int ret = 0;
    char *filename = NULL;
    IDE_DBG("\r\nInvoking save...");

    switch (parser.state) {
        case PARSE_ARG_FILENAME:
            filename = buf;
            ret = parse_filename(buf, len, 1);  // file name is arg 1
            if (ret < 0) {
                return;
            }
            skip_size(&buf, &len, ret);
            // fall through
        case PARSE_ARG_STREAM:  // subsequent invocations fall here
            // save stream, skip file name + separator
            save_stream(filename, buf, len);
            return;
        default:
            ide_reply(ERROR_GENERIC, "\"Parser error.\"");
            return;
    }
}

// Run the given program file and send back the status.
static void ide_cmd_run(char *buf, size_t len)
{
    IDE_DBG("\r\nInvoking run...");
    char *filename = buf;
    int ret = parse_filename(buf, len, 1);
    if (ret < 0) {
        return;
    }
    skip_size(&buf, &len, ret);
    IDE_DBG("Filename: %s", filename);

    if (!fs_exist(filename)) {
        ide_reply(ERROR_FILE, "\"File not found.\"");
        return;
    }
    javascript_run_code(buf);
    ide_reply(NO_ERROR, "ok");
}

// Stop the currently run program and send the status.
static void ide_cmd_stop(char *buf, size_t len)
{
    IDE_DBG("\r\nInvoking stop...");
    javascript_stop();
    ide_reply(NO_ERROR, "ok");
}

// Send the list names of saved programs and sizes, e.g.
// [1234, "file1.ext"], [234, "file2.ext"]
static void ide_cmd_list(char *buf, size_t len)
{
    static struct fs_dirent entry;
    fs_dir_t dir;
    char *start_format = "{ \"name\": \"%s\", \"size\": %d }";
    char *format = ", { \"name\": \"%s\", \"size\": %d }";

    IDE_DBG("\r\nInvoking list...");

    if (fs_opendir(&dir, "")) {
        ide_reply(ERROR_DIR_OPEN, "\"Cannot open current directory.\"");
        return;
    }

    ide_start_message("list");
    ide_spool("[");  // data is an array of dictionaries
    for (u8_t first = 1; !fs_readdir(&dir, &entry) && entry.name[0];) {
        if (entry.type != FS_DIR_ENTRY_DIR) {
            char *p = entry.name;
            for (; *p; ++p) {
                *p = tolower((int)*p);
            }
            ide_spool(first ? start_format : format, entry.name, entry.size);
            first &= ~first;
            if (ide_spool_space() < 20)  // appr. one dir entry size
                ide_spool_flush();
        }
    }
    fs_closedir(&dir);
    ide_spool("]");  // end of data
    ide_end_message(NO_ERROR);
    parser_reset();
}

static void ide_cmd_cat(char *buf, size_t len)
{
    IDE_DBG("\r\nInvoking cat...");
    char *filename = buf;
    fs_file_t *file;
    int count, ret;

    if ((ret = parse_filename(buf, len, 1)) > 0) {
        skip_size(&buf, &len, ret);
        IDE_DBG("\r\nFilename: %s\r\n", filename);
        if (fs_exist(filename) &&
            (file = fs_open_alloc(filename, "r")) != NULL) {
            if (fs_size(file) == 0) {
                fs_close_alloc(file);
                ide_reply(ERROR_FILE, "\"Empty file.\"");
                return;
            }
            // send a separate message for stream start and stream end
            ide_start_message("cat");
            ide_spool("\"start\"");  // data
            ide_end_message(NO_ERROR);

            fs_seek(file, 0, SEEK_SET);
            do {
                count = fs_read(file, ide_spool_ptr(), ide_spool_space());
                ide_spool_adjust(count);
                ide_spool_flush();
            } while (count > 0);
            fs_close_alloc(file);
            ide_reply(NO_ERROR, "\"end\"");
            return;
        }
        ide_reply(ERROR_FILE_OPEN, "\"Cannot open file.\"");
    }
}


// Rename the given file and send back the status.
static void ide_cmd_move(char *buf, size_t len)
{
    parser.state = PARSE_ARG_FILENAME;
    char *source, *dest;

    IDE_DBG("\r\nInvoking move...");
    source = buf;
    int size1 = parse_filename(buf, len, 1);
    if (size1 < 0)
        return;  // error has already been handled and parser is reset
    skip_size(&buf, &len, size1);
    IDE_DBG("\r\nSource: %s", source);

    dest = buf;
    int size2 = parse_filename(buf, len, 2);
    if (size2 <= 0)
        return;  // error has already been handled and parser is reset
    skip_size(&buf, &len, size2);
    IDE_DBG("\r\nDest: %s", dest);

    if (!fs_exist(source)) {
        ide_reply(ERROR_FILE, "\"Source file not found.\"");
        return;
    }

    if (fs_exist(dest)) {
        ide_reply(ERROR_FILE, "\"Destination already exists.\"");
        return;
    }

    f_rename(source, dest);
    ide_reply(NO_ERROR, "ok");
}

// Remove the given file and send back the status.
static void ide_cmd_remove(char *buf, size_t len)
{
    char *name = buf;

    IDE_DBG("\r\nInvoking remove...");

    int size = parse_filename(buf, len, 1);
    if (size <= 0)
        return;  // error has already been handled and parser is reset
    skip_size(&buf, &len, size);
    IDE_DBG("File: %s", name);

    if (!fs_exist(name)) {
        ide_reply(ERROR_FILE, "\"File not found.\"");
        return;
    }

    if (fs_unlink(name) < 0) {
        ide_reply(ERROR_FILE, "\"Cannot remove file.\"");
        return;
    }

    ide_reply(NO_ERROR, "ok");
}

// set the given program file to be automatically run on reboot, then reboot
static void ide_cmd_boot(char *buf, size_t len)
{
    const char *BUILD_TIMESTAMP = __DATE__ " " __TIME__ "\n";

    IDE_DBG("\r\nInvoking boot...");

    int size = parse_filename(buf, len, 1);
    if (size <= 0)
        return;  // error has already been handled and parser is reset

    if (!fs_exist(buf)) {
        ide_reply(ERROR_FILE, "\"File not found\"");
        return;
    }

    fs_file_t *file = fs_open_alloc("boot.cfg", "w+");
    if (!file) {
        ide_reply(ERROR_FILE, "\"Cannot create boot.cfg.\"");
        return;
    }

    ssize_t written = fs_write(file, BUILD_TIMESTAMP,
                               strlen(BUILD_TIMESTAMP));
    written += fs_write(file, buf, strlen(buf));
    if (written <= 0) {
        fs_close_alloc(file);
        ide_reply(ERROR_FILE, "\"Cannot write boot.cfg.\"");
    }

    fs_close_alloc(file);
    ide_reply(NO_ERROR, "ok");
}

#ifdef CONFIG_BOARD_ARDUINO_101
  #include <flash.h>
#endif

#ifdef CONFIG_REBOOT
// TODO Waiting for patch https://gerrit.zephyrproject.org/r/#/c/3161/
  #ifdef CONFIG_BOARD_ARDUINO_101
      #include <qm_init.h>
  #endif
#endif

// reset automatic run, then reboot
static void ide_cmd_reboot(char *buf, size_t len)
{
    IDE_DBG("\r\nInvoking reboot...");
    ide_reply(NO_ERROR, "ok");
    #ifdef CONFIG_REBOOT
        // TODO Waiting for patch https://gerrit.zephyrproject.org/r/#/c/3161/
        #ifdef CONFIG_BOARD_ARDUINO_101
            QM_SCSS_PMU->rstc |= QM_COLD_RESET;
        #endif
    #endif
    sys_reboot(SYS_REBOOT_COLD);
}
