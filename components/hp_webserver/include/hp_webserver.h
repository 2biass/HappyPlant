#ifndef _HP_HTTP_SERVER
#define _HP_HTTP_SERVER

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include "esp_http_server.h"
#include "esp_vfs.h"
#include "cJSON.h"
#include "hp_sensors.h"

/* Maximum chunksize for sending larger files */
#define MAX_CHUNKSIZE 1024
#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + 32)

/* Type for static server data */
typedef struct server_data
{
    /* A handle to the http-server */
    httpd_handle_t server;

    /* Our virtual file system base path for the server*/
    char vfs_base_path[ESP_VFS_PATH_MAX];
    
    /* We dont wan't to reallocate memory everytime we need to send data */
    char file_response_buffer[MAX_CHUNKSIZE];
} server_data_t;

/* Type for passing http server handle and socket fd to ws_async_send function */
typedef struct async_cb_data
{
    /* A handle to the http-server */
    httpd_handle_t server;

    /* The connection is maintained over an internal socket */
    int socket_fd;
} async_cb_data_t;

/* This function starts an HTTP-server on port 80 which files are served from the filepath "vfs_base_path"
 * The underlaying storage unit is a SD-card which is accessed through the SD-card driver */
httpd_handle_t start_webserver(const char* vfs_base_path);

/* This will register handler for websocket */
void setup_websocket(server_data_t* server_data);

/* This function will stop the HTTP-server */
void stop_webserver(httpd_handle_t server);

void context_free_func(void* ctx);

/* Uri-handlers. These are callback-function called from the underlaying HTTP-server.
 * Their scope is to handle incomming HTTP-requests */
esp_err_t root_get_handler(httpd_req_t *req);
esp_err_t logo_get_handler(httpd_req_t *req);
esp_err_t wsclient_get_handler(httpd_req_t *req);
esp_err_t ws_handler(httpd_req_t *req); /* This handles the websocket communication and triggers the async_send */

/* We want to send sensor data asynchronously. Therefore we implement these wrapper functions */
void ws_async_send(void *arg);
esp_err_t trigger_async_send(httpd_handle_t handle, httpd_req_t *req);

/* As some files are large we need to send them in chunks. This functions minimizes the length of code. */
esp_err_t file_send_chunks(httpd_req_t *req, const char* filepath, char* buffer);

#endif