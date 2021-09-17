#include "hp_webserver.h"

static const char *TAG = "HappyPlant http-server";
static server_data_t *server_data = NULL;

/* Start the server it */
httpd_handle_t start_webserver(const char* vfs_base_path)
{
    /* Allocate server data structure and zero it out */
    server_data = calloc(1, sizeof(server_data_t));
    if(server_data == NULL)
    {
        ESP_LOGI(TAG, "Error allocating server_data!");
        return NULL;
    }
    strlcpy(server_data->vfs_base_path, vfs_base_path, sizeof(server_data->vfs_base_path));

    /* Sets configuration to default stack-size, port, sockets etc. */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    /* Allow wildcards on root to make /index the general endpoint */

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK)
    {
        ESP_LOGI(TAG, "Error starting server!");
        return NULL;
    }

    /* Set URI handlers */
    ESP_LOGI(TAG, "Registering URI handlers");

    /* Uri structs binding uri, handler, http-method etc. together */
    httpd_uri_t root_get = {
        .uri        = "/index",
        .method     = HTTP_GET,
        .handler    = root_get_handler,
        .user_ctx   = server_data
    };
    httpd_register_uri_handler(server, &root_get);

    httpd_uri_t logo_get = {
        .uri        = "/logo.png",
        .method     = HTTP_GET,
        .handler    = logo_get_handler,
        .user_ctx   = server_data
    };
    httpd_register_uri_handler(server, &logo_get);

    httpd_uri_t wsclient_get = {
        .uri        = "/wsclient.js",
        .method     = HTTP_GET,
        .handler    = wsclient_get_handler,
        .user_ctx   = server_data
    };
    httpd_register_uri_handler(server, &wsclient_get);

    server_data->server = server;

    /* Setup a handle for the websocket connection */
    setup_websocket(server_data);

    return server;
}

void stop_webserver(httpd_handle_t server)
{
    httpd_stop(server);
    free(server_data);
}

void context_free_func(void* ctx)
{
    ESP_LOGI(TAG, "Freeing context");
    free(ctx);
}

/* Uri-handlers */
esp_err_t root_get_handler(httpd_req_t *req)
{
    /* Create session's context if not already available */
    if (! req->sess_ctx) {
        ESP_LOGI(TAG, "Allocating new session");
        req->sess_ctx = malloc(sizeof(int));
        req->free_ctx = context_free_func;
        *(int *)req->sess_ctx = 0;
    }
   
    /* Get the server data instance */
    server_data_t* user_data = (server_data_t*)req->user_ctx;

    /* Open and send the html file for "/index" */
    char filepath[FILE_PATH_MAX];
    const size_t base_pathlen = strlen(user_data->vfs_base_path);
    strcpy(filepath, user_data->vfs_base_path);
    strlcpy(filepath + base_pathlen, "/index.htm", strlen("/index.htm") + 1);

    if(file_send_chunks(req, filepath, user_data->file_response_buffer) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error sending logo in chunks");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t logo_get_handler(httpd_req_t *req)
{
    char buf[10];
    int  ret;
    /* Read data received in the request */
    ret = httpd_req_recv(req, buf, sizeof(buf));
    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
        httpd_resp_send_408(req);
        return ESP_FAIL;
    }
    /* Create session's context if not already available */
    if (! req->sess_ctx) {
        ESP_LOGI(TAG, "Allocating new session");
        req->sess_ctx = malloc(sizeof(int));
        req->free_ctx = context_free_func;
        *(int *)req->sess_ctx = 0;
    }
    
    /* Get the server data instance */
    server_data_t* user_data = (server_data_t*)req->user_ctx;

    /* Open and send the index html file */
    char filepath[FILE_PATH_MAX];
    const size_t base_pathlen = strlen(user_data->vfs_base_path);
    strcpy(filepath, user_data->vfs_base_path);
    strlcpy(filepath + base_pathlen, "/logo.png", strlen("/logo.png") + 1);

    /* Send the logo in chunks (image files are large) */
    if(file_send_chunks(req, filepath, user_data->file_response_buffer) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error sending logo in chunks");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t wsclient_get_handler(httpd_req_t *req)
{
    char buf[10];
    int  ret;
    /* Read data received in the request */
    ret = httpd_req_recv(req, buf, sizeof(buf));
    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
        httpd_resp_send_408(req);
        return ESP_FAIL;
    }
    /* Create session's context if not already available */
    if (! req->sess_ctx) {
        ESP_LOGI(TAG, "Allocating new session");
        req->sess_ctx = malloc(sizeof(int));
        req->free_ctx = context_free_func;
        *(int *)req->sess_ctx = 0;
    }

    /* Get the server data instance */
    server_data_t* user_data = (server_data_t*)req->user_ctx;

    /* Open and send the index html file */
    char filepath[FILE_PATH_MAX];
    const size_t base_pathlen = strlen(user_data->vfs_base_path);
    strcpy(filepath, user_data->vfs_base_path);
    strlcpy(filepath + base_pathlen, "/wsclient.js", strlen("/wsclient.js") + 1);

    /* Send the file in chunks */
    if(file_send_chunks(req, filepath, user_data->file_response_buffer) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error sending logo in chunks");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void setup_websocket(server_data_t* server_data)
{
    /* This uri is special since it maintains the websocket connection */
    httpd_uri_t ws = {
        .uri        = "/ws",
        .method     = HTTP_GET,
        .handler    = ws_handler,
        .user_ctx   = server_data,
        .is_websocket = true
    };
    httpd_register_uri_handler(server_data->server, &ws);
}

esp_err_t ws_handler(httpd_req_t *req)
{
    /* If the http-method is get the websocket handshake is completed */
    if(req->method == HTTP_GET)
    {
        ESP_LOGI(TAG, "Handshake done - Websocket connection open");
        return ESP_OK;
    }

    /* After the initial handshake we expect the client to send request data by sending "readall" */
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    /* Expect a frame and read its length setting max_len = 0 */
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get frame length");
        return ret;
    }
    uint8_t *buf = NULL;

    if (ws_pkt.len) {
        /* NULL termination as we are expecting a string */
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        /* Set length ws_pkt.len to receive the data */
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
    }

    /* If the payload is indeed a string saying "readall" then trigger the async send to send sensor data */
    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT &&
        strcmp((char*)ws_pkt.payload, "readall") == 0) {
        free(buf);
        return trigger_async_send(req->handle, req);
    }

    free(buf);
    return ESP_OK;
}

/* This function queues the websocket send to let the server handle it asynchronously */
esp_err_t trigger_async_send(httpd_handle_t handle, httpd_req_t *req)
{
    async_cb_data_t *async_arg = malloc(sizeof(async_cb_data_t));
    async_arg->server = req->handle;
    async_arg->socket_fd = httpd_req_to_sockfd(req);
    return httpd_queue_work(handle, ws_async_send, async_arg);
}

/* Send the sensor data readings */
void ws_async_send(void *arg)
{
    /* Fetch the async_cb_data from the argument */
    async_cb_data_t *async_arg = arg;
    httpd_handle_t server = async_arg->server;
    int socket_fd = async_arg->socket_fd;

    /* Read sensor values and package it into a json file */
    float temp = 0;
    float humid = 0;
    float lux = 0;
    i2c_GY21_read_temp(&temp);
    i2c_GY21_read_humid(&humid, temp);
    i2c_TSL2561_read_lux(&lux);

    char* json_unformatted = NULL;
    cJSON *sensors;
    sensors = cJSON_CreateObject();
    cJSON_AddNumberToObject(sensors, "temperature", temp);
    cJSON_AddNumberToObject(sensors, "humidity", humid);
    cJSON_AddNumberToObject(sensors, "light", lux);
    json_unformatted = cJSON_PrintUnformatted(sensors);

    /* Construct a frame and send the JSON file through the websocket */
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)json_unformatted;
    ws_pkt.len = strlen(json_unformatted);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    httpd_ws_send_frame_async(server, socket_fd, &ws_pkt);
    free(async_arg);
    free(json_unformatted);
}

esp_err_t file_send_chunks(httpd_req_t *req, const char* filepath, char* buffer)
{
    
    /* Read and send the requested file */
    FILE *fd = fopen(filepath, "rb");

    if(fd == NULL)
    {
        ESP_LOGE(TAG, "Could not open file: \"%s\"", filepath);
        httpd_resp_send_500(req);
    }
    else
    {   
        size_t chunksize;
        char* chunk = buffer;
        do
        {
            chunksize = fread(chunk, 1, MAX_CHUNKSIZE, fd);
            if(chunksize > 0)
            {
                /* Send the buffer contents in chunks */
                if(httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK)
                {
                    fclose(fd);
                    ESP_LOGE(TAG, "Error while sending file");
                    httpd_resp_sendstr_chunk(req, NULL);
                    httpd_resp_send_500(req);
                    return ESP_FAIL;
                } 
            }
        } while(chunksize != 0);
    }

    fclose(fd);
    /* Send NULL to signal HTTP response complete */
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}