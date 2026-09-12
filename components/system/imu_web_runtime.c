/**
 * @file imu_web_runtime.c
 * @brief IMU 状态、Wi-Fi、HTTP 与采样任务的分阶段实现
 */
#include "imu_web_runtime.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "imu963ra.h"
#include "imu963ra_attitude.h"
#include "nvs_flash.h"
#include "private_config.h"

#define WIFI_READY BIT0
#define SAMPLE_PERIOD_MS 10
#define CALIBRATION_SAMPLES 500
#define GYRO_WARMUP_MS 2000
#define IMU_TASK_PRIORITY 20
#define IMU_TASK_CORE 1

static const char *TAG = "imu_web";
static EventGroupHandle_t s_events;
static SemaphoreHandle_t s_lock;
static imu963ra_attitude_t s_attitude;
static imu963ra_sample_t s_sample;
static bool s_ready;
static bool s_state_initialized;
static bool s_wifi_initialized;
static bool s_web_initialized;
static bool s_sampling_started;

static const char INDEX_HTML[] =
"<!doctype html><html lang='zh-CN'><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>IMU963RA 姿态</title><style>"
":root{color-scheme:dark;font-family:Inter,system-ui,sans-serif}*{box-sizing:border-box}"
"body{margin:0;min-height:100vh;background:radial-gradient(circle at 50% 20%,#18334b,#071018 62%);color:#eaf7ff;display:grid;place-items:center}"
"main{width:min(920px,94vw);padding:28px}.top{display:flex;justify-content:space-between;align-items:end;border-bottom:1px solid #294457;padding-bottom:16px}"
"h1{margin:0;font-size:clamp(24px,4vw,40px)}#state{color:#55e6a5}.grid{display:grid;grid-template-columns:1.4fr 1fr;gap:18px;margin-top:18px}"
".card{background:#0d1d29cc;border:1px solid #294457;border-radius:18px;padding:20px;box-shadow:0 20px 50px #0007}"
"canvas{width:100%;height:390px;display:block}.angles{display:grid;gap:13px}.value{font:600 36px ui-monospace,monospace}.label{color:#91a9b8}"
".raw{margin-top:18px;font:14px ui-monospace,monospace;color:#a9c4d3;line-height:1.7}button{border:0;border-radius:10px;padding:11px 18px;background:#30b77b;color:white;font-weight:700;cursor:pointer}"
"@media(max-width:700px){.grid{grid-template-columns:1fr}canvas{height:300px}}"
"</style></head><body><main><div class='top'><div><div class='label'>ESP32-S3 · Hardware I²C</div><h1>IMU963RA 实时姿态</h1></div><div><span id='state'>连接中…</span> <button onclick='zero()'>当前姿态归零</button></div></div>"
"<div class='grid'><section class='card'><canvas id='view'></canvas></section><section class='card angles'>"
"<div><div class='label'>ROLL 横滚</div><div class='value' id='roll'>--°</div></div><div><div class='label'>PITCH 俯仰</div><div class='value' id='pitch'>--°</div></div><div><div class='label'>YAW 航向</div><div class='value' id='yaw'>--°</div></div>"
"<div class='raw' id='raw'>等待传感器…</div></section></div></main><script>"
"const c=document.querySelector('#view'),x=c.getContext('2d');let a={roll:0,pitch:0,yaw:0};"
"function R(p){let [X,Y,Z]=p,r=a.roll*Math.PI/180,q=a.pitch*Math.PI/180,y=a.yaw*Math.PI/180;let c1=Math.cos(r),s1=Math.sin(r),c2=Math.cos(q),s2=Math.sin(q),c3=Math.cos(y),s3=Math.sin(y);let y1=Y*c1-Z*s1,z1=Y*s1+Z*c1,x2=X*c2+z1*s2,z2=-X*s2+z1*c2;return[x2*c3-y1*s3,x2*s3+y1*c3,z2]}"
"function draw(){let d=devicePixelRatio||1,w=c.clientWidth,h=c.clientHeight;c.width=w*d;c.height=h*d;x.setTransform(d,0,0,d,0,0);x.clearRect(0,0,w,h);let v=[[-1,-.55,-1],[1,-.55,-1],[1,.55,-1],[-1,.55,-1],[-1,-.55,1],[1,-.55,1],[1,.55,1],[-1,.55,1]].map(p=>{let r=R(p),s=105/(3+r[2]);return[w/2+r[0]*s,h/2-r[1]*s]});let e=[[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]];x.strokeStyle='#55e6a5';x.lineWidth=3;e.forEach(([i,j])=>{x.beginPath();x.moveTo(...v[i]);x.lineTo(...v[j]);x.stroke()});x.fillStyle='#32a7ff66';x.beginPath();[4,5,6,7].forEach((i,n)=>n?x.lineTo(...v[i]):x.moveTo(...v[i]));x.closePath();x.fill();requestAnimationFrame(draw)}draw();"
"async function poll(){try{let d=await(await fetch('/api/attitude',{cache:'no-store'})).json();if(d.ready){a=d;['roll','pitch','yaw'].forEach(k=>document.querySelector('#'+k).textContent=d[k].toFixed(2)+'°');document.querySelector('#raw').innerHTML=`ACC g&nbsp; ${d.ax.toFixed(3)}, ${d.ay.toFixed(3)}, ${d.az.toFixed(3)}<br>GYRO °/s&nbsp; ${d.gx.toFixed(2)}, ${d.gy.toFixed(2)}, ${d.gz.toFixed(2)}<br>I²C&nbsp; 0x${d.address.toString(16).toUpperCase()}`;document.querySelector('#state').textContent='实时数据';}else document.querySelector('#state').textContent='静置校准中…';}catch(e){document.querySelector('#state').textContent='连接断开';}setTimeout(poll,50)}poll();"
"async function zero(){await fetch('/api/zero',{method:'POST'})}</script></body></html>";

/**
 * @brief 处理 Wi-Fi 连接、断线重连和 DHCP 地址获取事件
 * @param arg 用户上下文，当前未使用
 * @param base 事件类别
 * @param id 事件编号
 * @param data 事件附加数据
 */
static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_events, WIFI_READY);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = data;
        ESP_LOGI(TAG, "open http://" IPSTR "/", IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_events, WIFI_READY);
    }
}

/**
 * @brief 返回板端内置的简易状态首页
 * @param req HTTP 请求对象
 * @return esp_err_t HTTP 响应结果
 */
static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

/**
 * @brief 返回姿态、六轴和磁力计实时 JSON 数据
 * @param req HTTP 请求对象
 * @return esp_err_t HTTP 响应结果
 */
static esp_err_t attitude_handler(httpd_req_t *req)
{
    imu963ra_attitude_t attitude;
    imu963ra_sample_t sample;
    bool ready;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    attitude = s_attitude; sample = s_sample; ready = s_ready;
    xSemaphoreGive(s_lock);
    char json[384];
    snprintf(json, sizeof(json),
        "{\"ready\":%s,\"roll\":%.3f,\"pitch\":%.3f,\"yaw\":%.3f,"
        "\"qw\":%.6f,\"qx\":%.6f,\"qy\":%.6f,\"qz\":%.6f,"
        "\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,"
        "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,"
        "\"mx\":%.4f,\"my\":%.4f,\"mz\":%.4f,\"mag_valid\":%s,\"address\":%u}",
        ready ? "true" : "false", attitude.roll_deg, attitude.pitch_deg, attitude.yaw_deg,
        attitude.quaternion_w, attitude.quaternion_x, attitude.quaternion_y, attitude.quaternion_z,
        sample.accel_g[0], sample.accel_g[1], sample.accel_g[2],
        sample.gyro_dps[0], sample.gyro_dps[1], sample.gyro_dps[2],
        sample.mag_gauss[0], sample.mag_gauss[1], sample.mag_gauss[2],
        sample.mag_valid ? "true" : "false", imu963ra_address());
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json);
}

/**
 * @brief 将当前姿态设置为网页显示零点
 * @param req HTTP 请求对象
 * @return esp_err_t HTTP 响应结果
 */
static esp_err_t zero_handler(httpd_req_t *req)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    imu963ra_attitude_zero_current();
    imu963ra_attitude_get(&s_attitude);
    xSemaphoreGive(s_lock);
    return httpd_resp_sendstr(req, "ok");
}

/**
 * @brief 运行 100 Hz IMU 采样、静态零偏估计和姿态更新
 * @param arg FreeRTOS 任务参数，当前未使用
 * @note 磁场数据仅在 mag_valid 为真时参与融合以约束 yaw；磁力计不可用时
 *       自动退化为无磁力计解算，六轴姿态保持可用。
 */
static void imu_task(void *arg)
{
    float bias[3] = {0}, acc[3] = {0};
    unsigned valid_samples = 0;
    /* LSM6DSR output has a visible start-up transient.  Discard it before
     * estimating the zero-rate offset or yaw will inherit a false bias. */
    for (int elapsed = 0; elapsed < GYRO_WARMUP_MS; elapsed += SAMPLE_PERIOD_MS) {
        imu963ra_sample_t discard;
        (void)imu963ra_read(&discard);
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
    for (int n = 0; n < CALIBRATION_SAMPLES; ++n) {
        imu963ra_sample_t sample;
        if (imu963ra_read(&sample) == ESP_OK) {
            for (int i = 0; i < 3; ++i) { bias[i] += sample.gyro_dps[i]; acc[i] += sample.accel_g[i]; }
            valid_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
    if (valid_samples == 0) {
        ESP_LOGE(TAG, "gyro calibration failed: no valid samples");
        vTaskDelete(NULL);
    }
    for (int i = 0; i < 3; ++i) { bias[i] /= valid_samples; acc[i] /= valid_samples; }
    ESP_LOGI(TAG, "gyro bias (%u samples): %.4f, %.4f, %.4f dps",
             valid_samples, bias[0], bias[1], bias[2]);
    imu963ra_attitude_init_horizontal(acc[0], acc[1], acc[2]);
    imu963ra_attitude_set_gyro_bias_dps(bias[0], bias[1], bias[2]);
    int64_t previous = esp_timer_get_time();
    TickType_t wake = xTaskGetTickCount();
    uint32_t cycles = 0;
    float min_dt = 1.0f, max_dt = 0.0f, sum_dt = 0.0f;
    while (true) {
        imu963ra_sample_t sample;
        if (imu963ra_read(&sample) == ESP_OK) {
            int64_t now = esp_timer_get_time();
            float dt = (now - previous) / 1000000.0f;
            previous = now;
            if (dt < min_dt) min_dt = dt;
            if (dt > max_dt) max_dt = dt;
            sum_dt += dt;
            cycles++;
            xSemaphoreTake(s_lock, portMAX_DELAY);
            s_sample = sample;
            imu963ra_attitude_update_horizontal(sample.gyro_dps[0], sample.gyro_dps[1], sample.gyro_dps[2],
                                                sample.accel_g[0], sample.accel_g[1], sample.accel_g[2],
                                                sample.mag_gauss[0], sample.mag_gauss[1], sample.mag_gauss[2],
                                                sample.mag_valid, dt);
            imu963ra_attitude_get(&s_attitude);
            s_ready = true;
            xSemaphoreGive(s_lock);
            if (cycles == 500) {
                ESP_LOGI(TAG, "AHRS rate %.2f Hz, dt min/avg/max %.3f/%.3f/%.3f ms",
                         500.0f / sum_dt, min_dt * 1000.0f,
                         sum_dt * 2.0f, max_dt * 1000.0f);
                cycles = 0; min_dt = 1.0f; max_dt = 0.0f; sum_dt = 0.0f;
            }
        }
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

/**
 * @brief 初始化姿态共享状态、锁和网络事件同步对象
 * @return esp_err_t ESP_OK 表示基础服务状态初始化成功
 * @note 本函数不访问外设、不连接网络，也不创建周期任务。
 */
esp_err_t imu_runtime_service_init(void)
{
    if (s_state_initialized) return ESP_OK;
    s_lock = xSemaphoreCreateMutex();
    s_events = xEventGroupCreate();
    if (!s_lock || !s_events) return ESP_ERR_NO_MEM;
    imu963ra_attitude_reset();
    memset(&s_attitude, 0, sizeof(s_attitude));
    memset(&s_sample, 0, sizeof(s_sample));
    s_ready = false;
    s_state_initialized = true;
    return ESP_OK;
}

/**
 * @brief 初始化 NVS、网络栈和 Wi-Fi STA，并等待获得 IP 地址
 * @return esp_err_t ESP_OK 表示 Wi-Fi 服务可用
 * @note 必须在 imu_runtime_service_init 之后调用。
 */
esp_err_t wifi_service_init(void)
{
    if (!s_state_initialized) return ESP_ERR_INVALID_STATE;
    if (s_wifi_initialized) return ESP_OK;
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "erase NVS");
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "initialize NVS");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "initialize network stack");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "create event loop");
    if (!esp_netif_create_default_wifi_sta()) return ESP_ERR_NO_MEM;
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "initialize Wi-Fi driver");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL), TAG, "register Wi-Fi event");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL), TAG, "register IP event");
    wifi_config_t config = {0};
    strlcpy((char *)config.sta.ssid, APP_WIFI_SSID, sizeof(config.sta.ssid));
    strlcpy((char *)config.sta.password, APP_WIFI_PASSWORD, sizeof(config.sta.password));
    config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set Wi-Fi mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &config), TAG, "set Wi-Fi config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "start Wi-Fi");
    xEventGroupWaitBits(s_events, WIFI_READY, pdFALSE, pdTRUE, portMAX_DELAY);
    s_wifi_initialized = true;
    return ESP_OK;
}

/**
 * @brief 启动 HTTP 服务并注册首页、姿态数据和归零接口
 * @return esp_err_t ESP_OK 表示全部 HTTP 路由注册成功
 * @note 必须在状态服务和 Wi-Fi 服务初始化后调用。
 */
esp_err_t imu_web_service_init(void)
{
    if (!s_state_initialized || !s_wifi_initialized) return ESP_ERR_INVALID_STATE;
    if (s_web_initialized) return ESP_OK;
    httpd_handle_t server = NULL;
    httpd_config_t http_config = HTTPD_DEFAULT_CONFIG();
    ESP_RETURN_ON_ERROR(httpd_start(&server, &http_config), TAG, "start HTTP server");
    const httpd_uri_t index = {.uri = "/", .method = HTTP_GET, .handler = index_handler};
    const httpd_uri_t api = {.uri = "/api/attitude", .method = HTTP_GET, .handler = attitude_handler};
    const httpd_uri_t zero = {.uri = "/api/zero", .method = HTTP_POST, .handler = zero_handler};
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &index), TAG, "register index route");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &api), TAG, "register attitude route");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &zero), TAG, "register zero route");
    s_web_initialized = true;
    return ESP_OK;
}

/**
 * @brief 在独立核心上启动高优先级 IMU 周期采样任务
 * @return esp_err_t ESP_OK 表示任务创建成功
 */
esp_err_t imu_web_runtime_start_sampling(void)
{
    if (!s_state_initialized || !s_web_initialized) return ESP_ERR_INVALID_STATE;
    if (s_sampling_started) return ESP_OK;
    if (xTaskCreatePinnedToCore(imu_task, "imu_sample", 6144, NULL,
                                IMU_TASK_PRIORITY, NULL, IMU_TASK_CORE) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_sampling_started = true;
    return ESP_OK;
}
