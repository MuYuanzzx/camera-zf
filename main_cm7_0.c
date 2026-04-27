#include "zf_common_headfile.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

#define MT9V03X_W 188
#define MT9V03X_H 120
#define IMAGE_SIZE (MT9V03X_W * MT9V03X_H)

#define BOUNDARY_NUM (MT9V03X_H * 2)

uint8_t xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
uint8_t xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

int16 PX = 0;
int16 PY = 0;

uint8_t is_beacon_detected = 0;
uint8_t image_copy[MT9V03X_H][MT9V03X_W];

// ===================== 参数区 =====================
#define GRAY_THRESH 110 // 灰度低于110置0
#define BIN_THRESH 35
#define AREA_MIN 5
#define K_ROT 0.1f
#define K_FWD 1.0f
#define K_TRANS 0.125f
#define PX_DEAD1 30
#define PX_DEAD2 4
#define PY_DEAD 10
#define VW_MAX 150
#define VX_MAX 200
#define VY_MAX 200
#define SEARCH_VW 70
#define LED1 P19_0
// =================================================

// 8邻域
const int8_t dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
const int8_t dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

typedef enum// 灯状态枚举
{
    BEACON_STATE_LOST,
    BEACON_STATE_ALIGNING,
    BEACON_STATE_TRACKING,
} BeaconState;

static BeaconState current_state = BEACON_STATE_LOST;

typedef enum
{
    LIGHT_TYPE_POINT, // 点状亮斑
    LIGHT_TYPE_LINE   // 线形灯
} LightType;

LightType current_light_type = LIGHT_TYPE_POINT; // 当前灯类型
int16_t direct_dx = 0;   // 方向灯x轴方向
// 连通域结构体
typedef struct
{
    int16_t cx, cy;
    int16_t minx, maxx, miny, maxy;
    uint32_t area;
    float max_ratio;
} Blob;

Blob blobs[8];
uint8_t blob_cnt = 0;

// 方向灯全局变量
float dir_led_angle = 0.0f;
int16_t dir_top_x, dir_top_y;
int16_t dir_bottom_x, dir_bottom_y;

uint8_t RxPacket[6];
int16_t SpeedPacket[3];

void SpeedPacket_to_RxPacket(void)
{
    int16_t speed_buf[3];
    for (int i = 0; i < 3; i++)
        speed_buf[i] = SpeedPacket[i];
    RxPacket[0] = (uint8_t)(speed_buf[0] & 0xFF);
    RxPacket[1] = (uint8_t)((speed_buf[0] >> 8) & 0xFF);
    RxPacket[2] = (uint8_t)(speed_buf[1] & 0xFF);
    RxPacket[3] = (uint8_t)((speed_buf[1] >> 8) & 0xFF);
    RxPacket[4] = (uint8_t)(speed_buf[2] & 0xFF);
    RxPacket[5] = (uint8_t)((speed_buf[2] >> 8) & 0xFF);
}

void SetCarSpeed(int16_t vx, int16_t vy, int16_t vw)
{
    SpeedPacket[0] = vx;
    SpeedPacket[1] = vy;
    SpeedPacket[2] = vw;
    SpeedPacket_to_RxPacket();
    uart_write_byte(UART_0, 0xAB);
    uart_write_buffer(UART_0, RxPacket, 6);
    uart_write_byte(UART_0, 0xBA);
}


// 寻找所有连通域
void find_all_blobs(void)
{
    uint8_t vis[MT9V03X_H][MT9V03X_W] = {0};
    blob_cnt = 0;

    for (int y = 0; y < MT9V03X_H; y++)
    {
        for (int x = 0; x < MT9V03X_W; x++)
        {
            if (image_copy[y][x] > BIN_THRESH && !vis[y][x] && blob_cnt < 8)
            {
                int stack_x[1024], stack_y[1024];
                int top = 0;
                stack_x[top] = x;
                stack_y[top] = y;
                top++;
                vis[y][x] = 1;

                int64_t sum_x = 0, sum_y = 0;
                uint32_t area = 0;
                int16_t minx = MT9V03X_W, maxx = 0, miny = MT9V03X_H, maxy = 0;

                while (top > 0)
                {
                    top--;
                    int cx = stack_x[top];
                    int cy = stack_y[top];

                    sum_x += cx;
                    sum_y += cy;
                    area++;
                    if (cx < minx)
                        minx = cx;
                    if (cx > maxx)
                        maxx = cx;
                    if (cy < miny)
                        miny = cy;
                    if (cy > maxy)
                        maxy = cy;

                    for (int k = 0; k < 8; k++)
                    {
                        int nx = cx + dx[k];
                        int ny = cy + dy[k];
                        if (nx >= 0 && nx < MT9V03X_W && ny >= 0 && ny < MT9V03X_H && !vis[ny][nx] && image_copy[ny][nx] > BIN_THRESH)
                        {
                            vis[ny][nx] = 1;
                            stack_x[top] = nx;
                            stack_y[top] = ny;
                            top++;
                        }
                    }
                }

                if (area < AREA_MIN)
                    continue;

                int16_t w = maxx - minx + 1;
                int16_t h = maxy - miny + 1;

                float max_val = (w > h) ? w : h;
                float min_val = (w < h) ? w : h;
                float max_ratio = max_val / min_val;

                blobs[blob_cnt].cx = sum_x / area;
                blobs[blob_cnt].cy = sum_y / area;
                blobs[blob_cnt].minx = minx;
                blobs[blob_cnt].maxx = maxx;
                blobs[blob_cnt].miny = miny;
                blobs[blob_cnt].maxy = maxy;
                blobs[blob_cnt].area = area;
                blobs[blob_cnt].max_ratio = max_ratio;
                blob_cnt++;
            }
        }
    }
}

// 【修正后】以图像竖直向下为0°，顺时针正、逆时针负，±90°
float calculate_vertical_angle(int16_t top_x, int16_t top_y, int16_t bottom_x, int16_t bottom_y)
{
    int16_t dx = bottom_x - top_x;
    int16_t dy = bottom_y - top_y;

    if (dx == 0 && dy == 0)
        return 0.0f;

    float rad = atan2f(dx, dy);
    float deg = rad * 180.0f / PI;

    if (deg > 90.0f)
        deg = 90.0f;
    if (deg < -90.0f)
        deg = -90.0f;

    return deg;
}

// 目标识别主函数
void find_bright_center(void)
{
    memset(xy_x2_boundary, 0, sizeof(xy_x2_boundary));
    memset(xy_y2_boundary, 0, sizeof(xy_y2_boundary));
    memset(xy_x3_boundary, 0, sizeof(xy_x3_boundary));
    memset(xy_y3_boundary, 0, sizeof(xy_y3_boundary));

    int cnt_red = 0;
    int cnt_yel = 0;

    dir_led_angle = 0.0f;
    dir_top_x = dir_top_y = dir_bottom_x = dir_bottom_y = -1;

    find_all_blobs();

    if (blob_cnt == 0)
    {
        for (int8_t dy = -1; dy <= 1; dy++)
        {
            for (int8_t dx = -1; dx <= 1; dx++)
            {
                int16_t x = MT9V03X_W / 2 + dx, y = MT9V03X_H / 2 + dy;
                if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_red < BOUNDARY_NUM)
                {
                    xy_x2_boundary[cnt_red] = x;
                    xy_y2_boundary[cnt_red] = y;
                    cnt_red++;
                }
            }
        }
        return;
    }

    int bar_idx = 0;
    float max_ratio = blobs[0].max_ratio;
    for (int i = 1; i < blob_cnt; i++)
    {
        if (blobs[i].max_ratio > max_ratio)
        {
            max_ratio = blobs[i].max_ratio;
            bar_idx = i;
        }
    }

    Blob bar_blob = blobs[bar_idx];
    int16_t cx = bar_blob.cx;
    int16_t cy = bar_blob.cy;
    int16_t minx = bar_blob.minx, maxx = bar_blob.maxx;
    int16_t miny = bar_blob.miny, maxy = bar_blob.maxy;

    float top_max_dist_sq = -1.0f;
    float bottom_max_dist_sq = -1.0f;

    for (int y = miny; y <= maxy; y++)
    {
        for (int x = minx; x <= maxx; x++)
        {
            if (image_copy[y][x] > BIN_THRESH)
            {
                float dist_sq = (x - cx) * (x - cx) + (y - cy) * (y - cy);
                if (y < cy)
                {
                    if (dist_sq > top_max_dist_sq)
                    {
                        top_max_dist_sq = dist_sq;
                        dir_top_x = x;
                        dir_top_y = y;
                    }
                }
                else if (y > cy)
                {
                    if (dist_sq > bottom_max_dist_sq)
                    {
                        bottom_max_dist_sq = dist_sq;
                        dir_bottom_x = x;
                        dir_bottom_y = y;
                    }
                }
            }
        }
    }

    if (dir_top_x != -1 && dir_bottom_x != -1)
    {
        dir_led_angle = calculate_vertical_angle(dir_top_x, dir_top_y, dir_bottom_x, dir_bottom_y);
    }

    {
        for (int8_t dy = -1; dy <= 1; dy++)
        {
            for (int8_t dx = -1; dx <= 1; dx++)
            {
                int16_t x = cx + dx, y = cy + dy;
                if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_yel < BOUNDARY_NUM)
                {
                    xy_x3_boundary[cnt_yel] = x;
                    xy_y3_boundary[cnt_yel] = y;
                    cnt_yel++;
                }
            }
        }
        if (dir_top_x != -1)
        {
            for (int8_t dy = -1; dy <= 1; dy++)
            {
                for (int8_t dx = -1; dx <= 1; dx++)
                {
                    int16_t x = dir_top_x + dx, y = dir_top_y + dy;
                    if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_yel < BOUNDARY_NUM)
                    {
                        xy_x3_boundary[cnt_yel] = x;
                        xy_y3_boundary[cnt_yel] = y;
                        cnt_yel++;
                    }
                }
            }
        }
        if (dir_bottom_x != -1)
        {
            for (int8_t dy = -1; dy <= 1; dy++)
            {
                for (int8_t dx = -1; dx <= 1; dx++)
                {
                    int16_t x = dir_bottom_x + dx, y = dir_bottom_y + dy;
                    if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_yel < BOUNDARY_NUM)
                    {
                        xy_x3_boundary[cnt_yel] = x;
                        xy_y3_boundary[cnt_yel] = y;
                        cnt_yel++;
                    }
                }
            }
        }
        if (bar_blob.maxx - bar_blob.minx > bar_blob.maxy - bar_blob.miny)
        {
            for (int8_t dy = -1; dy <= 1; dy++)
            {
                for (int8_t dx = -1; dx <= 1; dx++)
                {
                    int16_t x = minx + dx, y = cy + dy;
                    if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_yel < BOUNDARY_NUM)
                    {
                        xy_x3_boundary[cnt_yel] = x;
                        xy_y3_boundary[cnt_yel] = y;
                        cnt_yel++;
                    }
                }
            }
            for (int8_t dy = -1; dy <= 1; dy++)
            {
                for (int8_t dx = -1; dx <= 1; dx++)
                {
                    int16_t x = maxx + dx, y = cy + dy;
                    if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_yel < BOUNDARY_NUM)
                    {
                        xy_x3_boundary[cnt_yel] = x;
                        xy_y3_boundary[cnt_yel] = y;
                        cnt_yel++;
                    }
                }
            }
        }
        else
        {
            for (int8_t dy = -1; dy <= 1; dy++)
            {
                for (int8_t dx = -1; dx <= 1; dx++)
                {
                    int16_t x = cx + dx, y = miny + dy;
                    if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_yel < BOUNDARY_NUM)
                    {
                        xy_x3_boundary[cnt_yel] = x;
                        xy_y3_boundary[cnt_yel] = y;
                        cnt_yel++;
                    }
                }
            }
            for (int8_t dy = -1; dy <= 1; dy++)
            {
                for (int8_t dx = -1; dx <= 1; dx++)
                {
                    int16_t x = cx + dx, y = maxy + dy;
                    if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_yel < BOUNDARY_NUM)
                    {
                        xy_x3_boundary[cnt_yel] = x;
                        xy_y3_boundary[cnt_yel] = y;
                        cnt_yel++;
                    }
                }
            }
        }
    }

    for (int i = 0; i < blob_cnt; i++)
    {
        if (i == bar_idx)
            continue;
        Blob circle_blob = blobs[i];
        int16_t cx_circle = circle_blob.cx;
        int16_t cy_circle = circle_blob.cy;
        for (int8_t dy = -1; dy <= 1; dy++)
        {
            for (int8_t dx = -1; dx <= 1; dx++)
            {
                int16_t x = cx_circle + dx, y = cy_circle + dy;
                if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_red < BOUNDARY_NUM)
                {
                    xy_x2_boundary[cnt_red] = x;
                    xy_y2_boundary[cnt_red] = y;
                    cnt_red++;
                }
            }
        }
    }
}

void TrackBeacon()
{
    int16_t vx = 0, vy = 0, vw = 0;
    switch (current_state)
    {
    case BEACON_STATE_LOST:
        if (is_beacon_detected)
            current_state = (abs(PX) > PX_DEAD2) ? BEACON_STATE_ALIGNING : BEACON_STATE_TRACKING;
        break;
    case BEACON_STATE_ALIGNING:
        if (abs(PX) <= PX_DEAD2)
            current_state = BEACON_STATE_TRACKING;
        break;
    case BEACON_STATE_TRACKING:
        if (!is_beacon_detected)
            current_state = BEACON_STATE_LOST;
        break;
    }
    current_state = BEACON_STATE_TRACKING; // 暂时默认进入跟踪状态，仅调试
    switch (current_state)
    {
    case BEACON_STATE_LOST:
        vw = SEARCH_VW;
        vx = 0;
        vy = 0;
        break;
    case BEACON_STATE_ALIGNING:
        if (abs(PX) > PX_DEAD1)
            vw = PX > 0 ? 35 : -35;
        else if (abs(PX) > PX_DEAD2)
            vw = PX > 0 ? 15 : -15;
        else
            vw = 0;
        vx = 0;
        vy = 0;
        break;

    case BEACON_STATE_TRACKING: // 暂时默认进入跟踪状态，仅调试
        if (current_light_type == LIGHT_TYPE_LINE && abs(direct_dx) > 6)
        {
            vx = 0;
            vy = 0;
            // ===================== 修复2：显式浮点转整数（消除警告）=====================
            vw = (int16_t)(7.0f * fabs(direct_dx) + 50.0f);
            // ==========================================================================
            vw = (direct_dx > 0) ? -vw : vw;
        }
        else
        {
            vw = 0;
            if (abs(PY) > PY_DEAD)
            {
                // ===================== 修复2：显式浮点转整数（消除警告）=====================
                vx = (int16_t)(3.4f * fabs(PY) + 35.0f);
                // ==========================================================================
                vx = (PY > 0) ? -vx : vx;
            }
            if (abs(PX - (-35)) > PY_DEAD)
            {
                // ===================== 修复2：显式浮点转整数（消除警告）=====================
                vy = (int16_t)(3.4f * fabs(PX) + 35.0f);
                // ==========================================================================
                vy = (PX > 0) ? -vy : vy;
            }
        }
        break;

    default:
        current_state = BEACON_STATE_LOST;
        vw = SEARCH_VW;
        vx = 0;
        vy = 0;
    }
    SetCarSpeed(vx, vy, vw);
}

void Angle_Alignment()//
{
    int16_t vx = 0, vy = 0, vw = 0;
    if (  dir_led_angle   > 15 && dir_led_angle < 90)
    {
        vw = 35;
        vx = 0;
        vy = 0;
    }
    else if (dir_led_angle < 14 && dir_led_angle > 0)
    {
        vw = 10;
        vx = 0;
        vy = 0;
    }
    else if (dir_led_angle > 90 || dir_led_angle < 0)
    {
        vw = 0;
        vx = 0;
        vy = 0;
    }
    SetCarSpeed(vx, vy, vw);
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL); 

    while (mt9v03x_init())
    {
        gpio_toggle_level(LED1);
        system_delay_ms(500);
    }

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X, image_copy[0],
        MT9V03X_W, MT9V03X_H);

    seekfree_assistant_camera_boundary_config(
        XY_BOUNDARY, BOUNDARY_NUM,
        xy_x1_boundary, xy_x2_boundary, xy_x3_boundary,
        xy_y1_boundary, xy_y2_boundary, xy_y3_boundary);

    while (1)
    {
        if (mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;

            // 灰度小于110全部置0
            for (int y = 0; y < MT9V03X_H; y++)
            {
                for (int x = 0; x < MT9V03X_W; x++)
                {
                    uint8_t pix = mt9v03x_image[y][x];
                    image_copy[y][x] = (pix < GRAY_THRESH) ? 0 : pix;
                }
            }
            //memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);// 复制图像到image_copy
            // 调用方向检测
            // orient_detect(bright_center_x, bright_center_y);
            // seekfree_assistant_camera_send();
            find_bright_center();
            printf("Led_Angle: %.2f°\r\n", dir_led_angle);
            seekfree_assistant_camera_send();
            TrackBeacon();
        }
        system_delay_ms(1);
    }
}



   



/*#include "zf_common_headfile.h"
#include <stdint.h>
#include <string.h>
#include <math.h> // 修复：浮点运算必须加的头文件

#define INCLUDE_BOUNDARY_TYPE 3
#define BOUNDARY_NUM (MT9V03X_H * 2)

uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];
uint8 y1_boundary[MT9V03X_W], y2_boundary[MT9V03X_W], y3_boundary[MT9V03X_W];

uint8 image_copy[MT9V03X_H][MT9V03X_W];

// 线形灯点集存储
#define MAX_CONN_POINTS 256
static int16_t conn_x[MAX_CONN_POINTS];
static int16_t conn_y[MAX_CONN_POINTS];
static uint16_t conn_point_cnt = 0;

typedef struct
{
    int16_t x;
    int16_t y;
} Point;

// ===================== 修复1：新增缺失的枚举/全局变量（解决4个编译错误）=====================
typedef enum
{
    LIGHT_TYPE_POINT, // 点状亮斑
    LIGHT_TYPE_LINE   // 线形灯
} LightType;

LightType current_light_type = LIGHT_TYPE_POINT; // 当前灯类型
int16_t direct_dx = 0;                           // 线形灯方向差值
// =========================================================================================

#define LED1 P19_0

// 核心参数
#define WINDOW_SIZE 3
#define BRIGHT_THRESH 40
#define ROI_SIZE 41
#define AREA_THRESH 10
#define THRESH_RATIO 8

#define LOST_FRAME_THRESH 3
#define FILTER_LEN 5
int16_t PX_buf[FILTER_LEN] = {0};
int16_t PY_buf[FILTER_LEN] = {0};
uint8_t buf_idx = 0;

uint8_t is_beacon_detected = 0;
uint8_t lost_frame_cnt = 0;

int16 bright_center_x = MT9V03X_W / 2;
int16 bright_center_y = MT9V03X_H / 2;
int16 CenterX = MT9V03X_W / 2;
int16 CenterY = MT9V03X_H / 2;

int16 PX = 0;
int16 PY = 0;
uint32_t TTime = 0;
uint8_t Tflag = 0;
int8_t output_angle = 0;

// 修复：保留唯一的连通域分析函数（删除了重复定义）
void find_max_connected(int16 roi_x0, int16 roi_y0, int16 roi_x1, int16 roi_y1, uint8 thresh,
                        int16 *final_x, int16 *final_y)//
{
    uint8 visited[ROI_SIZE][ROI_SIZE] = {0};
    int16 stack_x[ROI_SIZE * ROI_SIZE] = {0};
    int16 stack_y[ROI_SIZE * ROI_SIZE] = {0};
    int8 dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    int8 dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

    uint32 max_area = 0;
    int64 sum_x = 0, sum_y = 0, sum_i = 0;
    conn_point_cnt = 0;

    for (int16 y_roi = 0; y_roi <= roi_y1 - roi_y0; y_roi++)
    {
        for (int16 x_roi = 0; x_roi <= roi_x1 - roi_x0; x_roi++)
        {
            int16 x = roi_x0 + x_roi;
            int16 y = roi_y0 + y_roi;

            if (visited[y_roi][x_roi] == 0 && image_copy[y][x] > thresh)
            {
                int16 top = 0;
                stack_x[top] = x_roi;
                stack_y[top] = y_roi;
                top++;
                visited[y_roi][x_roi] = 1;

                uint32 curr_area = 0;
                int64 curr_sum_x = 0, curr_sum_y = 0, curr_sum_i = 0;

                while (top > 0)
                {
                    top--;
                    int16 cx_roi = stack_x[top];
                    int16 cy_roi = stack_y[top];
                    int16 cx = roi_x0 + cx_roi;
                    int16 cy = roi_y0 + cy_roi;
                    uint8 pix = image_copy[cy][cx];

                    curr_area++;
                    curr_sum_x += (int64)cx * pix;
                    curr_sum_y += (int64)cy * pix;
                    curr_sum_i += pix;

                    // 保存连通域所有点（用于找最远两点）
                    if (conn_point_cnt < MAX_CONN_POINTS)
                    {
                        conn_x[conn_point_cnt] = cx;
                        conn_y[conn_point_cnt] = cy;
                        conn_point_cnt++;
                    }

                    for (int8 k = 0; k < 8; k++)
                    {
                        int16 nx_roi = cx_roi + dx[k];
                        int16 ny_roi = cy_roi + dy[k];
                        int16 nx = roi_x0 + nx_roi;
                        int16 ny = roi_y0 + ny_roi;

                        if (nx_roi >= 0 && nx_roi <= roi_x1 - roi_x0 &&
                            ny_roi >= 0 && ny_roi <= roi_y1 - roi_y0 &&
                            visited[ny_roi][nx_roi] == 0 &&
                            image_copy[ny][nx] > thresh)
                        {
                            visited[ny_roi][nx_roi] = 1;
                            stack_x[top] = nx_roi;
                            stack_y[top] = ny_roi;
                            top++;
                        }
                    }
                }

                if (curr_area > max_area && curr_area > AREA_THRESH)
                {
                    max_area = curr_area;
                    sum_x = curr_sum_x;
                    sum_y = curr_sum_y;
                    sum_i = curr_sum_i;
                }
            }
        }
    }

    if (sum_i > 0)
    {
        *final_x = (int16)(sum_x / sum_i);
        *final_y = (int16)(sum_y / sum_i);
    }
}

// 找线形灯的上下最远点
void find_line_endpoints(Point *top_point, Point *bottom_point)//
{
    if (conn_point_cnt < 2)
    {
        top_point->x = MT9V03X_W / 2;
        top_point->y = MT9V03X_H / 2;
        bottom_point->x = MT9V03X_W / 2;
        bottom_point->y = MT9V03X_H / 2;
        conn_point_cnt = 0;
        return;
    }

    int64_t sum_x = 0, sum_y = 0;
    for (uint16_t i = 0; i < conn_point_cnt; i++)
    {
        sum_x += conn_x[i];
        sum_y += conn_y[i];
    }
    float mx = (float)sum_x / conn_point_cnt;
    float my = (float)sum_y / conn_point_cnt;

    float cov_xx = 0, cov_xy = 0, cov_yy = 0;
    for (uint16_t i = 0; i < conn_point_cnt; i++)
    {
        float dx = (float)conn_x[i] - mx;
        float dy = (float)conn_y[i] - my;
        cov_xx += dx * dx;
        cov_xy += dx * dy;
        cov_yy += dy * dy;
    }

    float u, v;
    if (fabs(cov_xy) < 1e-6f)
    {
        u = (cov_xx > cov_yy) ? 1.0f : 0.0f;
        v = (cov_xx > cov_yy) ? 0.0f : 1.0f;
    }
    else
    {
        float sqrt_term = sqrtf((cov_xx - cov_yy) * (cov_xx - cov_yy) + 4 * cov_xy * cov_xy);
        u = 2 * cov_xy;
        v = (cov_yy - cov_xx) + sqrt_term;
        float len = sqrtf(u * u + v * v);
        u /= len;
        v /= len;
    }

    float max_proj = -1e9f, min_proj = 1e9f;
    uint16_t max_idx = 0, min_idx = 0;
    for (uint16_t i = 0; i < conn_point_cnt; i++)
    {
        float dx = (float)conn_x[i] - mx;
        float dy = (float)conn_y[i] - my;
        float proj = dx * u + dy * v;
        if (proj > max_proj)
        {
            max_proj = proj;
            max_idx = i;
        }
        if (proj < min_proj)
        {
            min_proj = proj;
            min_idx = i;
        }
    }

    Point p1 = {conn_x[max_idx], conn_y[max_idx]};
    Point p2 = {conn_x[min_idx], conn_y[min_idx]};
    if (p1.y < p2.y)
    {
        *top_point = p1;
        *bottom_point = p2;
    }
    else
    {
        *top_point = p2;
        *bottom_point = p1;
    }
    conn_point_cnt = 0;
}

void FilterPXPY()未用到
{
    PX_buf[buf_idx] = PX;
    PY_buf[buf_idx] = PY;
    buf_idx = (buf_idx + 1) % FILTER_LEN;
    int32_t px_sum = 0, py_sum = 0;
    for (int i = 0; i < FILTER_LEN; i++)
    {
        px_sum += PX_buf[i];
        py_sum += PY_buf[i];
    }
    PX = px_sum / FILTER_LEN;
    PY = py_sum / FILTER_LEN;
}

void CheckBeaconLost()未用到
{
    lost_frame_cnt++;
    if (lost_frame_cnt >= LOST_FRAME_THRESH)
    {
        bright_center_x = MT9V03X_W / 2;
        bright_center_y = MT9V03X_H / 2;
        PX = 0;
        PY = 0;
        is_beacon_detected = 0;
        lost_frame_cnt = LOST_FRAME_THRESH;
    }
}

void UpdateBeaconPos(int16_t x, int16_t y)未用到
{
    if (x < 0)
        x = 0;
    if (x >= MT9V03X_W)
        x = MT9V03X_W - 1;
    if (y < 0)
        y = 0;
    if (y >= MT9V03X_H)
        y = MT9V03X_H - 1;

    bright_center_x = x;
    bright_center_y = y;
    PY = -(bright_center_y - CenterY);
    PX = bright_center_x - CenterX;
    lost_frame_cnt = 0;
    is_beacon_detected = 1;
}

// 核心函数：亮斑检测 + 线形灯最远两点标记
void find_bright_center(void)
{
    int16 half_win = WINDOW_SIZE / 2;
    uint32 max_win_sum = 0;
    uint32 total_sum = 0;
    int16 temp_x = MT9V03X_W / 2;
    int16 temp_y = MT9V03X_H / 2;

    for (int16 y = 0; y < MT9V03X_H; y++)
    {
        for (int16 x = 0; x < MT9V03X_W; x++)
        {
            total_sum += image_copy[y][x];
            if (y >= half_win && y < MT9V03X_H - half_win && x >= half_win && x < MT9V03X_W - half_win)
            {
                uint32 win_sum = 0;
                for (int8 dy = -half_win; dy <= half_win; dy++)
                    for (int8 dx = -half_win; dx <= half_win; dx++)
                        win_sum += image_copy[y + dy][x + dx];
                if (win_sum > max_win_sum)
                {
                    max_win_sum = win_sum;
                    temp_x = x;
                    temp_y = y;
                }
            }
        }
    }

    uint32 avg_bright = total_sum / (MT9V03X_W * MT9V03X_H);
    uint32 avg_win_bright = max_win_sum / (WINDOW_SIZE * WINDOW_SIZE);
    uint32 bright_diff = avg_win_bright - avg_bright;

    // 清空边界
    memset(xy_x1_boundary, 0, sizeof(xy_x1_boundary));
    memset(xy_y1_boundary, 0, sizeof(xy_y1_boundary));
    memset(xy_x2_boundary, 0, sizeof(xy_x2_boundary));
    memset(xy_y2_boundary, 0, sizeof(xy_y2_boundary));
    memset(xy_x3_boundary, 0, sizeof(xy_x3_boundary));
    memset(xy_y3_boundary, 0, sizeof(xy_y3_boundary));

    if (bright_diff > BRIGHT_THRESH)
    {
        int16 roi_half = ROI_SIZE / 2;
        int16 roi_x0 = temp_x - roi_half;
        int16 roi_y0 = temp_y - roi_half;
        int16 roi_x1 = temp_x + roi_half;
        int16 roi_y1 = temp_y + roi_half;

        roi_x0 = (roi_x0 < 0) ? 0 : roi_x0;
        roi_y0 = (roi_y0 < 0) ? 0 : roi_y0;
        roi_x1 = (roi_x1 >= MT9V03X_W) ? (MT9V03X_W - 1) : roi_x1;
        roi_y1 = (roi_y1 >= MT9V03X_H) ? (MT9V03X_H - 1) : roi_y1;

        uint8 center_pix = image_copy[temp_y][temp_x];
        uint8 roi_thresh = (center_pix * THRESH_RATIO) / 10;

        find_max_connected(roi_x0, roi_y0, roi_x1, roi_y1, roi_thresh, &temp_x, &temp_y);
        bright_center_x = temp_x;
        bright_center_y = temp_y;
        UpdateBeaconPos(temp_x, temp_y);

        // 核心功能：获取线形灯上下最远点
        Point top_p, bottom_p;
        find_line_endpoints(&top_p, &bottom_p);
        direct_dx = top_p.x - bottom_p.x;

        // ===================== 修复3：赋值灯类型（匹配枚举使用）=====================
        current_light_type = LIGHT_TYPE_LINE;
        // ==========================================================================

        // 标记最远两点（逐飞助手第三种颜色）
        int16 cnt3 = 0;
        for (int8 dy = -1; dy <= 1 && cnt3 < BOUNDARY_NUM; dy++)
            for (int8 dx = -1; dx <= 1 && cnt3 < BOUNDARY_NUM; dx++)
            {
                int16 x = top_p.x + dx;
                int16 y = top_p.y + dy;
                if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H)
                {
                    xy_x3_boundary[cnt3] = x;
                    xy_y3_boundary[cnt3] = y;
                    cnt3++;
                }
            }
        for (int8 dy = -1; dy <= 1 && cnt3 < BOUNDARY_NUM; dy++)
            for (int8 dx = -1; dx <= 1 && cnt3 < BOUNDARY_NUM; dx++)
            {
                int16 x = bottom_p.x + dx;
                int16 y = bottom_p.y + dy;
                if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H)
                {
                    xy_x3_boundary[cnt3] = x;
                    xy_y3_boundary[cnt3] = y;
                    cnt3++;
                }
            }

        // 中心3×3标记
        int16 cnt2 = 0;
        for (int8 dy = -1; dy <= 1 && cnt2 < BOUNDARY_NUM; dy++)
            for (int8 dx = -1; dx <= 1 && cnt2 < BOUNDARY_NUM; dx++)
            {
                int16 x = bright_center_x + dx;
                int16 y = bright_center_y + dy;
                if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H)
                {
                    xy_x2_boundary[cnt2] = x;
                    xy_y2_boundary[cnt2] = y;
                    cnt2++;
                }
            }
    }
    else
    {
        bright_center_x = MT9V03X_W / 2;
        bright_center_y = MT9V03X_H / 2;
        CheckBeaconLost();
        int16 cnt = 0;
        for (int8 dy = -1; dy <= 1 && cnt < BOUNDARY_NUM; dy++)
            for (int8 dx = -1; dx <= 1 && cnt < BOUNDARY_NUM; dx++)
            {
                int16 x = bright_center_x + dx;
                int16 y = bright_center_y + dy;
                if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H)
                {
                    xy_x2_boundary[cnt] = x;
                    xy_y2_boundary[cnt] = y;
                    cnt++;
                }
            }
    }
}

// 修复：方向识别函数（删除死循环+坐标错误）
// 识别方向函数（IAR编译无错版）
int8_t orient_detect(int16_t orimid_x, int16_t orimid_y)
{
    output_angle = 0;

    // 直接使用已计算好的线形灯最远两点
    Point top_p, bottom_p;
    find_line_endpoints(&top_p, &bottom_p);

    // 角度计算（保留你原有逻辑）
    int16_t dx = top_p.x - bottom_p.x;
    int16_t dy = top_p.y - bottom_p.y;

    if (dx == 0)
    {
        output_angle = 0;
    }
    else if (dy == 0)
    {
        output_angle = dx > 0 ? 90 : -90;
    }
    else
    {
        int abs_dx = dx > 0 ? dx : -dx;
        int abs_dy = dy > 0 ? dy : -dy;
        float ratio = (float)abs_dx / abs_dy;

        if (ratio < 0.4142f)
        {
            output_angle = (int)(ratio * 54.3f + 0.5f);
        }
        else if (ratio < 1.0f)
        {
            output_angle = (int)(22.5f + (ratio - 0.4142f) * 38.3f + 0.5f);
        }
        else if (ratio < 2.4142f)
        {
            output_angle = 90 - (int)(22.5f + (1.0f / ratio - 0.4142f) * 38.3f + 0.5f);
        }
        else if (ratio < 11.430f)
        {
            output_angle = 90 - (int)((1.0f / ratio) * 54.3f + 0.5f);
        }
        else
        {
            output_angle = 90;
        }

        output_angle = dx > 0 ? output_angle : -output_angle;
    }

    printf("angle:%d\r\n", output_angle);
    return output_angle;
}*/
