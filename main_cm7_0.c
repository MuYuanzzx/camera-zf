/*********************************************************************************************************************
 * CYT4BB 逐飞总钻风 稳定亮斑中心检测 (Blob-based approach)
 * 功能：使用全域连通域分析识别小车线型灯（中心点+边缘两点）+ 追信标控制
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

#define MT9V03X_W 188
#define MT9V03X_H 120
#define IMAGE_SIZE (MT9V03X_W * MT9V03X_H)

#define BOUNDARY_NUM (MT9V03X_H * 2)
uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

uint8 image_copy[MT9V03X_H][MT9V03X_W];

// ===================== 参数区 =====================
#define GRAY_THRESH 110 // 灰度低于110置0
#define BIN_THRESH 35   // 二值化阈值
#define AREA_MIN 5      // 最小连通域面积
// =================================================

// 8邻域
const int8_t dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
const int8_t dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

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

// 方向灯全局变量（线型灯最远两端点）
float dir_led_angle = 0.0f;
int16_t dir_top_x, dir_top_y;
int16_t dir_bottom_x, dir_bottom_y;

#define LED1 P19_0

// 核心参数（保留原控制参数）
#define LOST_FRAME_THRESH 3
#define FILTER_LEN 5
int16_t PX_buf[FILTER_LEN] = {0};
int16_t PY_buf[FILTER_LEN] = {0};
uint8_t buf_idx = 0;

uint8_t is_beacon_detected = 0;
uint8_t lost_frame_cnt = 0;

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

int16 bright_center_x = MT9V03X_W / 2;
int16 bright_center_y = MT9V03X_H / 2;
int16 CenterX = MT9V03X_W / 2;
int16 CenterY = MT9V03X_H / 2;

int16 PX = 0;
int16 PY = 0;
uint32_t TTime = 0;
uint8_t Tflag = 0;
int8_t output_angle = 0;

// 线形灯方向差值（供TrackBeacon用）
int16_t direct_dx = 0;

typedef enum
{
    LIGHT_TYPE_POINT,
    LIGHT_TYPE_LINE
} LightType;

LightType current_light_type = LIGHT_TYPE_POINT;

typedef enum
{
    BEACON_STATE_LOST,
    BEACON_STATE_ALIGNING,
    BEACON_STATE_TRACKING,
} BeaconState;

static BeaconState current_state = BEACON_STATE_LOST;

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

void FilterPXPY()
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

void CheckBeaconLost()
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

void UpdateBeaconPos(int16_t x, int16_t y)
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
    current_state = BEACON_STATE_TRACKING;
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

    case BEACON_STATE_TRACKING:
        if (current_light_type == LIGHT_TYPE_LINE && abs(direct_dx) > 6)
        {
            vx = 0;
            vy = 0;
            vw = (int16_t)(7.0f * fabs(direct_dx) + 50.0f);
            vw = (direct_dx > 0) ? -vw : vw;
        }
        else
        {
            vw = 0;
            if (abs(PY) > PY_DEAD)
            {
                vx = (int16_t)(3.4f * fabs(PY) + 35.0f);
                vx = (PY > 0) ? -vx : vx;
            }
            if (abs(PX - (-35)) > PY_DEAD)
            {
                vy = (int16_t)(3.4f * fabs(PX) + 35.0f);
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

void Angle_Alignment()
{
    int16_t vx = 0, vy = 0, vw = 0;
    if (output_angle > 15 && output_angle < 90)
    {
        vw = 35;
        vx = 0;
        vy = 0;
    }
    else if (output_angle < 14 && output_angle > 0)
    {
        vw = 10;
        vx = 0;
        vy = 0;
    }
    else if (output_angle > 90 || output_angle < 0)
    {
        vw = 0;
        vx = 0;
        vy = 0;
    }
    SetCarSpeed(vx, vy, vw);
}

// ==================== 新识别代码（从main_cm7_0.c移植）====================

// 寻找所有连通域（全域扫描）
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

// 以图像竖直向下为0°，顺时针正、逆时针负，±90°
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

// 替换后的核心识别函数（使用全域连通域分析）
void find_bright_center(void)
{
    memset(xy_x2_boundary, 0, sizeof(xy_x2_boundary));
    memset(xy_y2_boundary, 0, sizeof(xy_y2_boundary));
    memset(xy_x3_boundary, 0, sizeof(xy_x3_boundary));
    memset(xy_y3_boundary, 0, sizeof(xy_y3_boundary));

    dir_led_angle = 0.0f;
    dir_top_x = dir_top_y = dir_bottom_x = dir_bottom_y = -1;
    direct_dx = 0;
    current_light_type = LIGHT_TYPE_POINT;

    find_all_blobs();

    // 无连通域 → 信标丢失
    if (blob_cnt == 0)
    {
        bright_center_x = MT9V03X_W / 2;
        bright_center_y = MT9V03X_H / 2;
        CheckBeaconLost();

        int cnt = 0;
        for (int8_t dy = -1; dy <= 1 && cnt < BOUNDARY_NUM; dy++)
            for (int8_t dx = -1; dx <= 1 && cnt < BOUNDARY_NUM; dx++)
            {
                int16_t x = bright_center_x + dx, y = bright_center_y + dy;
                if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H)
                {
                    xy_x2_boundary[cnt] = x;
                    xy_y2_boundary[cnt] = y;
                    cnt++;
                }
            }
        return;
    }

    // 找到长宽比最大的连通域（即线型灯）
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

    // 在连通域内找距离中心最远的上下两点（线型灯两端）
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

    // 计算线型灯角度和方向差（供TrackBeacon使用）
    if (dir_top_x != -1 && dir_bottom_x != -1)
    {
        dir_led_angle = calculate_vertical_angle(dir_top_x, dir_top_y, dir_bottom_x, dir_bottom_y);
        direct_dx = dir_top_x - dir_bottom_x;
        current_light_type = LIGHT_TYPE_LINE;
    }

    // 更新信标位置（供控制使用）
    bright_center_x = cx;
    bright_center_y = cy;
    UpdateBeaconPos(cx, cy);

    // 标记中心点3×3到xy_x2_boundary（红色）
    int16 cnt_red = 0;
    for (int8_t dy = -1; dy <= 1; dy++)
        for (int8_t dx = -1; dx <= 1; dx++)
        {
            int16_t x = cx + dx, y = cy + dy;
            if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_red < BOUNDARY_NUM)
            {
                xy_x2_boundary[cnt_red] = x;
                xy_y2_boundary[cnt_red] = y;
                cnt_red++;
            }
        }

    // 标记线型灯两端点3×3到xy_x3_boundary（第三种颜色）
    int16 cnt3 = 0;
    if (dir_top_x != -1)
    {
        for (int8_t dy = -1; dy <= 1 && cnt3 < BOUNDARY_NUM; dy++)
            for (int8_t dx = -1; dx <= 1 && cnt3 < BOUNDARY_NUM; dx++)
            {
                int16_t x = dir_top_x + dx, y = dir_top_y + dy;
                if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H)
                {
                    xy_x3_boundary[cnt3] = x;
                    xy_y3_boundary[cnt3] = y;
                    cnt3++;
                }
            }
    }
    if (dir_bottom_x != -1)
    {
        for (int8_t dy = -1; dy <= 1 && cnt3 < BOUNDARY_NUM; dy++)
            for (int8_t dx = -1; dx <= 1 && cnt3 < BOUNDARY_NUM; dx++)
            {
                int16_t x = dir_bottom_x + dx, y = dir_bottom_y + dy;
                if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H)
                {
                    xy_x3_boundary[cnt3] = x;
                    xy_y3_boundary[cnt3] = y;
                    cnt3++;
                }
            }
    }
}

// ===================================================================

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

            // 灰度低于GRAY_THRESH全部置0（blob检测的前处理）
            for (int y = 0; y < MT9V03X_H; y++)
            {
                for (int x = 0; x < MT9V03X_W; x++)
                {
                    uint8_t pix = mt9v03x_image[y][x];
                    image_copy[y][x] = (pix < GRAY_THRESH) ? 0 : pix;
                }
            }

            find_bright_center();
            TrackBeacon();
        }
        system_delay_ms(1);
    }
}
