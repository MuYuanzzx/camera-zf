#include "zf_common_headfile.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

#define MT9V03X_W 188
#define MT9V03X_H 120
#define IMAGE_SIZE (MT9V03X_W * MT9V03X_H)

#define BOUNDARY_NUM (MT9V03X_H * 2)

int16 bright_center_x = MT9V03X_W / 2;
int16 bright_center_y = MT9V03X_H / 2;
int16 CenterX = MT9V03X_W / 2;
int16 CenterY = MT9V03X_H / 2;

uint8_t xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
uint8_t xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

int16 PX = 0;
int16 PY = 0;

uint8_t is_beacon_detected = 0;
uint8_t is_fly_beacon_detected = 0; // 飞控追踪的信标灯是否检测到

// 信标灯中心坐标（图像坐标系：左上角(0,0)，x向右，y向下）
int16_t beacon_cx = -1; // 选中的信标灯中心 x（像素坐标 [0,187]）
int16_t beacon_cy = -1; // 选中的信标灯中心 y（像素坐标 [0,119]）

// 信标灯相对于图像中心的偏移量（与 PX/PY 类似，但用于飞控）
int16_t beacon_PX = 0; // beacon_cx - CenterX（右为正）
int16_t beacon_PY = 0; // -(beacon_cy - CenterY)（上为正）

uint8_t image_copy[MT9V03X_H][MT9V03X_W];

// ===================== 参数区 =====================
#define GRAY_THRESH 110
#define BIN_THRESH 35
#define AREA_MIN 5
#define PX_DEAD1 30
#define PX_DEAD2 4
#define PY_DEAD 10
#define SEARCH_VW 70
#define LED1 P19_0
// =================================================

// 8邻域
const int8_t dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
const int8_t dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

typedef enum
{
    BEACON_STATE_LOST,
    BEACON_STATE_ALIGNING,
    BEACON_STATE_TRACKING,
} BeaconState;

static BeaconState current_state = BEACON_STATE_LOST;

int16_t direct_dx = 0;

typedef struct
{
    int16_t cx, cy;
    int16_t minx, maxx, miny, maxy;
    uint32_t area;
    float max_ratio;
    uint32_t sum_pixel; // 像素亮度总和（用于判断最亮的信标灯）
} Blob;

Blob blobs[8];
uint8_t blob_cnt = 0;

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

// ===================== 飞控通讯 =====================
// 飞控与小车共用同一个串口 UART_0（仅帧格式不同用于区分）
// 小车帧格式：0xAB + 6字节 + 0xBA
// 飞控帧格式：0xFF 0xFC + 6字节(3xint16小端) + 1字节校验和
#define FLY_CONTROL_UART UART_0

// 飞控接收解析函数参考：flymaple_sdk.c -> sdk_data_receive_prepare_2()
// 帧格式: 0xFF 0xFC + 6字节数据(3个int16小端) + 1字节校验和(最低位之和)
static uint8_t FlyTxPacket[9];

static void FlyPacket_Checksum(uint8_t *packet, const int16_t speed[3])
{
    // 帧头
    packet[0] = 0xFF;
    packet[1] = 0xFC;
    // 数据：小端模式 (低位在前)
    packet[2] = (uint8_t)(speed[0] & 0xFF);
    packet[3] = (uint8_t)((speed[0] >> 8) & 0xFF);
    packet[4] = (uint8_t)(speed[1] & 0xFF);
    packet[5] = (uint8_t)((speed[1] >> 8) & 0xFF);
    packet[6] = (uint8_t)(speed[2] & 0xFF);
    packet[7] = (uint8_t)((speed[2] >> 8) & 0xFF);
    // 校验：三个int16最低位之和
    packet[8] = (speed[0] & 0x01) + (speed[1] & 0x01) + (speed[2] & 0x01);
}

/**
 * @brief  通过串口发送速度指令给飞控（无名创新枫叶飞控）
 * @param  vx  前向速度 (cm/s)
 * @param  vy  侧向速度 (cm/s)
 * @param  vw  偏航角速度 (deg/s)
 * @note   飞控端由 UART5_IRQHandler -> sdk_data_receive_prepare_2() 解析
 */
void SetFlySpeed(int16_t vx, int16_t vy, int16_t vw)
{
    int16_t speed_buf[3] = {vx, vy, vw};
    FlyPacket_Checksum(FlyTxPacket, speed_buf);
    uart_write_buffer(FLY_CONTROL_UART, FlyTxPacket, 9);
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
}

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
                uint32_t sum_pixel = 0;
                int16_t minx = MT9V03X_W, maxx = 0, miny = MT9V03X_H, maxy = 0;

                while (top > 0)
                {
                    top--;
                    int cx = stack_x[top];
                    int cy = stack_y[top];

                    sum_x += cx;
                    sum_y += cy;
                    area++;
                    sum_pixel += image_copy[cy][cx];
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
                blobs[blob_cnt].sum_pixel = sum_pixel;
                blob_cnt++;
            }
        }
    }
}

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

    is_beacon_detected = 0;
    is_fly_beacon_detected = 0;

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

    is_beacon_detected = 1;

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

    UpdateBeaconPos(cx, cy);

    // ===================== 修复1：完善端点检测逻辑 =====================
    float top_max_dist_sq = -1.0f;
    float bottom_max_dist_sq = -1.0f;

    // 1. 优先尝试垂直方向寻找端点
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

    direct_dx = dir_top_x - dir_bottom_x;

    // 2. 如果垂直方向没找到（如水平灯），则找水平方向最左/最右点
    if (dir_top_x == -1 || dir_bottom_x == -1)
    {
        int16_t left_x = cx, left_y = cy;
        int16_t right_x = cx, right_y = cy;
        int16_t min_x_val = MT9V03X_W, max_x_val = 0;

        for (int y = miny; y <= maxy; y++)
        {
            for (int x = minx; x <= maxx; x++)
            {
                if (image_copy[y][x] > BIN_THRESH)
                {
                    if (x < min_x_val)
                    {
                        min_x_val = x;
                        left_x = x;
                        left_y = y;
                    }
                    if (x > max_x_val)
                    {
                        max_x_val = x;
                        right_x = x;
                        right_y = y;
                    }
                }
            }
        }
        dir_top_x = left_x;
        dir_top_y = left_y;
        dir_bottom_x = right_x;
        dir_bottom_y = right_y;
    }
    // =========================================================================

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

    // 从圆形信标灯中选出最亮的（飞控追踪目标）
    // 如果有多个亮度接近的（最高亮度的90%以上），选最接近图像中心的
    is_fly_beacon_detected = 0;
    beacon_cx = -1;
    beacon_cy = -1;
    {
        int16_t fly_choice_idx = -1;
        uint32_t max_sum_pixel = 0;

        // 第一轮：找最大亮度
        for (int i = 0; i < blob_cnt; i++)
        {
            if (i == bar_idx)
                continue;
            if (blobs[i].sum_pixel > max_sum_pixel)
                max_sum_pixel = blobs[i].sum_pixel;
        }

        if (max_sum_pixel > 0)
        {
            uint32_t brightness_thresh = (max_sum_pixel * 90) / 100; // 最高亮度的90%
            uint32_t min_dist_sq = (uint32_t)-1;

            // 第二轮：在亮度 >= threshold 的 blob 中选最近图像中心的
            for (int i = 0; i < blob_cnt; i++)
            {
                if (i == bar_idx)
                    continue;
                if (blobs[i].sum_pixel >= brightness_thresh)
                {
                    int16_t dx_fly = blobs[i].cx - CenterX;
                    int16_t dy_fly = blobs[i].cy - CenterY;
                    uint32_t dist_sq = (uint32_t)(dx_fly * dx_fly + dy_fly * dy_fly);
                    if (dist_sq < min_dist_sq)
                    {
                        min_dist_sq = dist_sq;
                        fly_choice_idx = i;
                    }
                }
            }
        }

        if (fly_choice_idx >= 0)
        {
            is_fly_beacon_detected = 1;
            beacon_cx = blobs[fly_choice_idx].cx;
            beacon_cy = blobs[fly_choice_idx].cy;
            beacon_PX = beacon_cx - CenterX;    // 右为正
            beacon_PY = -(beacon_cy - CenterY); // 上为正
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
/**
 * @brief  小车跟随飞机（原 TrackBeacon 的 BEACON_STATE_TRACKING 部分）
 *         通过条形灯角度和位置偏差控制小车速度和转向
 */
void TrackCar_FollowFly(void)
{
    int16_t vx = 0, vy = 0, vw = 0;

    if (abs(direct_dx) > 6)
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
        if (abs(PX - (-10)) > PY_DEAD)
        {
            vy = (int16_t)(3.4f * fabs(PX - (-10)) + 35.0f);
            vy = (PX - (-10) > 0) ? -vy : vy;
        }
    }

    SetCarSpeed(vx, vy, vw);
}

/**
 * @brief  飞机飞向信标灯
 *         检测到信标灯 → vx=±10, vy=±10 飞向灯（方向由 beacon_PX/PY 决定）
 *         未检测到信标灯 → 原地不动
 */
void TrackFly_Beacon(void)
{
    int16_t vx = 0, vy = 0, vw = 0;

    if (is_fly_beacon_detected)
    {
        // 根据 beacon_PX/PY 的方向决定速度正负
        vx = (beacon_PY > 0) ? 10 : -10; // 上偏则向前，下偏则向后
        vy = (beacon_PX > 0) ? 10 : -10; // 右偏则向右，左偏则向左
        vw = 0;                          // 不旋转
    }
    else
    {
        // 未检测到信标灯，原地不动
        vx = 0;
        vy = 0;
        vw = 0;
    }

    SetFlySpeed(vx, vy, vw);
}

// /**
//  * @brief  小车寻灯追踪（保留原 TrackBeacon 的状态机逻辑，用于小车）
//  */
// void TrackBeacon(void)
// {
//     int16_t vx = 0, vy = 0, vw = 0;
//     switch (current_state)
//     {
//     case BEACON_STATE_LOST:
//         if (is_beacon_detected)
//             current_state = (abs(PX) > PX_DEAD2) ? BEACON_STATE_ALIGNING : BEACON_STATE_TRACKING;
//         break;
//     case BEACON_STATE_ALIGNING:
//         if (abs(PX) <= PX_DEAD2)
//             current_state = BEACON_STATE_TRACKING;
//         break;
//     case BEACON_STATE_TRACKING:
//         if (!is_beacon_detected)
//             current_state = BEACON_STATE_LOST;
//         break;
//     }
//     switch (current_state)
//     {
//     case BEACON_STATE_LOST:
//         vw = SEARCH_VW;
//         vx = 0;
//         vy = 0;
//         break;
//     case BEACON_STATE_ALIGNING:
//         if (abs(PX) > PX_DEAD1)
//             vw = PX > 0 ? 35 : -35;
//         else if (abs(PX) > PX_DEAD2)
//             vw = PX > 0 ? 15 : -15;
//         else
//             vw = 0;
//         vx = 0;
//         vy = 0;
//         break;

//     case BEACON_STATE_TRACKING:
//         TrackCar_FollowFly();
//         // TrackCar_FollowFly 内部已调用 SetCarSpeed，此处直接返回
//         return;

//     default:
//         current_state = BEACON_STATE_LOST;
//         vw = SEARCH_VW;
//         vx = 0;
//         vy = 0;
//     }
//     SetCarSpeed(vx, vy, vw);
// }

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

    // seekfree_assistant_camera_information_config(
    //     SEEKFREE_ASSISTANT_MT9V03X, image_copy[0],
    //     MT9V03X_W, MT9V03X_H);

    // seekfree_assistant_camera_boundary_config(
    //     XY_BOUNDARY, BOUNDARY_NUM,
    //     xy_x1_boundary, xy_x2_boundary, xy_x3_boundary,
    //     xy_y1_boundary, xy_y2_boundary, xy_y3_boundary);
    // seekfree_assistant_camera_information_config(
    //     SEEKFREE_ASSISTANT_MT9V03X, image_copy[0],
    //     MT9V03X_W, MT9V03X_H);

    // seekfree_assistant_camera_boundary_config(
    //     XY_BOUNDARY, BOUNDARY_NUM,
    //     xy_x1_boundary, xy_x2_boundary, xy_x3_boundary,
    //     xy_y1_boundary, xy_y2_boundary, xy_y3_boundary);

    while (1)
    {
        if (mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;

            for (int y = 0; y < MT9V03X_H; y++)
            {
                for (int x = 0; x < MT9V03X_W; x++)
                {
                    uint8_t pix = mt9v03x_image[y][x];
                    image_copy[y][x] = (pix < GRAY_THRESH) ? 0 : pix;
                }
            }

            find_bright_center();
            TrackFly_Beacon();    // 飞机飞向信标灯
            TrackCar_FollowFly(); // 小车直接移动到飞机正下方
        }
        system_delay_ms(1);
    }
}
