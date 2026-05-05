#include "zf_common_headfile.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

#define IMAGE_SIZE (MT9V03X_W * MT9V03X_H)

#define BOUNDARY_NUM (MT9V03X_H * 2)

int16 bright_center_x = MT9V03X_W / 2;
int16 bright_center_y = MT9V03X_H / 2;
int16 CenterX = MT9V03X_W / 2;
int16 CenterY = MT9V03X_H / 2;
float dir_led_angle = 0;

uint8_t xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
uint8_t xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

int16 PX = 0;
int16 PY = 0;

uint8_t is_beacon_detected = 0;
uint8_t is_fly_beacon_detected = 0;

int16_t beacon_cx = -1;
int16_t beacon_cy = -1;

int16_t beacon_PX = 0;
int16_t beacon_PY = 0;

uint8_t image_copy[MT9V03X_H][MT9V03X_W];

#define GRAY_THRESH 110
#define BIN_THRESH 35
#define AREA_MIN 5
#define PX_DEAD1 30
#define PX_DEAD2 4
#define PY_DEAD 5
#define SEARCH_VW 70
#define LED1 P19_0

const int8_t dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
const int8_t dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

typedef enum { BEACON_STATE_LOST, BEACON_STATE_ALIGNING, BEACON_STATE_TRACKING } BeaconState;
static BeaconState current_state = BEACON_STATE_LOST;
int16_t direct_dx = 0;

// ===== Blob结构 =====
typedef struct
{
    int16_t cx, cy;
    int16_t minx, maxx, miny, maxy;
    uint32_t area;
    float max_ratio;
    uint32_t sum_pixel;
} Blob;

Blob blobs[8];
uint8_t blob_cnt = 0;

// ===== L型双垂直条灯结构：两个互相独立且垂直的长条灯，直角拐点方向即小车正前方 =====
typedef struct
{
    int16_t corner_x;         // L型拐点 x（小车正前方）
    int16_t corner_y;         // L型拐点 y
    uint8_t idx_strip1;       // 条灯1索引
    uint8_t idx_strip2;       // 条灯2索引
    float angle;              // 小车朝向角度（相对于图像Y轴，顺时针为正）
} LShapeLED;

LShapeLED lshape_led;

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

#define FLY_CONTROL_UART UART_0
static uint8_t FlyTxPacket[9];

static void FlyPacket_Checksum(uint8_t *packet, const int16_t speed[3])
{
    packet[0] = 0xFF;
    packet[1] = 0xFC;
    packet[2] = (uint8_t)(speed[0] & 0xFF);
    packet[3] = (uint8_t)((speed[0] >> 8) & 0xFF);
    packet[4] = (uint8_t)(speed[1] & 0xFF);
    packet[5] = (uint8_t)((speed[1] >> 8) & 0xFF);
    packet[6] = (uint8_t)(speed[2] & 0xFF);
    packet[7] = (uint8_t)((speed[2] >> 8) & 0xFF);
    packet[8] = (speed[0] & 0x01) + (speed[1] & 0x01) + (speed[2] & 0x01);
}

void SetFlySpeed(float vx, float vy, float vw)
{
    int16_t speed_buf[3];
    speed_buf[0] = (int16_t)vx;
    speed_buf[1] = (int16_t)vy;
    speed_buf[2] = (int16_t)vw;
    FlyPacket_Checksum(FlyTxPacket, speed_buf);
    uart_write_buffer(FLY_CONTROL_UART, FlyTxPacket, 9);
}

void UpdateBeaconPos(int16_t x, int16_t y)
{
    if (x < 0) x = 0;
    if (x >= MT9V03X_W) x = MT9V03X_W - 1;
    if (y < 0) y = 0;
    if (y >= MT9V03X_H) y = MT9V03X_H - 1;
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
                    if (cx < minx) minx = cx;
                    if (cx > maxx) maxx = cx;
                    if (cy < miny) miny = cy;
                    if (cy > maxy) maxy = cy;

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

                if (area < AREA_MIN) continue;

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

/**
 * 判断一个blob是否是长条灯（高长宽比）
 * 长条灯的max_ratio > 2，圆形信标灯的max_ratio ≈ 1
 */
uint8_t is_strip_led(const Blob *blob)
{
    return (blob->max_ratio > 2.0f);
}

/**
 * 判断两个blob是否互相垂直
 * 一个主要水平（w > h），一个主要垂直（h > w）
 */
uint8_t are_perpendicular(const Blob *strip1, const Blob *strip2)
{
    int16_t w1 = strip1->maxx - strip1->minx;
    int16_t h1 = strip1->maxy - strip1->miny;
    int16_t w2 = strip2->maxx - strip2->minx;
    int16_t h2 = strip2->maxy - strip2->miny;

    // 如果一个主要水平，一个主要垂直，则互相垂直
    if ((w1 > h1 && w2 < h2) || (w1 < h1 && w2 > h2))
        return 1;
    return 0;
}

/**
 * 计算L型拐点的方向角度（小车朝向）
 * L型两条条灯形成的角的角平分线反方向 = 车头正前方
 */
float calculate_lshape_angle(int16_t corner_x, int16_t corner_y,
                              int16_t strip1_cx, int16_t strip1_cy,
                              int16_t strip2_cx, int16_t strip2_cy)
{
    // 从拐点出发，两条条灯的方向向量
    float vec1_x = (float)(strip1_cx - corner_x);
    float vec1_y = (float)(strip1_cy - corner_y);
    float vec2_x = (float)(strip2_cx - corner_x);
    float vec2_y = (float)(strip2_cy - corner_y);

    // 小车朝向：L型"开口方向"的反方向（即两条条灯方向的和向量的反方向）
    float front_x = -(vec1_x + vec2_x);
    float front_y = -(vec1_y + vec2_y);

    // 计算角度（相对于图像Y轴，顺时针为正）
    float angle = atan2f(front_x, front_y) * 180.0f / PI;

    return angle;
}

uint8_t no_car_led = 0;

void find_bright_center(void)
{
    memset(xy_x2_boundary, 0, sizeof(xy_x2_boundary));
    memset(xy_y2_boundary, 0, sizeof(xy_y2_boundary));
    memset(xy_x3_boundary, 0, sizeof(xy_x3_boundary));
    memset(xy_y3_boundary, 0, sizeof(xy_y3_boundary));

    int cnt_red = 0;
    int cnt_yel = 0;

    // L型条灯初始化
    lshape_led.corner_x = lshape_led.corner_y = -1;
    lshape_led.idx_strip1 = 255;
    lshape_led.idx_strip2 = 255;
    lshape_led.angle = 0.0f;

    find_all_blobs();

    is_beacon_detected = 0;
    is_fly_beacon_detected = 0;

    if (blob_cnt == 0)
    {
        no_car_led = 1;
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
    no_car_led = 0;

    // ===== L型双条灯识别 =====
    // 第一步：找出所有长条灯（排除圆形信标）
    uint8_t strip_indices[8];
    uint8_t strip_cnt = 0;

    for (int i = 0; i < blob_cnt && strip_cnt < 4; i++)
    {
        if (is_strip_led(&blobs[i]))
        {
            strip_indices[strip_cnt] = i;
            strip_cnt++;
        }
    }

    // 第二步：寻找互相垂直的两个条灯组成L型
    uint8_t found_lshape = 0;

    if (strip_cnt >= 2)
    {
        for (int i = 0; i < strip_cnt && !found_lshape; i++)
        {
            for (int j = i + 1; j < strip_cnt && !found_lshape; j++)
            {
                if (are_perpendicular(&blobs[strip_indices[i]], &blobs[strip_indices[j]]))
                {
                    uint8_t idx1 = strip_indices[i];
                    uint8_t idx2 = strip_indices[j];

                    Blob *s1 = &blobs[idx1];
                    Blob *s2 = &blobs[idx2];

                    // 计算L型拐点：找两个条灯中距离最近的像素点对，取其中点作为拐点
                    float min_dist = 1e9f;
                    int16_t best_x1 = s1->cx, best_x2 = s2->cx;
                    int16_t best_y1 = s1->cy, best_y2 = s2->cy;

                    for (int y = s1->miny; y <= s1->maxy; y += 2)
                    {
                        for (int x = s1->minx; x <= s1->maxx; x++)
                        {
                            if (image_copy[y][x] > BIN_THRESH)
                            {
                                for (int y2 = s2->miny; y2 <= s2->maxy; y2 += 2)
                                {
                                    for (int x2 = s2->minx; x2 <= s2->maxx; x2++)
                                    {
                                        if (image_copy[y2][x2] > BIN_THRESH)
                                        {
                                            float dist = sqrtf((float)(x - x2) * (x - x2) + (y - y2) * (y - y2));
                                            if (dist < min_dist)
                                            {
                                                min_dist = dist;
                                                best_x1 = x;  best_y1 = y;
                                                best_x2 = x2; best_y2 = y2;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // 拐点取最近两点的中点
                    int16_t corner_x = (best_x1 + best_x2) / 2;
                    int16_t corner_y = (best_y1 + best_y2) / 2;

                    lshape_led.corner_x = corner_x;
                    lshape_led.corner_y = corner_y;
                    lshape_led.idx_strip1 = idx1;
                    lshape_led.idx_strip2 = idx2;

                    // 计算小车朝向角度（L型开口的角平分线反方向）
                    lshape_led.angle = calculate_lshape_angle(
                        corner_x, corner_y,
                        s1->cx, s1->cy,
                        s2->cx, s2->cy
                    );

                    found_lshape = 1;
                }
            }
        }
    }

    if (!found_lshape)
    {
        no_car_led = 1;
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

    // 使用L型条灯的拐点作为小车位置（摄像头正下方 = 直角拐点）
    UpdateBeaconPos(lshape_led.corner_x, lshape_led.corner_y);

    // 更新全局角度变量（兼容原有代码）
    dir_led_angle = lshape_led.angle;

    // ===== 边界点标记 =====
    {
        for (int8_t dy = -1; dy <= 1; dy++)
        {
            for (int8_t dx = -1; dx <= 1; dx++)
            {
                int16_t x = lshape_led.corner_x + dx, y = lshape_led.corner_y + dy;
                if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_yel < BOUNDARY_NUM)
                {
                    xy_x3_boundary[cnt_yel] = x;
                    xy_y3_boundary[cnt_yel] = y;
                    cnt_yel++;
                }
            }
        }

        // 条灯1中心附近
        if (lshape_led.idx_strip1 < blob_cnt)
        {
            Blob *s1 = &blobs[lshape_led.idx_strip1];
            for (int8_t dy = -1; dy <= 1; dy++)
            {
                for (int8_t dx = -1; dx <= 1; dx++)
                {
                    int16_t x = s1->cx + dx, y = s1->cy + dy;
                    if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_yel < BOUNDARY_NUM)
                    {
                        xy_x3_boundary[cnt_yel] = x;
                        xy_y3_boundary[cnt_yel] = y;
                        cnt_yel++;
                    }
                }
            }
        }

        // 条灯2中心附近
        if (lshape_led.idx_strip2 < blob_cnt)
        {
            Blob *s2 = &blobs[lshape_led.idx_strip2];
            for (int8_t dy = -1; dy <= 1; dy++)
            {
                for (int8_t dx = -1; dx <= 1; dx++)
                {
                    int16_t x = s2->cx + dx, y = s2->cy + dy;
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
    is_fly_beacon_detected = 0;
    beacon_cx = -1;
    beacon_cy = -1;
    {
        int16_t fly_choice_idx = -1;
        uint32_t max_sum_pixel = 0;

        // 第一轮：找最大亮度（排除两个条灯）
        for (int i = 0; i < blob_cnt; i++)
        {
            if (i == lshape_led.idx_strip1 || i == lshape_led.idx_strip2)
                continue;
            if (blobs[i].sum_pixel > max_sum_pixel)
                max_sum_pixel = blobs[i].sum_pixel;
        }

        if (max_sum_pixel > 0)
        {
            uint32_t brightness_thresh = (max_sum_pixel * 90) / 100;
            uint32_t min_dist_sq = (uint32_t)-1;

            // 第二轮：在亮度 >= threshold 的 blob 中选最近图像中心的
            for (int i = 0; i < blob_cnt; i++)
            {
                if (i == lshape_led.idx_strip1 || i == lshape_led.idx_strip2)
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
            beacon_PX = beacon_cx - CenterX;
            beacon_PY = -(beacon_cy - CenterY);
        }
    }

    for (int i = 0; i < blob_cnt; i++)
    {
        if (i == lshape_led.idx_strip1 || i == lshape_led.idx_strip2)
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

int TrackCar_Beacon(void)
{
    int16_t vx = 0, vy = 0, vw = 0;
    if (no_car_led == 1)
    {
        SetCarSpeed(vx, vy, vw);
        return 0;
    }

    if (!is_fly_beacon_detected)
    {
        SetCarSpeed(vx, vy, vw);
        return 0;
    }

    vw = 0;

    float angle_rad = -dir_led_angle * PI / 180.0f;
    float cos_a = cosf(angle_rad);
    float sin_a = sinf(angle_rad);

    float car_dx = beacon_PX * cos_a + beacon_PY * sin_a;
    float car_dy = -beacon_PX * sin_a + beacon_PY * cos_a;

    if (fabs(car_dy) > PY_DEAD)
    {
        vx = (int16_t)(3.4f * fabs(car_dy) + 35.0f);
        vx = (car_dy > 0) ? vx : -vx;
    }

    if (fabs(car_dx) > PY_DEAD)
    {
        vy = (int16_t)(3.4f * fabs(car_dx) + 35.0f);
        vy = (car_dx > 0) ? vy : -vy;
    }

    SetCarSpeed(vx, vy, vw);
}

void TrackFly_Car(void)
{
    float vx = 0.0f, vy = 0.0f, vw = 0.0f;

    if (!no_car_led)
    {
        // PX/PY是拐点的偏移量（拐点 = 小车位置）
        if (abs(PY) > PY_DEAD)
        {
            vx = (PY > 0) ? 10.0f : -10.0f;
        }
        else
        {
            vx = 0.0f;
        }

        if (abs(PX) > PY_DEAD)
        {
            vy = (PX > 0) ? 10.0f : -10.0f;
        }
        else
        {
            vy = 0.0f;
        }
        vw = 0.0f;
    }
    else
    {
        vx = 0.0f;
        vy = 0.0f;
        vw = 0.0f;
    }

    SetFlySpeed(vx, vy, vw);
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

            for (int y = 0; y < MT9V03X_H; y++)
            {
                for (int x = 0; x < MT9V03X_W; x++)
                {
                    uint8_t pix = mt9v03x_image[y][x];
                    image_copy[y][x] = (pix < GRAY_THRESH) ? 0 : pix;
                }
            }

            find_bright_center();
            TrackCar_Beacon();
            TrackFly_Car();
        }
        system_delay_ms(1);
    }
}
