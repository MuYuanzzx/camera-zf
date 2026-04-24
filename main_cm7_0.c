#include "zf_common_headfile.h"
#include <stdint.h>
#include <string.h>

#define MT9V03X_W    188
#define MT9V03X_H    120
#define IMAGE_SIZE   (MT9V03X_W * MT9V03X_H)

#define BOUNDARY_NUM (MT9V03X_H * 2)
uint8_t xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
uint8_t xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

uint8_t image_copy[MT9V03X_H][MT9V03X_W];

// ===================== 参数区（只保留必要的） =====================
#define GRAY_THRESH      80      // 预处理：低于此值直接置黑
#define BIN_THRESH       35      // 二值化阈值
#define AREA_MIN         5       // 最小面积过滤噪点
// ====================================================================

// 8邻域
const int8_t dx[8] = {-1,0,1,-1,1,-1,0,1};
const int8_t dy[8] = {-1,-1,-1,0,0,1,1,1};

// 连通域信息
typedef struct {
    int16_t cx, cy;
    int16_t minx, maxx, miny, maxy;
    uint32_t area;
    float max_ratio;  // 最长边/最短边（相对长宽比，用于判断长条）
} Blob;

Blob blobs[8];
uint8_t blob_cnt = 0;

// 找所有连通域 + 计算相对长宽比
void find_all_blobs(void)
{
    uint8_t vis[MT9V03X_H][MT9V03X_W] = {0};
    blob_cnt = 0;

    for(int y=0; y<MT9V03X_H; y++){
        for(int x=0; x<MT9V03X_W; x++){
            if(image_copy[y][x] > BIN_THRESH && !vis[y][x] && blob_cnt < 8)
            {
                int stack_x[1024], stack_y[1024];
                int top = 0;
                stack_x[top] = x;
                stack_y[top] = y;
                top++;
                vis[y][x] = 1;

                int64_t sum_x=0, sum_y=0;
                uint32_t area = 0;
                int16_t minx=MT9V03X_W, maxx=0, miny=MT9V03X_H, maxy=0;

                while(top>0){
                    top--;
                    int cx = stack_x[top];
                    int cy = stack_y[top];

                    sum_x += cx;
                    sum_y += cy;
                    area++;
                    if(cx<minx) minx=cx;
                    if(cx>maxx) maxx=cx;
                    if(cy<miny) miny=cy;
                    if(cy>maxy) maxy=cy;

                    for(int k=0;k<8;k++){
                        int nx = cx+dx[k];
                        int ny = cy+dy[k];
                        if(nx>=0&&nx<MT9V03X_W&&ny>=0&&ny<MT9V03X_H&&!vis[ny][nx]&&image_copy[ny][nx]>BIN_THRESH){
                            vis[ny][nx] = 1;
                            stack_x[top] = nx;
                            stack_y[top] = ny;
                            top++;
                        }
                    }
                }

                if(area < AREA_MIN) continue;

                // 计算宽度、高度
                int16_t w = maxx - minx + 1;
                int16_t h = maxy - miny + 1;
                
                // ===================== 修复报错：替换max/min为三目运算符 =====================
                float max_val = (w > h) ? w : h;  // 替代 max(w,h)
                float min_val = (w < h) ? w : h;  // 替代 min(w,h)
                float max_ratio = max_val / min_val;
                // ============================================================================

                // 保存连通域信息
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

// 主识别函数：长宽比最大的标黄（条形灯），其他标红（圆灯）
void find_bright_center(void)
{
    memset(xy_x2_boundary, 0, sizeof(xy_x2_boundary));
    memset(xy_y2_boundary, 0, sizeof(xy_y2_boundary));
    memset(xy_x3_boundary, 0, sizeof(xy_x3_boundary));
    memset(xy_y3_boundary, 0, sizeof(xy_y3_boundary));

    int cnt_red = 0;   // 圆形灯：红色 → xy2
    int cnt_yel = 0;   // 条形灯：黄色 → xy3

    // 1. 获取所有连通域
    find_all_blobs();

    if(blob_cnt == 0)
    {
        // 无目标：画中心红点
        for(int8_t dy=-1; dy<=1; dy++){
            for(int8_t dx=-1; dx<=1; dx++){
                int16_t x = MT9V03X_W/2+dx, y = MT9V03X_H/2+dy;
                if(x>=0&&x<MT9V03X_W&&y>=0&&y<MT9V03X_H&&cnt_red<BOUNDARY_NUM){
                    xy_x2_boundary[cnt_red] = x;
                    xy_y2_boundary[cnt_red] = y;
                    cnt_red++;
                }
            }
        }
        return;
    }

    // 2. 找出【长宽比最大】的连通域，作为条形灯
    int bar_idx = 0;
    float max_ratio = blobs[0].max_ratio;
    for(int i=1; i<blob_cnt; i++)
    {
        if(blobs[i].max_ratio > max_ratio)
        {
            max_ratio = blobs[i].max_ratio;
            bar_idx = i;
        }
    }

    // 3. 绘制条形灯（黄色，xy3）
    Blob bar_blob = blobs[bar_idx];
    {
        int16_t cx = bar_blob.cx;
        int16_t cy = bar_blob.cy;
        int16_t minx = bar_blob.minx;
        int16_t maxx = bar_blob.maxx;
        int16_t miny = bar_blob.miny;
        int16_t maxy = bar_blob.maxy;

        // 中心点
        for(int8_t dy=-1; dy<=1; dy++){
            for(int8_t dx=-1; dx<=1; dx++){
                int16_t x = cx+dx, y = cy+dy;
                if(x>=0&&x<MT9V03X_W&&y>=0&&y<MT9V03X_H&&cnt_yel<BOUNDARY_NUM){
                    xy_x3_boundary[cnt_yel] = x;
                    xy_y3_boundary[cnt_yel] = y;
                    cnt_yel++;
                }
            }
        }
        // 再画两个端点，让黄色标注更明显
        if(bar_blob.maxx - bar_blob.minx > bar_blob.maxy - bar_blob.miny)
        {
            // 水平长条：画左右端点
            for(int8_t dy=-1; dy<=1; dy++){
                for(int8_t dx=-1; dx<=1; dx++){
                    int16_t x = minx+dx, y = cy+dy;
                    if(x>=0&&x<MT9V03X_W&&y>=0&&y<MT9V03X_H&&cnt_yel<BOUNDARY_NUM){
                        xy_x3_boundary[cnt_yel] = x;
                        xy_y3_boundary[cnt_yel] = y;
                        cnt_yel++;
                    }
                }
            }
            for(int8_t dy=-1; dy<=1; dy++){
                for(int8_t dx=-1; dx<=1; dx++){
                    int16_t x = maxx+dx, y = cy+dy;
                    if(x>=0&&x<MT9V03X_W&&y>=0&&y<MT9V03X_H&&cnt_yel<BOUNDARY_NUM){
                        xy_x3_boundary[cnt_yel] = x;
                        xy_y3_boundary[cnt_yel] = y;
                        cnt_yel++;
                    }
                }
            }
        }
        else
        {
            // 竖直长条：画上下端点
            for(int8_t dy=-1; dy<=1; dy++){
                for(int8_t dx=-1; dx<=1; dx++){
                    int16_t x = cx+dx, y = miny+dy;
                    if(x>=0&&x<MT9V03X_W&&y>=0&&y<MT9V03X_H&&cnt_yel<BOUNDARY_NUM){
                        xy_x3_boundary[cnt_yel] = x;
                        xy_y3_boundary[cnt_yel] = y;
                        cnt_yel++;
                    }
                }
            }
            for(int8_t dy=-1; dy<=1; dy++){
                for(int8_t dx=-1; dx<=1; dx++){
                    int16_t x = cx+dx, y = maxy+dy;
                    if(x>=0&&x<MT9V03X_W&&y>=0&&y<MT9V03X_H&&cnt_yel<BOUNDARY_NUM){
                        xy_x3_boundary[cnt_yel] = x;
                        xy_y3_boundary[cnt_yel] = y;
                        cnt_yel++;
                    }
                }
            }
        }
    }

    // 4. 剩下的所有连通域，一律当作圆灯标红
    for(int i=0; i<blob_cnt; i++)
    {
        if(i == bar_idx) continue;  // 跳过已经标黄的条形灯

        Blob circle_blob = blobs[i];
        int16_t cx = circle_blob.cx;
        int16_t cy = circle_blob.cy;

        // 画中心点（3x3方块）
        for(int8_t dy=-1; dy<=1; dy++){
            for(int8_t dx=-1; dx<=1; dx++){
                int16_t x = cx+dx, y = cy+dy;
                if(x>=0&&x<MT9V03X_W&&y>=0&&y<MT9V03X_H&&cnt_red<BOUNDARY_NUM){
                    xy_x2_boundary[cnt_red] = x;
                    xy_y2_boundary[cnt_red] = y;
                    cnt_red++;
                }
            }
        }
    }
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);

    while(mt9v03x_init()){
        system_delay_ms(500);
    }

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X, image_copy[0],
        MT9V03X_W, MT9V03X_H
    );

    seekfree_assistant_camera_boundary_config(
        XY_BOUNDARY, BOUNDARY_NUM,
        xy_x1_boundary, xy_x2_boundary, xy_x3_boundary,
        xy_y1_boundary, xy_y2_boundary, xy_y3_boundary
    );

    while(1)
    {
        if(mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;

            // 灰度预处理：低于GRAY_THRESH的像素直接置0
            for(int y=0; y<MT9V03X_H; y++){
                for(int x=0; x<MT9V03X_W; x++){
                    uint8_t pix = mt9v03x_image[y][x];
                    image_copy[y][x] = (pix <= GRAY_THRESH) ? 0 : pix;
                }
            }

            find_bright_center();
            seekfree_assistant_camera_send();
        }
        system_delay_ms(1);
    }
}