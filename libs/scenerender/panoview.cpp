/***********************************************************************
* Software License Agreement (Ruler License)
*
* Copyright 2008-2011  Li YunQiang (liyunqiang@91ruler.com). All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
* NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
* THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*************************************************************************/
#include "panoview.h"
#include <Windows.h>
#include <gl/GL.h>
#include <gl/glut.h>

float  g_degree_rx = 0.0;    //X 旋转量
float  g_degree_ry = 0.0;    //Y 旋转量
float  g_degree_rz = 0.0;    //Z 旋转量
unsigned int g_textures_array[6];    //存储6个纹理

//交叉点的坐标
int g_intersect_cx = 0;
int g_intersect_cy = 0;

inline void drawCube(void); // 绘制
inline void display(void);
inline void reshape(int w, int h);

//处理鼠标事件
inline void onMouseClickEvent(int button, int state, int x, int y);
inline void onMouseMoveEvent(int x, int y);

//键盘输入事件函数
inline void onKeyboardEvent(unsigned char key, int x, int y);

//特殊按键
inline void onSpecialKeyEvent(int key, int x, int y);


Ruler::PanoViewer::PanoViewer()
{
    g_intersect_cx = g_intersect_cy = 0;
    g_degree_rx = g_degree_ry = g_degree_rz = 0.0;
    memset(g_textures_array, 0, 6 * sizeof(int));
}

Ruler::PanoViewer::~PanoViewer() {}

Ruler::PanoViewer& Ruler::PanoViewer::instance()
{
    static Ruler::PanoViewer pano_viewer;
    return pano_viewer;
}

void Ruler::PanoViewer::show(const cv::Mat& siximage, const std::string& name)
{
    int c = 1;
    char* cr[] = { "" };
    glutInit(&c, cr);    //固定格式                    
    glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE);
    glutInitWindowSize(1024 / 2, 1024 / 2);      //显示框的大小
    glutInitWindowPosition(100, 100);            //确定显示框左上角的位置
    glutCreateWindow(name.c_str());

    // 初始化
    {
        glClearColor(0.0, 0.0, 0.0, 0.0);    //清理颜色，为黑色，（也可认为是背景颜色）
        glCullFace(GL_FRONT);                //背面裁剪(背面不可见)
        glEnable(GL_CULL_FACE);              //启用裁剪
        glEnable(GL_TEXTURE_2D);

        memset(g_textures_array, 0x0, sizeof(g_textures_array));
        cv::Mat siximages[6];
        for (int i = 0; i < 6; i++) {
            cv::Mat subimage = siximage.colRange(i*siximage.rows, (i + 1)*siximage.rows).clone();

            glGenTextures(1, &g_textures_array[i]);            //创建纹理
            glBindTexture(GL_TEXTURE_2D, g_textures_array[i]); //使用来自位图数据生成 的典型纹理
            glTexImage2D(GL_TEXTURE_2D, 0, 3, subimage.cols, subimage.rows, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, subimage.data); //生成纹理
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);    //线形滤波
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);    //线形滤波
        }
    }

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);    //绘制图形时的回调
    glutKeyboardFunc(onKeyboardEvent);
    glutMouseFunc(onMouseClickEvent);
    glutMotionFunc(onMouseMoveEvent);
    glutKeyboardFunc(onKeyboardEvent);
    glutSpecialFunc(onSpecialKeyEvent); //特殊按键
    glutMainLoop();
}

void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    //清楚颜色数据和深度数据（清屏）
    glLoadIdentity();     //Reset The View
    drawCube();
    glutSwapBuffers();    //交换缓冲区。显示图形
    Sleep(10);
}

//当窗口大小改变时，会调用这个函数
void reshape(int w, int h)
{
    //这里小说明一下：矩阵模式是不同的，他们各自有一个矩阵。投影相关
    //只能用投影矩阵。(只是目前情况下哦，等我学多了可能就知道为什么了。)

    glViewport(0, 0, w, h);         //设置视口
    glMatrixMode(GL_PROJECTION);    //设置矩阵模式为投影变换矩阵，
    glLoadIdentity();               //变为单位矩阵
    gluPerspective(90, (GLfloat)w / h, 0.1f, 100.0f);    //设置投影矩阵
    glMatrixMode(GL_MODELVIEW);     //设置矩阵模式为视图矩阵(模型)
    glLoadIdentity();               //变为单位矩阵
}

void drawCube(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    //清除屏幕和深度缓存
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();    //重置当前的模型观察矩阵

    glPushMatrix();
    {
        gluLookAt(0, 0, -5, 0, 0, 0, 0, 1, 0);
        glTranslatef(0.0f, 0.0f, -5.0f);             //移入屏幕 5 个单位
        glRotatef(g_degree_rx, 1.0f, 0.0f, 0.0f);    //绕X轴旋转
        glRotatef(g_degree_ry, 0.0f, 1.0f, 0.0f);    //绕Y轴旋转
        glRotatef(g_degree_rz, 0.0f, 0.0f, 1.0f);    //绕Z轴旋转

        glBindTexture(GL_TEXTURE_2D, g_textures_array[0]);    //选择纹理
        glBegin(GL_QUADS); {
            //前面：逆时针
            glTexCoord2f(0.0f, 0.0f); glVertex3f(1.0f, 1.0f, 1.0f);      //纹理和四边形的左下
            glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, 1.0f, 1.0f);     //纹理和四边形的右下
            glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f, -1.0f, 1.0f);    //纹理和四边形的右上
            glTexCoord2f(0.0f, 1.0f); glVertex3f(1.0f, -1.0f, 1.0f);     //纹理和四边形的左上
        }glEnd();

        glBindTexture(GL_TEXTURE_2D, g_textures_array[1]);    //选择纹理
        glBegin(GL_QUADS); {
            //右面：逆时针
            glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, 1.0f, 1.0f);    //纹理和四边形的左下
            glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, 1.0f, -1.0f);   //纹理和四边形的右下
            glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f, -1.0f, -1.0f);  //纹理和四边形的右上
            glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f, -1.0f, 1.0f);   //纹理和四边形的左上
        }glEnd();

        glBindTexture(GL_TEXTURE_2D, g_textures_array[2]);    //选择纹理
        glBegin(GL_QUADS); {
            //后面：逆时针
            glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, 1.0f, -1.0f);   //纹理和四边形的左下
            glTexCoord2f(1.0f, 0.0f); glVertex3f(1.0f, 1.0f, -1.0f);    //纹理和四边形的右下
            glTexCoord2f(1.0f, 1.0f); glVertex3f(1.0f, -1.0f, -1.0f);   //纹理和四边形的右上
            glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f, -1.0f, -1.0f);  //纹理和四边形的左上
        }glEnd();

        glBindTexture(GL_TEXTURE_2D, g_textures_array[3]);    //选择纹理
        glBegin(GL_QUADS); {
            //左面：逆时针
            glTexCoord2f(0.0f, 0.0f); glVertex3f(1.0f, 1.0f, -1.0f);    //纹理和四边形的左下
            glTexCoord2f(1.0f, 0.0f); glVertex3f(1.0f, 1.0f, 1.0f);     //纹理和四边形的右下
            glTexCoord2f(1.0f, 1.0f); glVertex3f(1.0f, -1.0f, 1.0f);    //纹理和四边形的右上
            glTexCoord2f(0.0f, 1.0f); glVertex3f(1.0f, -1.0f, -1.0f);   //纹理和四边形的左上
        }glEnd();

        glBindTexture(GL_TEXTURE_2D, g_textures_array[4]);    //选择纹理
        glBegin(GL_QUADS); {
            //顶面：逆时针
            glTexCoord2f(0.0f, 0.0f); glVertex3f(1.0f, 1.0f, -1.0f);    //纹理和四边形的右下
            glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, 1.0f, -1.0f);   //纹理和四边形的右上
            glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f, 1.0f, 1.0f);    //纹理和四边形的左上
            glTexCoord2f(0.0f, 1.0f); glVertex3f(1.0f, 1.0f, 1.0f);     //纹理和四边形的左下
        }glEnd();

        glBindTexture(GL_TEXTURE_2D, g_textures_array[5]);    //选择纹理
        glBegin(GL_QUADS); {
            //底面：逆时针
            glTexCoord2f(0.0f, 0.0f); glVertex3f(1.0f, -1.0f, 1.0f);    //纹理和四边形的左下
            glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f, 1.0f);   //纹理和四边形的右下
            glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f, -1.0f, -1.0f);  //纹理和四边形的右上
            glTexCoord2f(0.0f, 1.0f); glVertex3f(1.0f, -1.0f, -1.0f);   //纹理和四边形的左上
        }glEnd();

    }
    glPopMatrix();
    glFlush();
}

//处理鼠标点击
void onMouseClickEvent(int button, int state, int x, int y)
{
    if (state == GLUT_DOWN) //第一次鼠标按下时,记录鼠标在窗口中的初始坐标
    {
        g_intersect_cx = x;
        g_intersect_cy = y;
    }
}

//处理鼠标拖动
void onMouseMoveEvent(int x, int y)
{
    float offset = 0.18;
    //计算拖动后的偏移量，然后进行xy叠加减
    g_degree_ry -= ((x - g_intersect_cx) * offset);

    if (g_degree_rx < 90 && y > g_intersect_cy) //往下拉
    {
        g_degree_rx += ((y - g_intersect_cy) * offset);
    }
    else if (g_degree_rx > -90 && y < g_intersect_cy)  //往上拉
    {
        g_degree_rx += ((y - g_intersect_cy) * offset);
    }
    glutPostRedisplay();

    //保存好当前拖放后光标坐标点
    g_intersect_cx = x;
    g_intersect_cy = y;
}

//键盘输入事件函数
void onKeyboardEvent(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 'x':        //当按下键盘上d时，以沿X轴旋转为主
            if (g_degree_rx < 85.0f)
            {
                g_degree_rx += 1.0f;    //设置旋转增量
            }
            break;
        case 'X':
            if (g_degree_rx > -85.0f)
            {
                g_degree_rx -= 1.0f;    //设置旋转增量
            }
            break;
        case 'y':
            g_degree_ry += 1.0f;
            break;
        case 'Y':
            g_degree_ry -= 1.0f;
            break;
        case 'z':
            g_degree_rz += 1.0f;
            break;
        case 'Z':
            g_degree_rz -= 1.0f;
            break;
        default:
            return;
    }
    glutPostRedisplay();    //重绘函数
}

//特殊按键
void onSpecialKeyEvent(int key, int x, int y)
{
    float offset = 1.5;
    switch (key)
    {
        case GLUT_KEY_UP:                 //脑袋向上往前看
            if (g_degree_rx < 90.0f)
            {
                g_degree_rx += offset;    //设置旋转增量
            }
            break;
        case GLUT_KEY_DOWN:               //脑袋向下往前看
            if (g_degree_rx > -90.0f)
            {
                g_degree_rx -= offset;    //设置旋转增量
            }
            break;
        case GLUT_KEY_LEFT:               //脑袋想左往前看
            g_degree_ry -= offset;
            break;
        case GLUT_KEY_RIGHT:              //脑袋向右往前看
            g_degree_ry += offset;
            break;
        default:
            break;
    }
    glutPostRedisplay();
}