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

float  g_degree_rx = 0.0;    //X ��ת��
float  g_degree_ry = 0.0;    //Y ��ת��
float  g_degree_rz = 0.0;    //Z ��ת��
unsigned int g_textures_array[6];    //�洢6������

//����������
int g_intersect_cx = 0;
int g_intersect_cy = 0;

inline void drawCube(void); // ����
inline void display(void);
inline void reshape(int w, int h);

//��������¼�
inline void onMouseClickEvent(int button, int state, int x, int y);
inline void onMouseMoveEvent(int x, int y);

//���������¼�����
inline void onKeyboardEvent(unsigned char key, int x, int y);

//���ⰴ��
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
    glutInit(&c, cr);    //�̶���ʽ                    
    glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE);
    glutInitWindowSize(1024 / 2, 1024 / 2);      //��ʾ��Ĵ�С
    glutInitWindowPosition(100, 100);            //ȷ����ʾ�����Ͻǵ�λ��
    glutCreateWindow(name.c_str());

    // ��ʼ��
    {
        glClearColor(0.0, 0.0, 0.0, 0.0);    //������ɫ��Ϊ��ɫ����Ҳ����Ϊ�Ǳ�����ɫ��
        glCullFace(GL_FRONT);                //����ü�(���治�ɼ�)
        glEnable(GL_CULL_FACE);              //���òü�
        glEnable(GL_TEXTURE_2D);

        memset(g_textures_array, 0x0, sizeof(g_textures_array));
        cv::Mat siximages[6];
        for (int i = 0; i < 6; i++) {
            cv::Mat subimage = siximage.colRange(i*siximage.rows, (i + 1)*siximage.rows).clone();

            glGenTextures(1, &g_textures_array[i]);            //��������
            glBindTexture(GL_TEXTURE_2D, g_textures_array[i]); //ʹ������λͼ�������� �ĵ�������
            glTexImage2D(GL_TEXTURE_2D, 0, 3, subimage.cols, subimage.rows, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, subimage.data); //��������
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);    //�����˲�
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);    //�����˲�
        }
    }

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);    //����ͼ��ʱ�Ļص�
    glutKeyboardFunc(onKeyboardEvent);
    glutMouseFunc(onMouseClickEvent);
    glutMotionFunc(onMouseMoveEvent);
    glutKeyboardFunc(onKeyboardEvent);
    glutSpecialFunc(onSpecialKeyEvent); //���ⰴ��
    glutMainLoop();
}

void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    //�����ɫ���ݺ�������ݣ�������
    glLoadIdentity();     //Reset The View
    drawCube();
    glutSwapBuffers();    //��������������ʾͼ��
    Sleep(10);
}

//�����ڴ�С�ı�ʱ��������������
void reshape(int w, int h)
{
    //����С˵��һ�£�����ģʽ�ǲ�ͬ�ģ����Ǹ�����һ������ͶӰ���
    //ֻ����ͶӰ����(ֻ��Ŀǰ�����Ŷ������ѧ���˿��ܾ�֪��Ϊʲô�ˡ�)

    glViewport(0, 0, w, h);         //�����ӿ�
    glMatrixMode(GL_PROJECTION);    //���þ���ģʽΪͶӰ�任����
    glLoadIdentity();               //��Ϊ��λ����
    gluPerspective(90, (GLfloat)w / h, 0.1f, 100.0f);    //����ͶӰ����
    glMatrixMode(GL_MODELVIEW);     //���þ���ģʽΪ��ͼ����(ģ��)
    glLoadIdentity();               //��Ϊ��λ����
}

void drawCube(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    //�����Ļ����Ȼ���
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();    //���õ�ǰ��ģ�͹۲����

    glPushMatrix();
    {
        gluLookAt(0, 0, -5, 0, 0, 0, 0, 1, 0);
        glTranslatef(0.0f, 0.0f, -5.0f);             //������Ļ 5 ����λ
        glRotatef(g_degree_rx, 1.0f, 0.0f, 0.0f);    //��X����ת
        glRotatef(g_degree_ry, 0.0f, 1.0f, 0.0f);    //��Y����ת
        glRotatef(g_degree_rz, 0.0f, 0.0f, 1.0f);    //��Z����ת

        glBindTexture(GL_TEXTURE_2D, g_textures_array[0]);    //ѡ������
        glBegin(GL_QUADS); {
            //ǰ�棺��ʱ��
            glTexCoord2f(0.0f, 0.0f); glVertex3f(1.0f, 1.0f, 1.0f);      //������ı��ε�����
            glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, 1.0f, 1.0f);     //������ı��ε�����
            glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f, -1.0f, 1.0f);    //������ı��ε�����
            glTexCoord2f(0.0f, 1.0f); glVertex3f(1.0f, -1.0f, 1.0f);     //������ı��ε�����
        }glEnd();

        glBindTexture(GL_TEXTURE_2D, g_textures_array[1]);    //ѡ������
        glBegin(GL_QUADS); {
            //���棺��ʱ��
            glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, 1.0f, 1.0f);    //������ı��ε�����
            glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, 1.0f, -1.0f);   //������ı��ε�����
            glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f, -1.0f, -1.0f);  //������ı��ε�����
            glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f, -1.0f, 1.0f);   //������ı��ε�����
        }glEnd();

        glBindTexture(GL_TEXTURE_2D, g_textures_array[2]);    //ѡ������
        glBegin(GL_QUADS); {
            //���棺��ʱ��
            glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, 1.0f, -1.0f);   //������ı��ε�����
            glTexCoord2f(1.0f, 0.0f); glVertex3f(1.0f, 1.0f, -1.0f);    //������ı��ε�����
            glTexCoord2f(1.0f, 1.0f); glVertex3f(1.0f, -1.0f, -1.0f);   //������ı��ε�����
            glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f, -1.0f, -1.0f);  //������ı��ε�����
        }glEnd();

        glBindTexture(GL_TEXTURE_2D, g_textures_array[3]);    //ѡ������
        glBegin(GL_QUADS); {
            //���棺��ʱ��
            glTexCoord2f(0.0f, 0.0f); glVertex3f(1.0f, 1.0f, -1.0f);    //������ı��ε�����
            glTexCoord2f(1.0f, 0.0f); glVertex3f(1.0f, 1.0f, 1.0f);     //������ı��ε�����
            glTexCoord2f(1.0f, 1.0f); glVertex3f(1.0f, -1.0f, 1.0f);    //������ı��ε�����
            glTexCoord2f(0.0f, 1.0f); glVertex3f(1.0f, -1.0f, -1.0f);   //������ı��ε�����
        }glEnd();

        glBindTexture(GL_TEXTURE_2D, g_textures_array[4]);    //ѡ������
        glBegin(GL_QUADS); {
            //���棺��ʱ��
            glTexCoord2f(0.0f, 0.0f); glVertex3f(1.0f, 1.0f, -1.0f);    //������ı��ε�����
            glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, 1.0f, -1.0f);   //������ı��ε�����
            glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f, 1.0f, 1.0f);    //������ı��ε�����
            glTexCoord2f(0.0f, 1.0f); glVertex3f(1.0f, 1.0f, 1.0f);     //������ı��ε�����
        }glEnd();

        glBindTexture(GL_TEXTURE_2D, g_textures_array[5]);    //ѡ������
        glBegin(GL_QUADS); {
            //���棺��ʱ��
            glTexCoord2f(0.0f, 0.0f); glVertex3f(1.0f, -1.0f, 1.0f);    //������ı��ε�����
            glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f, 1.0f);   //������ı��ε�����
            glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f, -1.0f, -1.0f);  //������ı��ε�����
            glTexCoord2f(0.0f, 1.0f); glVertex3f(1.0f, -1.0f, -1.0f);   //������ı��ε�����
        }glEnd();

    }
    glPopMatrix();
    glFlush();
}

//���������
void onMouseClickEvent(int button, int state, int x, int y)
{
    if (state == GLUT_DOWN) //��һ����갴��ʱ,��¼����ڴ����еĳ�ʼ����
    {
        g_intersect_cx = x;
        g_intersect_cy = y;
    }
}

//��������϶�
void onMouseMoveEvent(int x, int y)
{
    float offset = 0.18;
    //�����϶����ƫ������Ȼ�����xy���Ӽ�
    g_degree_ry -= ((x - g_intersect_cx) * offset);

    if (g_degree_rx < 90 && y > g_intersect_cy) //������
    {
        g_degree_rx += ((y - g_intersect_cy) * offset);
    }
    else if (g_degree_rx > -90 && y < g_intersect_cy)  //������
    {
        g_degree_rx += ((y - g_intersect_cy) * offset);
    }
    glutPostRedisplay();

    //����õ�ǰ�Ϸź��������
    g_intersect_cx = x;
    g_intersect_cy = y;
}

//���������¼�����
void onKeyboardEvent(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 'x':        //�����¼�����dʱ������X����תΪ��
            if (g_degree_rx < 85.0f)
            {
                g_degree_rx += 1.0f;    //������ת����
            }
            break;
        case 'X':
            if (g_degree_rx > -85.0f)
            {
                g_degree_rx -= 1.0f;    //������ת����
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
    glutPostRedisplay();    //�ػ溯��
}

//���ⰴ��
void onSpecialKeyEvent(int key, int x, int y)
{
    float offset = 1.5;
    switch (key)
    {
        case GLUT_KEY_UP:                 //�Դ�������ǰ��
            if (g_degree_rx < 90.0f)
            {
                g_degree_rx += offset;    //������ת����
            }
            break;
        case GLUT_KEY_DOWN:               //�Դ�������ǰ��
            if (g_degree_rx > -90.0f)
            {
                g_degree_rx -= offset;    //������ת����
            }
            break;
        case GLUT_KEY_LEFT:               //�Դ�������ǰ��
            g_degree_ry -= offset;
            break;
        case GLUT_KEY_RIGHT:              //�Դ�������ǰ��
            g_degree_ry += offset;
            break;
        default:
            break;
    }
    glutPostRedisplay();
}