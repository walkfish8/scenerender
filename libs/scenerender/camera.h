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
#ifndef _SCENERENDER_CAMERA_H_
#define _SCENERENDER_CAMERA_H_

#include "defines.h"
#include "math.h"

namespace Ruler
{

template<class FloatT_>
struct CameraT_
{
public:
    FloatT_		t[3];				// T in  P = K[R T], T = - RC
    FloatT_		m[3][3];			// R in  P = K[R T]. 

    //////////////////////////////////////////////////////////
    CameraT_()
    {
        t[0] = t[1] = t[2] = 0.0;
        m[0][0] = 1.0; m[0][1] = 0.0; m[0][2] = 0.0;
        m[1][0] = 0.0; m[1][1] = 1.0; m[1][2] = 0.0;
        m[2][0] = 0.0; m[2][1] = 0.0; m[2][2] = 1.0;
    }

    template <class FloatT> CameraT_(FloatT tx, FloatT ty, FloatT tz, FloatT qx, FloatT qy, FloatT qz, FloatT qw)
    {
        t[0] = (FloatT_)tx;
        t[1] = (FloatT_)ty;
        t[2] = (FloatT_)tz;

        FloatT q[4] = { qw, qx, qy, qz };
        SetQuaternionRotation(q);
    }

    //////////////////////////////////////////////
    template<class CameraX> void SetCameraT(const CameraX& cam)
    {
        t[0] = (FloatT_)cam.t[0];    t[1] = (FloatT_)cam.t[1];    t[2] = (FloatT_)cam.t[2];
        for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) m[i][j] = (FloatT_)cam.m[i][j];
    }

    //////////////////////////////////////////
    template <class FloatT> void SetRodriguesRotation(const FloatT r[3])
    {
        double a = sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
        double ct = a == 0.0 ? 0.5 : (1.0 - cos(a)) / a / a;
        double st = a == 0.0 ? 1 : sin(a) / a;
        m[0][0] = FloatT_(1.0 - (r[1] * r[1] + r[2] * r[2])*ct);
        m[0][1] = FloatT_(r[0] * r[1] * ct - r[2] * st);
        m[0][2] = FloatT_(r[2] * r[0] * ct + r[1] * st);
        m[1][0] = FloatT_(r[0] * r[1] * ct + r[2] * st);
        m[1][1] = FloatT_(1.0 - (r[2] * r[2] + r[0] * r[0])*ct);
        m[1][2] = FloatT_(r[1] * r[2] * ct - r[0] * st);
        m[2][0] = FloatT_(r[2] * r[0] * ct - r[1] * st);
        m[2][1] = FloatT_(r[1] * r[2] * ct + r[0] * st);
        m[2][2] = FloatT_(1.0 - (r[0] * r[0] + r[1] * r[1])*ct);
    }

    template <class FloatT> void SetRodriguesRotation(FloatT rx, FloatT ry, FloatT rz)
    {
        FloatT r[3] = { rx, ry, rz };
        SetRodriguesRotation(r);
    }

    template <class FloatT> void GetRodriguesRotation(FloatT r[3]) const
    {
        double a = (m[0][0] + m[1][1] + m[2][2] - 1.0) / 2.0;
        const double epsilon = 0.0001;
        if (fabs(m[0][1] - m[1][0]) < epsilon && fabs(m[1][2] - m[2][1]) < epsilon && fabs(m[0][2] - m[2][0]) < epsilon)
        {
            if (fabs(m[0][1] + m[1][0]) < 0.1 && fabs(m[1][2] + m[2][1]) < 0.1 && fabs(m[0][2] + m[2][0]) < 0.1 && a > 0.9)
            {
                r[0] = 0;
                r[1] = 0;
                r[2] = 0;
            }
            else
            {
                const FloatT ha = FloatT(sqrt(0.5) * 3.14159265358979323846);
                double xx = (m[0][0] + 1.0) / 2.0;
                double yy = (m[1][1] + 1.0) / 2.0;
                double zz = (m[2][2] + 1.0) / 2.0;
                double xy = (m[0][1] + m[1][0]) / 4.0;
                double xz = (m[0][2] + m[2][0]) / 4.0;
                double yz = (m[1][2] + m[2][1]) / 4.0;

                if ((xx > yy) && (xx > zz))
                {
                    if (xx< epsilon)
                    {
                        r[0] = 0;    r[1] = r[2] = ha;
                    }
                    else
                    {
                        double t = sqrt(xx);
                        r[0] = FloatT(t * 3.14159265358979323846);
                        r[1] = FloatT(xy / t * 3.14159265358979323846);
                        r[2] = FloatT(xz / t * 3.14159265358979323846);
                    }
                }
                else if (yy > zz)
                {
                    if (yy < epsilon)
                    {
                        r[0] = r[2] = ha; r[1] = 0;
                    }
                    else
                    {
                        double t = sqrt(yy);
                        r[0] = FloatT(xy / t* 3.14159265358979323846);
                        r[1] = FloatT(t * 3.14159265358979323846);
                        r[2] = FloatT(yz / t* 3.14159265358979323846);
                    }
                }
                else
                {
                    if (zz < epsilon)
                    {
                        r[0] = r[1] = ha; r[2] = 0;
                    }
                    else
                    {
                        double t = sqrt(zz);
                        r[0] = FloatT(xz / t* 3.14159265358979323846);
                        r[1] = FloatT(yz / t* 3.14159265358979323846);
                        r[2] = FloatT(t * 3.14159265358979323846);
                    }
                }
            }
        }
        else
        {
            a = acos(a);
            double b = 0.5*a / sin(a);
            r[0] = FloatT(b*(m[2][1] - m[1][2]));
            r[1] = FloatT(b*(m[0][2] - m[2][0]));
            r[2] = FloatT(b*(m[1][0] - m[0][1]));
        }
    }

    ////////////////////////
    template <class FloatT> void SetQuaternionRotation(const FloatT q[4])
    {
        double qq = sqrt((double)(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]));
        double qw, qx, qy, qz;
        if (qq > 0)
        {
            qw = q[0] / qq;
            qx = q[1] / qq;
            qy = q[2] / qq;
            qz = q[3] / qq;
        }
        else
        {
            qw = 1;
            qx = qy = qz = 0;
        }
        m[0][0] = FloatT_(qw*qw + qx*qx - qz*qz - qy*qy);
        m[0][1] = FloatT_(2 * qx*qy - 2 * qz*qw);
        m[0][2] = FloatT_(2 * qy*qw + 2 * qz*qx);
        m[1][0] = FloatT_(2 * qx*qy + 2 * qw*qz);
        m[1][1] = FloatT_(qy*qy + qw*qw - qz*qz - qx*qx);
        m[1][2] = FloatT_(2 * qz*qy - 2 * qx*qw);
        m[2][0] = FloatT_(2 * qx*qz - 2 * qy*qw);
        m[2][1] = FloatT_(2 * qy*qz + 2 * qw*qx);
        m[2][2] = FloatT_(qz*qz + qw*qw - qy*qy - qx*qx);
    }

    template <class FloatT> void SetQuaternionRotation(FloatT qx, FloatT qy, FloatT qz, FloatT qw)
    {
        FloatT q[4] = { qw, qx, qy, qz };
        SetQuaternionRotation(q);
    }

    template<class FloatT> void GetQuaternionRotation(FloatT q[4]) const
    {
        q[0] = 1 + m[0][0] + m[1][1] + m[2][2];
        if (q[0] > 0.000000001)
        {
            q[0] = sqrt(q[0]) / 2.0;
            q[1] = (m[2][1] - m[1][2]) / (4.0 *q[0]);
            q[2] = (m[0][2] - m[2][0]) / (4.0 *q[0]);
            q[3] = (m[1][0] - m[0][1]) / (4.0 *q[0]);
        }
        else
        {
            double s;
            if (m[0][0] > m[1][1] && m[0][0] > m[2][2])
            {
                s = 2.0 * sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]);
                q[1] = 0.25 * s;
                q[2] = (m[0][1] + m[1][0]) / s;
                q[3] = (m[0][2] + m[2][0]) / s;
                q[0] = (m[1][2] - m[2][1]) / s;
            }
            else if (m[1][1] > m[2][2])
            {
                s = 2.0 * sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]);
                q[1] = (m[0][1] + m[1][0]) / s;
                q[2] = 0.25 * s;
                q[3] = (m[1][2] + m[2][1]) / s;
                q[0] = (m[0][2] - m[2][0]) / s;
            }
            else
            {
                s = 2.0 * sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]);
                q[1] = (m[0][2] + m[2][0]) / s;
                q[2] = (m[1][2] + m[2][1]) / s;
                q[3] = 0.25f * s;
                q[0] = (m[0][1] - m[1][0]) / s;
            }
        }
    }

    ////////////////////////////////////////////////
    template<class FloatT> void SetMatrixRotation(const FloatT * r)
    {
        for (int i = 0; i < 9; ++i) 
        {
            m[0][i] = FloatT_(r[i]); 
        }
    }

    template<class FloatT> void GetMatrixRotation(FloatT * r) const
    {
        for (int i = 0; i < 9; ++i)
        {
            r[i] = FloatT(m[0][i]);
        }
    }

    float GetRotationMatrixDeterminant()const
    {
        return m[0][0] * m[1][1] * m[2][2] +
            m[0][1] * m[1][2] * m[2][0] +
            m[0][2] * m[1][0] * m[2][1] -
            m[0][2] * m[1][1] * m[2][0] -
            m[0][1] * m[1][0] * m[2][2] -
            m[0][0] * m[1][2] * m[2][1];
    }

    ///////////////////////////////////////
    template<class FloatT> void SetTranslation(const FloatT T[3])
    {
        t[0] = (FloatT_)T[0];
        t[1] = (FloatT_)T[1];
        t[2] = (FloatT_)T[2];
    }

    template<class FloatT> void SetTranslation(FloatT tx, FloatT ty, FloatT tz)
    {
        t[0] = (FloatT_)tx;
        t[1] = (FloatT_)ty;
        t[2] = (FloatT_)tz;
    }

    template<class FloatT> void GetTranslation(FloatT T[3])  const
    {
        T[0] = (FloatT)t[0];
        T[1] = (FloatT)t[1];
        T[2] = (FloatT)t[2];
    }

    /////////////////////////////////////////////
    template<class FloatT> void SetPositionAfterRotation(const FloatT c[3])
    {
        //t = - R * C
        for (int j = 0; j < 3; ++j) 
        {
            t[j] = -FloatT_(double(m[j][0])*double(c[0]) + double(m[j][1])*double(c[1]) + double(m[j][2])*double(c[2]));
        }
    }

    template<class FloatT> void SetPositionAfterRotation(FloatT cx, FloatT cy, FloatT cz)
    {
        //t = - R * C
        for (int j = 0; j < 3; ++j) 
        {
            t[j] = -FloatT_(double(m[j][0])*double(cx) + double(m[j][1])*double(cy) + double(m[j][2])*double(cz));
        }
    }

    template<class FloatT> void GetPositionMatrix(FloatT c[3]) const
    {
        //C = - R' * t
        for (int j = 0; j < 3; ++j) 
        {
            c[j] = -FloatT(double(m[0][j])*double(t[0]) + double(m[1][j])*double(t[1]) + double(m[2][j])*double(t[2]));
        }
    }

    ////////////////////////////////////////////
    template<class FloatT> void SetInvertedRT(const FloatT e[3], const FloatT T[3])
    {
        SetRodriguesRotation(e);
        for (int i = 3; i < 9; ++i)
        {
            m[0][i] = -m[0][i];
        }
        SetTranslation(T); 
        t[1] = -t[1]; 
        t[2] = -t[2];
    }

    template<class FloatT> void GetInvertedRT(FloatT e[3], FloatT T[3]) const
    {
        CameraT ci;    
        ci.SetMatrixRotation(m[0]);
        for (int i = 3; i < 9; ++i)
        {
            ci.m[0][i] = -ci.m[0][i];
        }
        ci.GetRodriguesRotation(e);

        GetTranslation(T);    
        T[1] = -T[1]; 
        T[2] = -T[2];
    }

    template<class FloatT> void SetInvertedR9T(const FloatT e[9], const FloatT T[3])
    {
        //for(int i = 0; i < 9; ++i) m[0][i] = (i < 3 ? e[i] : - e[i]);
        //SetTranslation(T); t[1] = - t[1]; t[2] = -t[2];
        m[0][0] = e[0];        m[0][1] = e[1];      m[0][2] = e[2];
        m[1][0] = -e[3];       m[1][1] = -e[4];     m[1][2] = -e[5];
        m[2][0] = -e[6];       m[2][1] = -e[7];     m[2][2] = -e[8];
        t[0] = T[0];           t[1] = -T[1];        t[2] = -T[2];
    }

    template<class FloatT> void GetInvertedR9T(FloatT e[9], FloatT T[3]) const
    {
        e[0] = m[0][0];       e[1] = m[0][1];      e[2] = m[0][2];
        e[3] = -m[1][0];      e[4] = -m[1][1];     e[5] = -m[1][2];
        e[6] = -m[2][0];      e[7] = -m[2][1];     e[8] = -m[2][2];
        T[0] = t[0];          T[1] = -t[1];		   T[2] = -t[2];
    }
};

typedef CameraT_<float>  CameraF;
typedef CameraT_<double> CameraD;

} // namespace Ruler

#endif // _SCENERENDER_CAMERA_H_