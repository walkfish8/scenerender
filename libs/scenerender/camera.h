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
#include "opencv2/opencv.hpp"

namespace Ruler
{

//transfer data type with 4-float alignment
#define CameraT CameraT_

template<class FT>
struct CameraT_
{
    typedef FT float_t;
    //////////////////////////////////////////////////////
    float_t		t[3];				// T in  P = K[R T], T = - RC
    float_t		m[3][3];			// R in  P = K[R T]. 

    //////////////////////////////////////////////////////////
    CameraT_() 
    {
        t[0] = t[1] = t[2] = 0.0;
        m[0][0] = 1.0; m[0][1] = 0.0; m[0][2] = 0.0;
        m[1][0] = 0.0; m[1][1] = 1.0; m[1][2] = 0.0;
        m[2][0] = 0.0; m[2][1] = 0.0; m[2][2] = 1.0;
    }

    template <class Float> CameraT_(Float tx, Float ty, Float tz, Float qx, Float qy, Float qz, Float qw)
    {
        t[0] = (float_t)tx;
        t[1] = (float_t)ty;
        t[2] = (float_t)tz;

        Float q[4] = { qx, qy, qz, qw };
        SetQuaternionRotation(q);
    }

    //////////////////////////////////////////////
    template<class CameraX> void SetCameraT(const CameraX& cam)
    {
        t[0] = (float_t)cam.t[0];    t[1] = (float_t)cam.t[1];    t[2] = (float_t)cam.t[2];
        for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) m[i][j] = (float_t)cam.m[i][j];
    }

    //////////////////////////////////////////
    template <class Float> void SetRodriguesRotation(const Float r[3])
    {
        double a = sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
        double ct = a == 0.0 ? 0.5 : (1.0 - cos(a)) / a / a;
        double st = a == 0.0 ? 1 : sin(a) / a;
        m[0][0] = float_t(1.0 - (r[1] * r[1] + r[2] * r[2])*ct);
        m[0][1] = float_t(r[0] * r[1] * ct - r[2] * st);
        m[0][2] = float_t(r[2] * r[0] * ct + r[1] * st);
        m[1][0] = float_t(r[0] * r[1] * ct + r[2] * st);
        m[1][1] = float_t(1.0 - (r[2] * r[2] + r[0] * r[0])*ct);
        m[1][2] = float_t(r[1] * r[2] * ct - r[0] * st);
        m[2][0] = float_t(r[2] * r[0] * ct - r[1] * st);
        m[2][1] = float_t(r[1] * r[2] * ct + r[0] * st);
        m[2][2] = float_t(1.0 - (r[0] * r[0] + r[1] * r[1])*ct);
    }

    template <class Float> void SetRodriguesRotation(Float rx, Float ry, Float rz)
    {
        Float r[3] = { rx, ry, rz };
        SetRodriguesRotation(r);
    }

    template <class Float> void GetRodriguesRotation(Float r[3]) const
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
                const Float ha = Float(sqrt(0.5) * 3.14159265358979323846);
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
                        r[0] = Float(t * 3.14159265358979323846);
                        r[1] = Float(xy / t * 3.14159265358979323846);
                        r[2] = Float(xz / t * 3.14159265358979323846);
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
                        r[0] = Float(xy / t* 3.14159265358979323846);
                        r[1] = Float(t * 3.14159265358979323846);
                        r[2] = Float(yz / t* 3.14159265358979323846);
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
                        r[0] = Float(xz / t* 3.14159265358979323846);
                        r[1] = Float(yz / t* 3.14159265358979323846);
                        r[2] = Float(t * 3.14159265358979323846);
                    }
                }
            }
        }
        else
        {
            a = acos(a);
            double b = 0.5*a / sin(a);
            r[0] = Float(b*(m[2][1] - m[1][2]));
            r[1] = Float(b*(m[0][2] - m[2][0]));
            r[2] = Float(b*(m[1][0] - m[0][1]));
        }
    }

    ////////////////////////
    template <class Float> void SetQuaternionRotation(const Float q[4])
    {
        double qq = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
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
        m[0][0] = float_t(qw*qw + qx*qx - qz*qz - qy*qy);
        m[0][1] = float_t(2 * qx*qy - 2 * qz*qw);
        m[0][2] = float_t(2 * qy*qw + 2 * qz*qx);
        m[1][0] = float_t(2 * qx*qy + 2 * qw*qz);
        m[1][1] = float_t(qy*qy + qw*qw - qz*qz - qx*qx);
        m[1][2] = float_t(2 * qz*qy - 2 * qx*qw);
        m[2][0] = float_t(2 * qx*qz - 2 * qy*qw);
        m[2][1] = float_t(2 * qy*qz + 2 * qw*qx);
        m[2][2] = float_t(qz*qz + qw*qw - qy*qy - qx*qx);
    }

    template <class Float> void SetQuaternionRotation(Float qx, Float qy, Float qz, Float qw)
    {
        Float q[4] = { qx, qy, qz, qw };
        SetQuaternionRotation(q);
    }

    template<class Float> void GetQuaternionRotation(Float q[4]) const
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
    template<class Float> void SetMatrixRotation(const Float * r)
    {
        for (int i = 0; i < 9; ++i) m[0][i] = float_t(r[i]);
    }

    template<class Float> void GetMatrixRotation(Float * r) const
    {
        for (int i = 0; i < 9; ++i) r[i] = Float(m[0][i]);
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
    template<class Float> void SetTranslation(const Float T[3])
    {
        t[0] = (float_t)T[0];
        t[1] = (float_t)T[1];
        t[2] = (float_t)T[2];
    }

    template<class Float> void SetTranslation(Float tx, Float ty, Float tz)
    {
        t[0] = (float_t)tx;
        t[1] = (float_t)ty;
        t[2] = (float_t)tz;
    }

    template<class Float> void GetTranslation(Float T[3])  const
    {
        T[0] = (Float)t[0];
        T[1] = (Float)t[1];
        T[2] = (Float)t[2];
    }

    /////////////////////////////////////////////
    template<class Float> void SetPositionAfterRotation(const Float c[3])
    {
        //t = - R * C
        for (int j = 0; j < 3; ++j) t[j] = -float_t(double(m[j][0])*double(c[0]) + double(m[j][1])*double(c[1]) + double(m[j][2])*double(c[2]));
    }

    template<class Float> void SetPositionAfterRotation(Float cx, Float cy, Float cz)
    {
        //t = - R * C
        for (int j = 0; j < 3; ++j) t[j] = -float_t(double(m[j][0])*double(cx) + double(m[j][1])*double(cy) + double(m[j][2])*double(cz));
    }

    template<class Float> void GetPositionMatrix(Float c[3]) const
    {
        //C = - R' * t
        for (int j = 0; j < 3; ++j) c[j] = -Float(double(m[0][j])*double(t[0]) + double(m[1][j])*double(t[1]) + double(m[2][j])*double(t[2]));
    }

    ////////////////////////////////////////////
    template<class Float> void SetInvertedRT(const Float e[3], const Float T[3])
    {
        SetRodriguesRotation(e);
        for (int i = 3; i < 9; ++i) m[0][i] = -m[0][i];
        SetTranslation(T); t[1] = -t[1]; t[2] = -t[2];
    }

    template<class Float> void GetInvertedRT(Float e[3], Float T[3]) const
    {
        CameraT ci;    ci.SetMatrixRotation(m[0]);
        for (int i = 3; i < 9; ++i) ci.m[0][i] = -ci.m[0][i];
        //for(int i = 1; i < 3; ++i) for(int j = 0; j < 3; ++j) ci.m[i][j] = - ci.m[i][j];
        ci.GetRodriguesRotation(e);
        GetTranslation(T);    T[1] = -T[1]; T[2] = -T[2];
    }

    template<class Float> void SetInvertedR9T(const Float e[9], const Float T[3])
    {
        //for(int i = 0; i < 9; ++i) m[0][i] = (i < 3 ? e[i] : - e[i]);
        //SetTranslation(T); t[1] = - t[1]; t[2] = -t[2];
        m[0][0] = e[0];        m[0][1] = e[1];      m[0][2] = e[2];
        m[1][0] = -e[3];       m[1][1] = -e[4];     m[1][2] = -e[5];
        m[2][0] = -e[6];       m[2][1] = -e[7];     m[2][2] = -e[8];
        t[0] = T[0];           t[1] = -T[1];        t[2] = -T[2];
    }

    template<class Float> void GetInvertedR9T(Float e[9], Float T[3]) const
    {
        e[0] = m[0][0];        e[1] = m[0][1];      e[2] = m[0][2];
        e[3] = -m[1][0];      e[4] = -m[1][1];     e[5] = -m[1][2];
        e[6] = -m[2][0];       e[7] = -m[2][1];     e[8] = -m[2][2];
        T[0] = t[0];           T[1] = -t[1];		T[2] = -t[2];
    }

    cv::Mat GetRotationAndTranslationMatrix() const
    {
        cv::Mat P(4, 4, CV_64F);
        P.at<double>(0, 0) = m[0][0]; P.at<double>(0, 1) = m[0][1]; P.at<double>(0, 2) = m[0][2]; P.at<double>(0, 3) = t[0];
        P.at<double>(1, 0) = m[1][0]; P.at<double>(1, 1) = m[1][1]; P.at<double>(1, 2) = m[1][2]; P.at<double>(1, 3) = t[1];
        P.at<double>(2, 0) = m[2][0]; P.at<double>(2, 1) = m[2][1]; P.at<double>(2, 2) = m[2][2]; P.at<double>(2, 3) = t[2];
        P.at<double>(3, 0) = 0;       P.at<double>(3, 1) = 0;       P.at<double>(3, 2) = 0;       P.at<double>(3, 3) = 1;
        return std::move(P);
    }

    cv::Mat GetRotationMatrix() const
    {
        cv::Mat R(3, 3, CV_64F);
        R.at<double>(0, 0) = m[0][0]; R.at<double>(0, 1) = m[0][1]; R.at<double>(0, 2) = m[0][2];
        R.at<double>(1, 0) = m[1][0]; R.at<double>(1, 1) = m[1][1]; R.at<double>(1, 2) = m[1][2];
        R.at<double>(2, 0) = m[2][0]; R.at<double>(2, 1) = m[2][1]; R.at<double>(2, 2) = m[2][2];
        return std::move(R);
    }

    cv::Mat GetTranslationMatrix() const
    {
        cv::Mat T(3, 1, CV_64F);
        T.at<double>(0, 0) = t[0];
        T.at<double>(1, 0) = t[1];
        T.at<double>(2, 0) = t[2];
        return std::move(T);
    }

    cv::Mat GetPositionMatrix() const
    {
        cv::Mat C(3, 1, CV_64F);
        C.at<double>(0, 0) = -(double(m[0][0])*double(t[0]) + double(m[1][0])*double(t[1]) + double(m[2][0])*double(t[2]));
        C.at<double>(1, 0) = -(double(m[0][1])*double(t[0]) + double(m[1][1])*double(t[1]) + double(m[2][1])*double(t[2]));
        C.at<double>(2, 0) = -(double(m[0][2])*double(t[0]) + double(m[1][2])*double(t[1]) + double(m[2][2])*double(t[2]));
        return std::move(C);
    }
};

#undef CameraT
typedef CameraT_<float>  CameraF;
typedef CameraT_<double> CameraD;

} // namespace Ruler

#endif // _SCENERENDER_CAMERA_H_