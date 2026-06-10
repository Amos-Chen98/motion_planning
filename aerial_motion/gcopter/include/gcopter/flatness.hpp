/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef FLATNESS_HPP
#define FLATNESS_HPP

#include <Eigen/Eigen>

#include <cmath>

namespace flatness
{
    // Drag-free differential flatness map for a multicopter.
    // The thrust direction is acc + g*e3; attitude and body rates follow from
    // its normalization, so only the gravitational acceleration is needed.
    // See https://github.com/ZJU-FAST-Lab/GCOPTER/blob/main/misc/flatness.pdf
    class FlatnessMap
    {
    public:
        inline void reset(const double &gravitational_acceleration)
        {
            grav = gravitational_acceleration;

            return;
        }

        inline void forward(const Eigen::Vector3d &acc,
                            const Eigen::Vector3d &jer,
                            const double &psi,
                            const double &dpsi,
                            Eigen::Vector4d &quat,
                            Eigen::Vector3d &omg)
        {
            zu0 = acc(0);
            zu1 = acc(1);
            zu2 = acc(2) + grav;
            zu_sqr0 = zu0 * zu0;
            zu_sqr1 = zu1 * zu1;
            zu_sqr2 = zu2 * zu2;
            zu01 = zu0 * zu1;
            zu12 = zu1 * zu2;
            zu02 = zu0 * zu2;
            zu_sqr_norm = zu_sqr0 + zu_sqr1 + zu_sqr2;
            zu_norm = sqrt(zu_sqr_norm);
            z0 = zu0 / zu_norm;
            z1 = zu1 / zu_norm;
            z2 = zu2 / zu_norm;
            ng_den = zu_sqr_norm * zu_norm;
            ng00 = (zu_sqr1 + zu_sqr2) / ng_den;
            ng01 = -zu01 / ng_den;
            ng02 = -zu02 / ng_den;
            ng11 = (zu_sqr0 + zu_sqr2) / ng_den;
            ng12 = -zu12 / ng_den;
            ng22 = (zu_sqr0 + zu_sqr1) / ng_den;
            dz_term0 = jer(0);
            dz_term1 = jer(1);
            dz_term2 = jer(2);
            dz0 = ng00 * dz_term0 + ng01 * dz_term1 + ng02 * dz_term2;
            dz1 = ng01 * dz_term0 + ng11 * dz_term1 + ng12 * dz_term2;
            dz2 = ng02 * dz_term0 + ng12 * dz_term1 + ng22 * dz_term2;
            tilt_den = sqrt(2.0 * (1.0 + z2));
            tilt0 = 0.5 * tilt_den;
            tilt1 = -z1 / tilt_den;
            tilt2 = z0 / tilt_den;
            c_half_psi = cos(0.5 * psi);
            s_half_psi = sin(0.5 * psi);
            quat(0) = tilt0 * c_half_psi;
            quat(1) = tilt1 * c_half_psi + tilt2 * s_half_psi;
            quat(2) = tilt2 * c_half_psi - tilt1 * s_half_psi;
            quat(3) = tilt0 * s_half_psi;
            c_psi = cos(psi);
            s_psi = sin(psi);
            omg_den = z2 + 1.0;
            omg_term = dz2 / omg_den;
            omg(0) = dz0 * s_psi - dz1 * c_psi -
                     (z0 * s_psi - z1 * c_psi) * omg_term;
            omg(1) = dz0 * c_psi + dz1 * s_psi -
                     (z0 * c_psi + z1 * s_psi) * omg_term;
            omg(2) = (z1 * dz0 - z0 * dz1) / omg_den + dpsi;

            return;
        }

        inline void backward(const Eigen::Vector4d &quat_grad,
                             const Eigen::Vector3d &omg_grad,
                             Eigen::Vector3d &acc_total_grad,
                             Eigen::Vector3d &jer_total_grad,
                             double &psi_total_grad,
                             double &dpsi_total_grad) const
        {
            double dz0b, dz1b, dz2b;
            double z0b, z1b, z2b;
            double zu_sqr_normb, zu_normb, zu0b, zu1b, zu2b;
            double zu_sqr0b, zu_sqr1b, zu_sqr2b, zu01b, zu12b, zu02b;
            double ng00b, ng01b, ng02b, ng11b, ng12b, ng22b, ng_denb;
            double dz_term0b, dz_term1b, dz_term2b;
            double tilt_denb, tilt0b, tilt1b, tilt2b, head0b, head3b;
            double cpsib, spsib, omg_denb, omg_termb;
            double tempb, tilt_den_sqr;

            tilt0b = s_half_psi * (quat_grad(3)) + c_half_psi * (quat_grad(0));
            head3b = tilt0 * (quat_grad(3)) + tilt2 * (quat_grad(1)) - tilt1 * (quat_grad(2));
            tilt2b = c_half_psi * (quat_grad(2)) + s_half_psi * (quat_grad(1));
            head0b = tilt2 * (quat_grad(2)) + tilt1 * (quat_grad(1)) + tilt0 * (quat_grad(0));
            tilt1b = c_half_psi * (quat_grad(1)) - s_half_psi * (quat_grad(2));
            tilt_den_sqr = tilt_den * tilt_den;
            tilt_denb = (z1 * tilt1b - z0 * tilt2b) / tilt_den_sqr + 0.5 * tilt0b;
            omg_termb = -((z0 * c_psi + z1 * s_psi) * (omg_grad(1))) -
                        (z0 * s_psi - z1 * c_psi) * (omg_grad(0));
            tempb = omg_grad(2) / omg_den;
            dpsi_total_grad = omg_grad(2);
            z1b = dz0 * tempb;
            dz0b = z1 * tempb + c_psi * (omg_grad(1)) + s_psi * (omg_grad(0));
            z0b = -(dz1 * tempb);
            dz1b = s_psi * (omg_grad(1)) - z0 * tempb - c_psi * (omg_grad(0));
            omg_denb = -((z1 * dz0 - z0 * dz1) * tempb / omg_den) -
                       dz2 * omg_termb / (omg_den * omg_den);
            tempb = -(omg_term * (omg_grad(1)));
            cpsib = dz0 * (omg_grad(1)) + z0 * tempb;
            spsib = dz1 * (omg_grad(1)) + z1 * tempb;
            z0b += c_psi * tempb;
            z1b += s_psi * tempb;
            tempb = -(omg_term * (omg_grad(0)));
            spsib += dz0 * (omg_grad(0)) + z0 * tempb;
            cpsib += -dz1 * (omg_grad(0)) - z1 * tempb;
            z0b += s_psi * tempb + tilt2b / tilt_den;
            z1b += -c_psi * tempb - tilt1b / tilt_den;
            dz2b = omg_termb / omg_den;
            z2b = omg_denb + tilt_denb / tilt_den;
            psi_total_grad = c_psi * spsib + 0.5 * c_half_psi * head3b -
                             s_psi * cpsib - 0.5 * s_half_psi * head0b;
            ng02b = dz_term0 * dz2b + dz_term2 * dz0b;
            dz_term0b = ng02 * dz2b + ng01 * dz1b + ng00 * dz0b;
            ng12b = dz_term1 * dz2b + dz_term2 * dz1b;
            dz_term1b = ng12 * dz2b + ng11 * dz1b + ng01 * dz0b;
            ng22b = dz_term2 * dz2b;
            dz_term2b = ng22 * dz2b + ng12 * dz1b + ng02 * dz0b;
            ng01b = dz_term0 * dz1b + dz_term1 * dz0b;
            ng11b = dz_term1 * dz1b;
            ng00b = dz_term0 * dz0b;
            jer_total_grad(2) = dz_term2b;
            jer_total_grad(1) = dz_term1b;
            jer_total_grad(0) = dz_term0b;
            tempb = ng22b / ng_den;
            zu_sqr0b = tempb;
            zu_sqr1b = tempb;
            ng_denb = -((zu_sqr0 + zu_sqr1) * tempb / ng_den);
            zu12b = -(ng12b / ng_den);
            tempb = ng11b / ng_den;
            ng_denb += zu12 * ng12b / (ng_den * ng_den) -
                       (zu_sqr0 + zu_sqr2) * tempb / ng_den;
            zu_sqr0b += tempb;
            zu_sqr2b = tempb;
            zu02b = -(ng02b / ng_den);
            zu01b = -(ng01b / ng_den);
            tempb = ng00b / ng_den;
            ng_denb += zu02 * ng02b / (ng_den * ng_den) +
                       zu01 * ng01b / (ng_den * ng_den) -
                       (zu_sqr1 + zu_sqr2) * tempb / ng_den;
            zu_normb = zu_sqr_norm * ng_denb -
                       (zu2 * z2b + zu1 * z1b + zu0 * z0b) / zu_sqr_norm;
            zu_sqr_normb = zu_norm * ng_denb + zu_normb / (2.0 * zu_norm);
            tempb += zu_sqr_normb;
            zu_sqr1b += tempb;
            zu_sqr2b += tempb;
            zu2b = z2b / zu_norm + zu0 * zu02b + zu1 * zu12b + 2 * zu2 * zu_sqr2b;
            zu1b = z1b / zu_norm + zu2 * zu12b + zu0 * zu01b + 2 * zu1 * zu_sqr1b;
            zu_sqr0b += zu_sqr_normb;
            zu0b = z0b / zu_norm + zu2 * zu02b + zu1 * zu01b + 2 * zu0 * zu_sqr0b;
            acc_total_grad(2) = zu2b;
            acc_total_grad(1) = zu1b;
            acc_total_grad(0) = zu0b;

            return;
        }

    private:
        double grav;

        double z0, z1, z2, dz0, dz1, dz2;
        double zu_sqr_norm, zu_norm, zu0, zu1, zu2;
        double zu_sqr0, zu_sqr1, zu_sqr2, zu01, zu12, zu02;
        double ng00, ng01, ng02, ng11, ng12, ng22, ng_den;
        double dz_term0, dz_term1, dz_term2;
        double tilt_den, tilt0, tilt1, tilt2, c_half_psi, s_half_psi;
        double c_psi, s_psi, omg_den, omg_term;
    };
}

#endif
