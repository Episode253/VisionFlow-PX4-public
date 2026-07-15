#pragma once

/**
 * @file gamma_arm_dynamics_params.hpp
 */

#include "gamma_arm_dynamics.hpp"

namespace gamma_arm
{

inline ArmUavParam makeDefaultParam()
{
    ArmUavParam p;

    // ── UAV (from swan_uav_v2/model.sdf base_link inertial) ──
    p.m_uav = 7.8874f;
    p.pc_uav = matrix::Vector3f(-0.0751f, 0.f, -0.0322f);
    {
        auto &I = p.I_uav;
        I(0,0)=0.3655f; I(0,1)=0.f; I(0,2)=0.f;
        I(1,0)=0.f; I(1,1)=0.1895f; I(1,2)=0.f;
        I(2,0)=0.f; I(2,1)=0.f; I(2,2)=0.5488f;
    }

    // ── Mount pose (from swan_gamma_v2/model.sdf <include><pose>) ──
    p.p_mount = matrix::Vector3f(0.405f, -0.015f, -0.0295f);
    {
        const float roll = -2.6443f, pitch = 0.f, yaw = 0.f;
        const float cr = cosf(roll), sr = sinf(roll);
        const float cp = cosf(pitch), sp = sinf(pitch);
        const float cy = cosf(yaw), sy = sinf(yaw);
        {
            auto &R = p.R_mount;
            R(0,0)=cp*cy;  R(0,1)=sr*sp*cy-cr*sy;  R(0,2)=cr*sp*cy+sr*sy;
            R(1,0)=cp*sy;  R(1,1)=sr*sp*sy+cr*cy;  R(1,2)=cr*sp*sy-sr*cy;
            R(2,0)=-sp;    R(2,1)=sr*cp;            R(2,2)=cr*cp;
        }
    }

    // ── DH (Craig standard, manually maintained) ──
    p.dh[0] = {0.06407f, -kPi/2.f,  0.10429f,  kPi - 0.50f};
    p.dh[1] = {0.24873f,  0.f,       0.02305f,  0.f};
    p.dh[2] = {0.06301f,  kPi/2.f,  -0.025f,    0.f};
    p.dh[3] = {0.f,       kPi/2.f,   0.165f,    0.f};
    p.dh[4] = {0.f,       kPi/2.f,  -0.0015f,   0.f};
    p.dh[5] = {0.f,       0.f,       0.084f,     0.f};

    // ── base_link (from gamma_arm/model.sdf) ──
    p.base_link.m  = 0.361f;
    p.base_link.pc = matrix::Vector3f(0.026f, 0.f, 0.f);
    {
        auto &I = p.base_link.I;
        I(0,0)=0.00014566f; I(0,1)=0.f; I(0,2)=0.f;
        I(1,0)=0.f; I(1,1)=0.00015872f; I(1,2)=0.f;
        I(2,0)=0.f; I(2,1)=0.f; I(2,2)=0.00015872f;
    }

    // ── A_Link ──
    p.links[0].m  = 0.482f;
    p.links[0].pc = matrix::Vector3f(0.036314f, 0.007409f, 0.053527f);
    {
        auto &I = p.links[0].I;
        I(0,0)=0.00048916f; I(0,1)=0.f; I(0,2)=0.f;
        I(1,0)=0.f; I(1,1)=0.00049299f; I(1,2)=0.f;
        I(2,0)=0.f; I(2,1)=0.f; I(2,2)=0.0003462f;
    }

    // ── B_Link ──
    p.links[1].m  = 0.481f;
    p.links[1].pc = matrix::Vector3f(-0.14539f, -0.024762f, 0.16247f);
    {
        auto &I = p.links[1].I;
        I(0,0)=0.0015459f; I(0,1)=0.f; I(0,2)=0.f;
        I(1,0)=0.f; I(1,1)=0.0025514f; I(1,2)=0.f;
        I(2,0)=0.f; I(2,1)=0.f; I(2,2)=0.0012874f;
    }

    // ── C_Link ──
    p.links[2].m  = 0.239f;
    p.links[2].pc = matrix::Vector3f(0.0017414f, 0.023334f, 0.057853f);
    {
        auto &I = p.links[2].I;
        I(0,0)=0.00014499f; I(0,1)=0.f; I(0,2)=0.f;
        I(1,0)=0.f; I(1,1)=0.0001552f; I(1,2)=0.f;
        I(2,0)=0.f; I(2,1)=0.f; I(2,2)=0.00010724f;
    }

    // ── D_Link ──
    p.links[3].m  = 0.262f;
    p.links[3].pc = matrix::Vector3f(0.13241f, -0.004409f, 7.6e-05f);
    {
        auto &I = p.links[3].I;
        I(0,0)=0.00010245f; I(0,1)=0.f; I(0,2)=0.f;
        I(1,0)=0.f; I(1,1)=0.00052076f; I(1,2)=0.f;
        I(2,0)=0.f; I(2,1)=0.f; I(2,2)=0.0005369f;
    }

    // ── E_Link ──
    p.links[4].m  = 0.042397f;
    p.links[4].pc = matrix::Vector3f(1.62e-06f, 0.014793f, 0.051607f);
    {
        auto &I = p.links[4].I;
        I(0,0)=6.544e-05f; I(0,1)=0.f; I(0,2)=0.f;
        I(1,0)=0.f; I(1,1)=5.714e-05f; I(1,2)=0.f;
        I(2,0)=0.f; I(2,1)=0.f; I(2,2)=2.005e-05f;
    }

    // ── F_Link ──
    p.links[5].m  = 0.192f;
    p.links[5].pc = matrix::Vector3f(-4.643e-05f, 0.00010323f, -0.024394f);
    {
        auto &I = p.links[5].I;
        I(0,0)=8.297e-05f; I(0,1)=0.f; I(0,2)=0.f;
        I(1,0)=0.f; I(1,1)=8.274e-05f; I(1,2)=0.f;
        I(2,0)=0.f; I(2,1)=0.f; I(2,2)=6.854e-05f;
    }

    return p;
}

} // namespace gamma_arm
