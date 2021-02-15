// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Michael Taylor, Rainer Gericke
// =============================================================================
//
// Template for a tire model based on the Pacejka 2002 Tire Model
//
// =============================================================================
// =============================================================================
// STILL UNDERDEVELOPMENT
// =============================================================================
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono/core/ChGlobal.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChMFTyre62.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChMFTyre62::ChMFTyre62(const std::string& name)
    : ChTire(name),
      m_kappa(0),
      m_alpha(0),
      m_gamma(0),
      m_gamma_limit(3.0 * CH_C_DEG_TO_RAD),
      m_mu(0),
      m_use_mode(1),
      m_Shf(0),
      m_use_friction_ellipsis(false),
      m_allow_mirroring(false),
      m_measured_side(LEFT) {
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.point = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);

    p1 = 0.0;
    p2 = 0.0;
    p3 = 0.0;
    p4 = 0.0;
    p5 = 0.0;
    p6 = 0.0;
    p7 = 0.0;
    p8 = 0.0;

    // standard settings for scaling factors
    m_PacScal.lfz0 = 1.0;
    m_PacScal.lcx = 1.0;
    m_PacScal.lmux = 1.0;
    m_PacScal.lex = 1.0;
    m_PacScal.lkx = 1.0;
    m_PacScal.lhx = 1.0;
    m_PacScal.lvx = 1.0;
    m_PacScal.lcy = 1.0;
    m_PacScal.lmuy = 1.0;
    m_PacScal.ley = 1.0;
    m_PacScal.lky = 1.0;
    m_PacScal.lkyc = 1.0;
    m_PacScal.lkzc = 1.0;
    m_PacScal.lhy = 1.0;
    m_PacScal.lvy = 1.0;
    m_PacScal.ltr = 1.0;
    m_PacScal.lres = 1.0;
    m_PacScal.lxal = 1.0;
    m_PacScal.lyka = 1.0;
    m_PacScal.lvyka = 1.0;
    m_PacScal.ls = 1.0;
    m_PacScal.lmx = 1.0;
    m_PacScal.lvmx = 1.0;
    m_PacScal.lmy = 1.0;
    m_PacScal.lmp = 1.0;

    m_PacCoeff.mu0 = 0.8;           // reference friction coefficient
    m_PacCoeff.R0 = 0.0;            // unloaded radius
    m_PacCoeff.width = 0.0;         // tire width = 0.0;
    m_PacCoeff.aspect_ratio = 0.8;  // actually unused
    m_PacCoeff.rim_radius = 0.0;    // actually unused
    m_PacCoeff.rim_width = 0.0;     // actually unused
    m_PacCoeff.InflPres = 0.0;      // tire inflation pressure
    m_PacCoeff.NomPres = 0.0;       // nominal tire inflation pressure
    m_PacCoeff.FzNomin = 0.0;       // nominla wheel load
    m_PacCoeff.Cz = 0.0;            // vertical tire stiffness
    m_PacCoeff.Kz = 0.0;            // vertical tire damping

    // Longitudinal Coefficients
    m_PacCoeff.pcx1 = 0.0;
    m_PacCoeff.pdx1 = 0.0;
    m_PacCoeff.pdx2 = 0.0;
    m_PacCoeff.pdx3 = 0.0;
    m_PacCoeff.pex1 = 0.0;
    m_PacCoeff.pex2 = 0.0;
    m_PacCoeff.pex3 = 0.0;
    m_PacCoeff.pex4 = 0.0;
    m_PacCoeff.pkx1 = 0.0;
    m_PacCoeff.pkx2 = 0.0;
    m_PacCoeff.pkx3 = 0.0;
    m_PacCoeff.phx1 = 0.0;
    m_PacCoeff.phx2 = 0.0;
    m_PacCoeff.pvx1 = 0.0;
    m_PacCoeff.pvx2 = 0.0;
    m_PacCoeff.ppx1 = 0.0;
    m_PacCoeff.ppx2 = 0.0;
    m_PacCoeff.ppx3 = 0.0;
    m_PacCoeff.ppx4 = 0.0;
    m_PacCoeff.rbx1 = 0.0;
    m_PacCoeff.rbx2 = 0.0;
    m_PacCoeff.rbx3 = 0.0;
    m_PacCoeff.rcx1 = 0.0;
    m_PacCoeff.rex1 = 0.0;
    m_PacCoeff.rex2 = 0.0;
    m_PacCoeff.rhx1 = 0.0;

    // overturning coefficients
    m_PacCoeff.qsx1 = 0.0;
    m_PacCoeff.qsx2 = 0.0;
    m_PacCoeff.qsx3 = 0.0;
    m_PacCoeff.qsx4 = 0.0;
    m_PacCoeff.qsx5 = 0.0;
    m_PacCoeff.qsx6 = 0.0;
    m_PacCoeff.qsx7 = 0.0;
    m_PacCoeff.qsx8 = 0.0;
    m_PacCoeff.qsx9 = 0.0;
    m_PacCoeff.qsx10 = 0.0;
    m_PacCoeff.qsx11 = 0.0;
    m_PacCoeff.qsx12 = 0.0;
    m_PacCoeff.qsx13 = 0.0;
    m_PacCoeff.qsx14 = 0.0;
    m_PacCoeff.ppmx1 = 0.0;

    // rolling coefficients
    m_PacCoeff.qsy1 = 0.0;
    m_PacCoeff.qsy2 = 0.0;
    m_PacCoeff.qsy3 = 0.0;
    m_PacCoeff.qsy4 = 0.0;
    m_PacCoeff.qsy5 = 0.0;
    m_PacCoeff.qsy6 = 0.0;
    m_PacCoeff.qsy7 = 0.0;
    m_PacCoeff.qsy8 = 0.0;

    // Lateral Coefficients
    m_PacCoeff.pcy1 = 0.0;
    m_PacCoeff.pdy1 = 0.0;
    m_PacCoeff.pdy2 = 0.0;
    m_PacCoeff.pdy3 = 0.0;
    m_PacCoeff.pey1 = 0.0;
    m_PacCoeff.pey2 = 0.0;
    m_PacCoeff.pey3 = 0.0;
    m_PacCoeff.pey4 = 0.0;
    m_PacCoeff.pey5 = 0.0;
    m_PacCoeff.pky1 = 0.0;
    m_PacCoeff.pky2 = 0.0;
    m_PacCoeff.pky3 = 0.0;
    m_PacCoeff.pky4 = 0.0;
    m_PacCoeff.pky5 = 0.0;
    m_PacCoeff.pky6 = 0.0;
    m_PacCoeff.pky7 = 0.0;
    m_PacCoeff.phy1 = 0.0;
    m_PacCoeff.phy2 = 0.0;
    m_PacCoeff.pvy1 = 0.0;
    m_PacCoeff.pvy2 = 0.0;
    m_PacCoeff.pvy3 = 0.0;
    m_PacCoeff.pvy4 = 0.0;
    m_PacCoeff.ppy1 = 0.0;
    m_PacCoeff.ppy2 = 0.0;
    m_PacCoeff.ppy3 = 0.0;
    m_PacCoeff.ppy4 = 0.0;
    m_PacCoeff.ppy5 = 0.0;
    m_PacCoeff.rby1 = 0.0;
    m_PacCoeff.rby2 = 0.0;
    m_PacCoeff.rby3 = 0.0;
    m_PacCoeff.rby4 = 0.0;
    m_PacCoeff.rcy1 = 0.0;
    m_PacCoeff.rey1 = 0.0;
    m_PacCoeff.rey2 = 0.0;
    m_PacCoeff.rhy1 = 0.0;
    m_PacCoeff.rhy2 = 0.0;
    m_PacCoeff.rvy1 = 0.0;
    m_PacCoeff.rvy2 = 0.0;
    m_PacCoeff.rvy3 = 0.0;
    m_PacCoeff.rvy4 = 0.0;
    m_PacCoeff.rvy5 = 0.0;
    m_PacCoeff.rvy6 = 0.0;

    // alignment coefficients
    m_PacCoeff.qbz1 = 0.0;
    m_PacCoeff.qbz2 = 0.0;
    m_PacCoeff.qbz3 = 0.0;
    m_PacCoeff.qbz4 = 0.0;
    m_PacCoeff.qbz5 = 0.0;
    m_PacCoeff.qbz9 = 0.0;
    m_PacCoeff.qbz10 = 0.0;
    m_PacCoeff.qcz1 = 0.0;
    m_PacCoeff.qdz1 = 0.0;
    m_PacCoeff.qdz2 = 0.0;
    m_PacCoeff.qdz3 = 0.0;
    m_PacCoeff.qdz4 = 0.0;
    m_PacCoeff.qdz6 = 0.0;
    m_PacCoeff.qdz7 = 0.0;
    m_PacCoeff.qdz8 = 0.0;
    m_PacCoeff.qdz9 = 0.0;
    m_PacCoeff.qdz10 = 0.0;
    m_PacCoeff.qdz11 = 0.0;
    m_PacCoeff.qez1 = 0.0;
    m_PacCoeff.qez2 = 0.0;
    m_PacCoeff.qez3 = 0.0;
    m_PacCoeff.qez4 = 0.0;
    m_PacCoeff.qez5 = 0.0;
    m_PacCoeff.qhz1 = 0.0;
    m_PacCoeff.qhz2 = 0.0;
    m_PacCoeff.qhz3 = 0.0;
    m_PacCoeff.qhz4 = 0.0;
    m_PacCoeff.ppz1 = 0.0;
    m_PacCoeff.ppz2 = 0.0;
    m_PacCoeff.ssz1 = 0.0;
    m_PacCoeff.ssz2 = 0.0;
    m_PacCoeff.ssz3 = 0.0;
    m_PacCoeff.ssz4 = 0.0;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMFTyre62::Initialize(std::shared_ptr<ChBody> wheel, VehicleSide side) {
    ChTire::Initialize(wheel, side);

    SetMFTyre62Params();
    // Build the lookup table for penetration depth as function of intersection area
    // (used only with the ChTire::ENVELOPE method for terrain-tire collision detection)
    ConstructAreaDepthTable(m_PacCoeff.R0, m_areaDep);

    // all parameters are known now pepare mirroring
    if (m_allow_mirroring) {
        if (m_side != m_measured_side) {
            // we flip the sign of some parameters
            m_PacCoeff.rhx1 *= -1.0;
            m_PacCoeff.qsx1 *= -1.0;
            m_PacCoeff.pey3 *= -1.0;
            m_PacCoeff.phy1 *= -1.0;
            m_PacCoeff.phy2 *= -1.0;
            m_PacCoeff.pvy1 *= -1.0;
            m_PacCoeff.pvy2 *= -1.0;
            m_PacCoeff.rby3 *= -1.0;
            m_PacCoeff.rvy1 *= -1.0;
            m_PacCoeff.rvy2 *= -1.0;
            m_PacCoeff.qbz4 *= -1.0;
            m_PacCoeff.qdz3 *= -1.0;
            m_PacCoeff.qdz6 *= -1.0;
            m_PacCoeff.qdz7 *= -1.0;
            m_PacCoeff.qez4 *= -1.0;
            m_PacCoeff.qhz1 *= -1.0;
            m_PacCoeff.qhz2 *= -1.0;
            m_PacCoeff.ssz1 *= -1.0;
            if (m_measured_side == LEFT) {
                GetLog() << "Tire is measured as left tire but mounted on the right vehicle side -> mirroring.\n";
            } else {
                GetLog() << "Tire is measured as right tire but mounted on the lleft vehicle side -> mirroring.\n";
            }
        }
    }

    // Initialize contact patch state variables to 0
    m_data.normal_force = 0;
    m_states.R_eff = m_PacCoeff.R0;
    m_states.cp_long_slip = 0;
    m_states.cp_side_slip = 0;
    m_states.vx = 0;
    m_states.vsx = 0;
    m_states.vsy = 0;
    m_states.omega = 0;
    m_states.disc_normal = ChVector<>(0, 0, 0);

    m_states.cp_long_slip = 0;
    m_states.cp_side_slip = 0;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMFTyre62::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    m_cyl_shape = chrono_types::make_shared<ChCylinderShape>();
    m_cyl_shape->GetCylinderGeometry().rad = m_PacCoeff.R0;
    m_cyl_shape->GetCylinderGeometry().p1 = ChVector<>(0, GetVisualizationWidth() / 2, 0);
    m_cyl_shape->GetCylinderGeometry().p2 = ChVector<>(0, -GetVisualizationWidth() / 2, 0);
    m_wheel->AddAsset(m_cyl_shape);

    m_texture = chrono_types::make_shared<ChTexture>();
    m_texture->SetTextureFilename(GetChronoDataFile("greenwhite.png"));
    m_wheel->AddAsset(m_texture);
}

void ChMFTyre62::RemoveVisualizationAssets() {
    // Make sure we only remove the assets added by ChMFTyre62::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto& assets = m_wheel->GetAssets();
    {
        auto it = std::find(assets.begin(), assets.end(), m_cyl_shape);
        if (it != assets.end())
            assets.erase(it);
    }
    {
        auto it = std::find(assets.begin(), assets.end(), m_texture);
        if (it != assets.end())
            assets.erase(it);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMFTyre62::Synchronize(double time,
                              const WheelState& wheel_state,
                              const ChTerrain& terrain,
                              CollisionType collision_type) {
    // Invoke the base class function.
    ChTire::Synchronize(time, wheel_state, terrain, collision_type);

    m_mu = terrain.GetCoefficientFriction(m_tireforce.point.x(), m_tireforce.point.y());

    ChCoordsys<> contact_frame;
    // Clear the force accumulators and set the application point to the wheel
    // center.
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);
    m_tireforce.point = wheel_state.pos;

    // Extract the wheel normal (expressed in global frame)
    ChMatrix33<> A(wheel_state.rot);
    ChVector<> disc_normal = A.Get_A_Yaxis();

    double dum_cam;

    // Assuming the tire is a disc, check contact with terrain
    switch (collision_type) {
        case ChTire::CollisionType::SINGLE_POINT:
            m_data.in_contact =
                DiscTerrainCollision(terrain, wheel_state.pos, disc_normal, m_PacCoeff.R0, m_data.frame, m_data.depth);
            break;
        case ChTire::CollisionType::FOUR_POINTS:
            m_data.in_contact = DiscTerrainCollision4pt(terrain, wheel_state.pos, disc_normal, m_PacCoeff.R0,
                                                        m_PacCoeff.width, m_data.frame, m_data.depth, dum_cam);
            break;
        case ChTire::CollisionType::ENVELOPE:
            m_data.in_contact = DiscTerrainCollisionEnvelope(terrain, wheel_state.pos, disc_normal, m_PacCoeff.R0,
                                                             m_areaDep, m_data.frame, m_data.depth);
            break;
    }
    if (m_data.in_contact) {
        // Wheel velocity in the ISO-C Frame
        ChVector<> vel = wheel_state.lin_vel;
        m_data.vel = m_data.frame.TransformDirectionParentToLocal(vel);

        // Generate normal contact force (recall, all forces are reduced to the wheel
        // center). If the resulting force is negative, the disc is moving away from
        // the terrain so fast that no contact force is generated.
        // The sign of the velocity term in the damping function is negative since
        // a positive velocity means a decreasing depth, not an increasing depth
        double Fn_mag = GetNormalStiffnessForce(m_data.depth) + GetNormalDampingForce(m_data.depth, -m_data.vel.z());

        if (Fn_mag < 0) {
            Fn_mag = 0;
            m_data.in_contact = false;  // Skip Force and moment calculations when the normal force = 0
        }

        m_data.normal_force = Fn_mag;
        m_states.R_eff = m_PacCoeff.R0 - m_data.depth;
        m_states.vx = std::abs(m_data.vel.x());
        m_states.vsx = m_data.vel.x() - wheel_state.omega * m_states.R_eff;
        m_states.vsy = -m_data.vel.y();  // PAC89 is defined in a modified SAE coordinate system
        m_states.omega = wheel_state.omega;
        m_states.disc_normal = disc_normal;
    } else {
        // Reset all states if the tire comes off the ground.
        m_data.normal_force = 0;
        m_states.R_eff = m_PacCoeff.R0;
        m_states.cp_long_slip = 0;
        m_states.cp_side_slip = 0;
        m_states.vx = 0;
        m_states.vsx = 0;
        m_states.vsy = 0;
        m_states.omega = 0;
        m_states.disc_normal = ChVector<>(0, 0, 0);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMFTyre62::Advance(double step) {
    // Set tire forces to zero.
    //m_tireforce.point = m_wheel->GetPos();
    //m_tireforce.force = ChVector<>(0, 0, 0);
    //m_tireforce.moment = ChVector<>(0, 0, 0);

    // Return now if no contact.
    if (!m_data.in_contact)
        return;

    // prevent singularity for kappa, when vx == 0
    const double epsilon = 0.1;
    m_states.cp_long_slip = -m_states.vsx / (m_states.vx + epsilon);

    if (m_states.omega != 0) {
        m_states.cp_side_slip = std::atan(m_states.vsy / std::abs(m_states.omega * (m_PacCoeff.R0 - m_data.depth)));
    } else {
        m_states.cp_side_slip = 0;
    }

    // Ensure that cp_lon_slip stays between -1 & 1
    ChClampValue(m_states.cp_long_slip, -1.0, 1.0);

    // Ensure that cp_side_slip stays between -pi()/2 & pi()/2 (a little less to prevent tan from going to infinity)
    ChClampValue(m_states.cp_side_slip, -CH_C_PI_2 + 0.001, CH_C_PI_2 - 0.001);

    // Calculate the new force and moment values (normal force and moment have already been accounted for in
    // Synchronize()).
    // Express Fz in kN (note that all other forces and moments are in N and Nm).
    // See reference for details on the calculations.
    double Fx = 0;
    double Fy = 0;
    double Fz = m_data.normal_force;
    double Mx = 0;
    double My = 0;
    double Mz = 0;

    // Express alpha and gamma in rad. Express kappa as ratio.
    m_gamma = CH_C_PI_2 - std::acos(m_states.disc_normal.z());
    m_alpha = m_states.cp_side_slip;
    m_kappa = m_states.cp_long_slip;

    // Clamp |gamma| to specified value: Limit due to tire testing, avoids erratic extrapolation. m_gamma_limit is
    // in rad too.
    double gamma = ChClamp(m_gamma, -m_gamma_limit, m_gamma_limit);
    
    switch (m_use_mode) {
        case 0:
            // vertical spring & damper mode
            break;
        case 1:
            // steady state pure longitudinal slip
            Fx = CalcFx(m_kappa, Fz, gamma);
            break;
        case 2:
            // steady state pure lateral slip
            Fy = CalcFy(m_alpha, Fz, gamma);
            break;
        case 3:
            // steady state pure lateral slip uncombined
            Fx = CalcFx(m_kappa, Fz, gamma);
            Fy = CalcFy(m_alpha, Fz, gamma);
            Mx = CalcMx(Fy, Fz, gamma);
            My = CalcMx(Fx, Fz, gamma);
            CalcFyComb(m_kappa, m_alpha, Fz, gamma, &p1, &p2, &p3, &p4, &p5, &p6, &p7);
            Mz = CalcMz(m_alpha, Fz, gamma, &p1, &p2, &p3, &p4, &p5, &p6, &p7);
            break;
        case 4:
            // steady state combined slip
            Fx = CalcFxComb(m_kappa, m_alpha, Fz, gamma, &p8);
            Fy = CalcFyComb(m_kappa, m_alpha, Fz, gamma);
            Mx = CalcMx(Fy, Fz, gamma);
            My = CalcMx(Fx, Fz, gamma);
            CalcFyComb(m_kappa, m_alpha, Fz, gamma, &p1, &p2, &p3, &p4, &p5, &p6, &p7);
            Mz = CalcMzComb(m_kappa, m_alpha, Fz, gamma, Fx, Fy, &p1, &p2, &p3, &p4, &p5, &p6, &p7, &p8);
            break;
    }

    // Compile the force and moment vectors so that they can be
    // transformed into the global coordinate system.
    // Convert from SAE to ISO Coordinates at the contact patch.
    m_tireforce.force = ChVector<>(Fx, -Fy, m_data.normal_force);
    m_tireforce.moment = ChVector<>(Mx, -My, -Mz);

    // Rotate into global coordinates
    m_tireforce.force = m_data.frame.TransformDirectionLocalToParent(m_tireforce.force);
    m_tireforce.moment = m_data.frame.TransformDirectionLocalToParent(m_tireforce.moment);

    // Move the tire forces from the contact patch to the wheel center
    m_tireforce.moment +=
        Vcross((m_data.frame.pos + m_data.depth * m_data.frame.rot.GetZaxis()) - m_tireforce.point, m_tireforce.force);
    //GetLog() << "tire_force_y = " << m_tireforce.force.y() << "\n\n";
}

double ChMFTyre62::CalcFx(double kappa, double Fz, double gamma) {
    // calculates the longitudinal force based on a limited parameter set.
    // Pi is not considered
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double dFz = (Fz - Fz0s) / Fz0s;
    double dPi = 0.0;
    if (m_PacCoeff.InflPres != m_PacCoeff.NomPres) {
        double dPi = (m_PacCoeff.InflPres - m_PacCoeff.NomPres) / m_PacCoeff.NomPres;
    }
    double C = m_PacCoeff.pcx1 * m_PacScal.lcx;
    double Mu = (m_PacCoeff.pdx1 + m_PacCoeff.pdx2 * dFz) * (1.0 - m_PacCoeff.pdx3 * pow(gamma, 2)) * (1.0 + m_PacCoeff.ppx3 * dPi + m_PacCoeff.ppx4 * pow(dPi, 2)) * m_PacScal.lmux;
    double D = Mu * Fz * m_mu / m_PacCoeff.mu0;
    double Sh = (m_PacCoeff.phx1 + m_PacCoeff.phx2 * dFz) * m_PacScal.lhx;
    double kappa_x = kappa + Sh;
    double E = (m_PacCoeff.pex1 + m_PacCoeff.pex2 * dFz + m_PacCoeff.pex3 * pow(dFz, 2)) * (1.0 - m_PacCoeff.pex4 * ChSignum(kappa_x)) * m_PacScal.lex;
    double K = (m_PacCoeff.pkx1 + m_PacCoeff.pkx2 * dFz) * exp(m_PacCoeff.pkx3 * dFz) * (1.0 + m_PacCoeff.ppx1 * dPi + m_PacCoeff.ppx2 * pow(dPi, 2)) * Fz * m_PacScal.lkx;  // BCD = Kx
    double B = K / (C * D);
    double Sv = (m_PacCoeff.pvx1 + m_PacCoeff.pvx2 * dFz) * Fz * m_PacScal.lvx * m_PacScal.lmux;
    double X1 = B * kappa_x;
    double Fx0 = D * sin(C * atan(X1 - E * (X1 - atan(X1)))) + Sv;
    return Fx0;
}

double ChMFTyre62::CalcFy(double alpha, double Fz, double gamma) {
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double dFz = (Fz - Fz0s) / Fz0s;
    double dPi = 0.0;
    if (m_PacCoeff.InflPres != m_PacCoeff.NomPres) {
        double dPi = (m_PacCoeff.InflPres - m_PacCoeff.NomPres) / m_PacCoeff.NomPres;
    }
    double C = m_PacCoeff.pcy1 * m_PacScal.lcy;
    double Mu = (m_PacCoeff.pdy1 + m_PacCoeff.pdy2 * dFz) * (1.0 - m_PacCoeff.pdy3 * pow(gamma, 2)) * (1.0 + m_PacCoeff.ppy3 * dPi + m_PacCoeff.ppy4 * pow(dPi, 2)) * m_PacScal.lmuy;
    double D = Mu * Fz * m_mu / m_PacCoeff.mu0;
    double Ka = m_PacCoeff.pky1 * Fz0s * (1.0 + m_PacCoeff.ppy1 * dPi) * sin(m_PacCoeff.pky4 * atan(Fz / ((m_PacCoeff.pky2 + m_PacCoeff.pky5 * pow(gamma, 2)) * (1.0 + m_PacCoeff.ppy2 * dPi) * Fz0s))) * (1.0 - m_PacCoeff.pky3 * std::abs(gamma)) * m_PacScal.lky;
    double Kg = (m_PacCoeff.pky6 + m_PacCoeff.pky7 * dFz) * (1.0 + m_PacCoeff.ppy5 * dPi) * Fz * m_PacScal.lkyc;
    double B = Ka / (C * D);
    double Sv0 = Fz * (m_PacCoeff.pvy1 + m_PacCoeff.pvy2 * dFz) * m_PacScal.lvy * m_PacScal.lmuy;
    double Svg = Fz * (m_PacCoeff.pvy3 + m_PacCoeff.pvy4 * dFz) * gamma * m_PacScal.lkyc * m_PacScal.lmuy;
    double Sv = Sv0 + Svg;
    double Sh0 = (m_PacCoeff.phy1 + m_PacCoeff.phy2 * dFz) * m_PacScal.lhy;
    double Shg = (Kg * gamma - Svg) / Ka;
    double Sh = Sh0 + Shg;
    double alpha_y = alpha + Sh;
    double X1 = ChClamp(B * alpha_y, -CH_C_PI_2 + 0.001, CH_C_PI_2 - 0.001);  // Ensure that X1 stays within +/-90 deg minus a little bit
    double E = (m_PacCoeff.pey1 + m_PacCoeff.pey2 * dFz) *
               (1.0 + m_PacCoeff.pey5 * pow(gamma, 2) - (m_PacCoeff.pey3 + m_PacCoeff.pey4 * gamma) * ChSignum(alpha_y)) * m_PacScal.ley;
    double Fy0 = D * sin(C * atan(X1 - E * (X1 - atan(X1)))) + Sv;
    //GetLog() << "Fz0s = " << Fz0s << "\n\n";
    //GetLog() << "dFz = " << dFz << "\n\n";
    //GetLog() << "dPi = " << dPi << "\n\n";
    //GetLog() << "C = " << C << "\n\n";
    //GetLog() << "Mu = " << Mu << "\n\n";
    //GetLog() << "D = " << D << "\n\n";
    //GetLog() << "Ka = " << Ka << "\n\n";
    //GetLog() << "Kg = " << Kg << "\n\n";
    //GetLog() << "B = " << B << "\n\n";
    //GetLog() << "Sv0 = " << Sv0 << "\n\n";
    //GetLog() << "Svg = " << Svg << "\n\n";
    //GetLog() << "Sh0 = " << Sh0 << "\n\n";
    //GetLog() << "Shg = " << Shg << "\n\n";
    //GetLog() << "alpha_y = " << alpha_y << "\n\n";
    //GetLog() << "X1 = " << X1 << "\n\n";
    //GetLog() << "E = " << E << "\n\n";
    //GetLog() << "Fy0 = " << Fy0 << "\n\n";
    return Fy0;
}

// Oeverturning Couple
double ChMFTyre62::CalcMx(double Fy, double Fz, double gamma) {
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double dPi = 0.0;
    if (m_PacCoeff.InflPres != m_PacCoeff.NomPres) {
        double dPi = (m_PacCoeff.InflPres - m_PacCoeff.NomPres) / m_PacCoeff.NomPres;
    }
    double Mx = m_PacCoeff.R0 * Fz * m_PacScal.lmx *
                (m_PacCoeff.qsx1 * m_PacScal.lvmx - m_PacCoeff.qsx2 * gamma * (1.0 + m_PacCoeff.ppmx1 * dPi) -
                 m_PacCoeff.qsx12 * gamma * std::abs(gamma) + m_PacCoeff.qsx3 * Fy / Fz0s +
                 m_PacCoeff.qsx4 * cos(m_PacCoeff.qsx5 * atan(pow(m_PacCoeff.qsx6 * Fz / Fz0s, 2))) *
                     sin(m_PacCoeff.qsx7 * gamma + m_PacCoeff.qsx8 * atan(m_PacCoeff.qsx9 * Fy / Fz0s)) +
                 m_PacCoeff.qsx10 * atan(m_PacCoeff.qsx11 * Fz / Fz0s) * gamma) +
                 m_PacCoeff.R0 * Fy * m_PacScal.lmx * (m_PacCoeff.qsx13 + m_PacCoeff.qsx14 * std::abs(gamma));
    return Mx;
}

// Rolling Resistance
double ChMFTyre62::CalcMy(double Fx, double Fz, double gamma) {
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double v0 = sqrt(9.81 * m_PacCoeff.R0);
    double vstar = std::abs(m_states.vx / v0);
    double My = ChSineStep(std::abs(m_states.vx), 0.5, 0, 1.0, 1.0) * ChSignum(m_states.vx) * Fz * m_PacCoeff.R0 *
                (m_PacCoeff.qsy1 + m_PacCoeff.qsy2 * Fx / Fz0s + m_PacCoeff.qsy3 * vstar +
                 m_PacCoeff.qsy4 * pow(vstar, 4) +
                 (m_PacCoeff.qsy5 + m_PacCoeff.qsy6 * Fz / Fz0s) * pow(gamma, 2)) *
                pow(Fz / Fz0s, m_PacCoeff.qsy7) * m_PacScal.lmuy;
    return My;
}

/*
double ChMFTyre62::CalcTrail(double alpha, double Fz, double gamma) {
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double dFz = (Fz - Fz0s) / Fz0s;
    double C = m_PacCoeff.qcz1;
    double Sh = m_PacCoeff.qhz1 + m_PacCoeff.qhz2 * dFz + (m_PacCoeff.qhz3 + m_PacCoeff.qhz4 * dFz) * gamma_z;
    double alpha_t = alpha + Sh;
    double B = (m_PacCoeff.qbz1 + m_PacCoeff.qbz2 * dFz + m_PacCoeff.qbz3 * pow(dFz, 2)) *
               (1.0 + m_PacCoeff.qbz4 * gamma_z + m_PacCoeff.qbz5 * std::abs(gamma_z)) * m_PacScal.lky / m_PacScal.lmuy;
    double D = Fz * (m_PacCoeff.qdz1 + m_PacCoeff.qdz2 * dFz) *
               (1.0 + m_PacCoeff.qdz3 * gamma_z + m_PacCoeff.qdz4 * pow(gamma_z, 2)) * m_PacCoeff.R0 / Fz0s *
               m_PacScal.ltr;
    double E = (m_PacCoeff.qez1 + m_PacCoeff.qez2 * dFz + m_PacCoeff.qez3 * pow(dFz, 2)) *
               (1.0 + (m_PacCoeff.qez4 + m_PacCoeff.qez5 * gamma_z) * atan(B * C * alpha_t) / CH_C_PI_2);
    double X1 = B * alpha_t;
    return D * cos(C * atan(B * X1 - E * (B * X1 - atan(B * X1)))) * cos(alpha);
}
*/

/*
double ChMFTyre62::CalcMres(double alpha, double Fz, double gamma) {
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double dFz = (Fz - Fz0s) / Fz0s;
    double alpha_r = alpha + m_Shf;
    double gamma_z = gamma * m_PacScal.lgaz;
    double C = 1.0;
    double B = (m_PacCoeff.qbz9 * m_PacScal.lky / m_PacScal.lmuy + m_PacCoeff.qbz10 * m_By * m_Cy);
    double D = Fz *
               ((m_PacCoeff.qdz6 + m_PacCoeff.qdz7 * dFz) * m_PacScal.ltr +
                (m_PacCoeff.qdz8 + m_PacCoeff.qdz9 * dFz) * gamma_z) *
               m_PacCoeff.R0 * m_PacScal.lmuy;
    return D * cos(C * atan(B * alpha_r)) * cos(alpha);
}
*/

double ChMFTyre62::CalcMz(double alpha, double Fz, double gamma, double* Cy, double* Kya, double* By, double* Svy, double* Shy, double* Gyk, double* Fyp) {
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double dFz = (Fz - Fz0s) / Fz0s;
    double dPi = 0.0;
    if (m_PacCoeff.InflPres != m_PacCoeff.NomPres) {
        double dPi = (m_PacCoeff.InflPres - m_PacCoeff.NomPres) / m_PacCoeff.NomPres;
    }
    double Sht = m_PacCoeff.qhz1 + m_PacCoeff.qhz2 * dFz + (m_PacCoeff.qhz3 + m_PacCoeff.qhz4 * dFz) * gamma;
    double alpha_t = alpha + Sht;
    double alpha_r = alpha + *Shy + *Svy / *Kya;
    // Pneumatic trail
    double Ct = m_PacCoeff.qcz1;
    double Bt = (m_PacCoeff.qbz1 + m_PacCoeff.qbz2 * dFz + m_PacCoeff.qbz3 * pow(dFz, 2)) *
               (1.0 + m_PacCoeff.qbz4 + m_PacCoeff.qbz5 * std::abs(gamma)) * m_PacScal.lky / m_PacScal.lmuy;
    double Dt = (m_PacCoeff.qdz1 + m_PacCoeff.qdz2 * dFz) * (1.0 - m_PacCoeff.ppz1 * dPi) *
               (1.0 + m_PacCoeff.qdz3 * gamma + m_PacCoeff.qdz4 * pow(gamma, 2)) * Fz * m_PacCoeff.R0 / Fz0s *
               m_PacScal.ltr;
    double Et = (m_PacCoeff.qez1 + m_PacCoeff.qez2 * dFz + m_PacCoeff.qez3 * pow(dFz, 2)) *
               (1.0 + (m_PacCoeff.qez4 + m_PacCoeff.qez5 * gamma) * atan(Bt * Ct * alpha_t) / CH_C_PI_2);
    double X1 = Bt * alpha_t;
    double t = Dt * cos(Ct * atan(X1 - Et * (X1 - atan(X1)))) * cos(alpha);
    // Residual moment
    double Br = (m_PacCoeff.qbz9 * m_PacScal.lky / m_PacScal.lmuy + m_PacCoeff.qbz10 * *By * *Cy);
    double Dr = ((m_PacCoeff.qdz6 + m_PacCoeff.qdz7 * dFz) * m_PacScal.ltr +
                (m_PacCoeff.qdz8 + m_PacCoeff.qdz9 * dFz) * (1.0 - m_PacCoeff.ppz2 * dPi) * gamma * m_PacScal.lkzc +
                (m_PacCoeff.qdz10 + m_PacCoeff.qdz11 * dFz) * gamma * std::abs(gamma) * m_PacScal.lkzc) *
                Fz * m_PacCoeff.R0 * m_PacScal.lmuy;
    double Mzr = Dr * cos(atan(Br * alpha_r)) * cos(alpha);
    double Mz = -t * *Fyp * *Gyk + Mzr;
    return Mz;
}

double ChMFTyre62::CalcFxComb(double kappa, double alpha, double Fz, double gamma, double* Kxk) {
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double dFz = (Fz - Fz0s) / Fz0s;
    double dPi = 0.0;
    if (m_PacCoeff.InflPres != m_PacCoeff.NomPres) {
        double dPi = (m_PacCoeff.InflPres - m_PacCoeff.NomPres) / m_PacCoeff.NomPres;
    }
    double C = m_PacCoeff.pcx1 * m_PacScal.lcx;
    double Mu = (m_PacCoeff.pdx1 + m_PacCoeff.pdx2 * dFz) * (1.0 - m_PacCoeff.pdx3 * pow(gamma, 2)) * (1.0 + m_PacCoeff.ppx3 * dPi + m_PacCoeff.ppx4 * pow(dPi, 2)) * m_PacScal.lmux;
    double D = Mu * Fz * m_mu / m_PacCoeff.mu0;
    double Sh = (m_PacCoeff.phx1 + m_PacCoeff.phx2 * dFz) * m_PacScal.lhx;
    double kappa_x = kappa + Sh;
    double E = (m_PacCoeff.pex1 + m_PacCoeff.pex2 * dFz + m_PacCoeff.pex3 * pow(dFz, 2)) * (1.0 - m_PacCoeff.pex4 * ChSignum(kappa_x)) * m_PacScal.lex;
    double K = (m_PacCoeff.pkx1 + m_PacCoeff.pkx2 * dFz) * exp(m_PacCoeff.pkx3 * dFz) * (1.0 + m_PacCoeff.ppx1 * dPi + m_PacCoeff.ppx2 * pow(dPi, 2)) * Fz * m_PacScal.lkx;  // BCD = Kx
    double B = K / (C * D);
    double Sv = (m_PacCoeff.pvx1 + m_PacCoeff.pvx2 * dFz) * Fz * m_PacScal.lvx * m_PacScal.lmux;
    double X1 = B * kappa_x;
    // combined slip
    double Shxa = m_PacCoeff.rhx1;
    double alpha_s = alpha + Shxa;
    double Bxa = (m_PacCoeff.rbx1 + m_PacCoeff.rbx3 * pow(gamma, 2)) * cos(atan(m_PacCoeff.rbx2 * kappa)) * m_PacScal.lxal;
    double Cxa = m_PacCoeff.rcx1;
    double Exa = m_PacCoeff.rex1 + m_PacCoeff.rex2 * dFz;
    double Gxa = cos(Cxa * atan(Bxa * alpha_s - Exa * (Bxa * alpha_s - atan(Bxa * alpha_s)))) /
                 cos(Cxa * atan(Bxa * Shxa - Exa * (Bxa * Shxa - atan(Bxa * Shxa))));
    double Fx0 = D * sin(C * atan(X1 - E * (X1 - atan(X1)))) + Sv;
    *Kxk = K;
    return Fx0 * Gxa;
}

double ChMFTyre62::CalcFyComb(double kappa, double alpha, double Fz, double gamma) {
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double dFz = (Fz - Fz0s) / Fz0s;
    double dPi = 0.0;
    if (m_PacCoeff.InflPres != m_PacCoeff.NomPres) {
        double dPi = (m_PacCoeff.InflPres - m_PacCoeff.NomPres) / m_PacCoeff.NomPres;
    }
    double C = m_PacCoeff.pcy1 * m_PacScal.lcy;
    double Mu = (m_PacCoeff.pdy1 + m_PacCoeff.pdy2 * dFz) * (1.0 - m_PacCoeff.pdy3 * pow(gamma, 2)) * (1.0 + m_PacCoeff.ppy3 * dPi + m_PacCoeff.ppy4 * pow(dPi, 2)) * m_PacScal.lmuy;
    double D = Mu * Fz * m_mu / m_PacCoeff.mu0;
    double Ka = m_PacCoeff.pky1 * Fz0s * (1.0 + m_PacCoeff.ppy1 * dPi) * sin(m_PacCoeff.pky4 * atan(Fz / ((m_PacCoeff.pky2 + m_PacCoeff.pky5 * pow(gamma, 2)) * (1.0 + m_PacCoeff.ppy2 * dPi) * Fz0s))) * (1.0 - m_PacCoeff.pky3 * std::abs(gamma)) * m_PacScal.lky;
    double Kg = (m_PacCoeff.pky6 + m_PacCoeff.pky7 * dFz) * (1.0 + m_PacCoeff.ppy5 * dPi) * Fz * m_PacScal.lkyc;
    double B = Ka / (C * D);
    double Sv0 = Fz * (m_PacCoeff.pvy1 + m_PacCoeff.pvy2 * dFz) * m_PacScal.lvy * m_PacScal.lmuy;
    double Svg = Fz * (m_PacCoeff.pvy3 + m_PacCoeff.pvy4 * dFz) * gamma * m_PacScal.lkyc * m_PacScal.lmuy;
    double Sv = Sv0 + Svg;
    double Sh0 = (m_PacCoeff.phy1 + m_PacCoeff.phy2 * dFz) * m_PacScal.lhy;
    double Shg = (Kg * gamma - Svg) / Ka;
    double Sh = Sh0 + Shg;
    double alpha_y = alpha + Sh;
    double X1 = ChClamp(B * alpha_y, -CH_C_PI_2 + 0.001, CH_C_PI_2 - 0.001);  // Ensure that X1 stays within +/-90 deg minus a little bit
    double E = (m_PacCoeff.pey1 + m_PacCoeff.pey2 * dFz) *
               (1.0 + m_PacCoeff.pey5 * pow(gamma, 2) - (m_PacCoeff.pey3 + m_PacCoeff.pey4 * gamma) * ChSignum(alpha_y)) * m_PacScal.ley;
    // combined slip
    double Dvyk = Mu * Fz * (m_PacCoeff.rvy1 + m_PacCoeff.rvy2 * dFz + m_PacCoeff.rvy3 * gamma) *
                  cos(atan(m_PacCoeff.rvy4 * alpha));
    double Svyk = Dvyk * sin(m_PacCoeff.rvy5 * atan(m_PacCoeff.rvy6 * kappa)) * m_PacScal.lvyka;
    double Byk = (m_PacCoeff.rby1 + m_PacCoeff.rby4 * pow(gamma, 2)) * cos(atan(m_PacCoeff.rby2 * (alpha - m_PacCoeff.rby3))) * m_PacScal.lyka;
    double Cyk = m_PacCoeff.rcy1;
    double Eyk = m_PacCoeff.rey1 + m_PacCoeff.rey2 * dFz;
    double Shyk = m_PacCoeff.rhy1 + m_PacCoeff.rhy2 * dFz;
    double kappa_s = kappa + Shyk;
    double Gyk = cos(Cyk * atan(Byk * kappa_s - Eyk * (Byk * kappa_s - atan(Byk * kappa_s)))) / cos(Cyk * atan(Byk * Shyk - Eyk * (Byk * Shyk - atan(Byk * Shyk))));
    double Fy0 = D * sin(C * atan(X1 - E * (X1 - atan(X1)))) + Sv;
    return Fy0 * Gyk + Svyk;
}

void ChMFTyre62::CalcFyComb(double kappa, double alpha, double Fz, double gamma, double* Cy, double* Kya, double* By, double* Svy, double* Shy, double* Gyk, double* Fyp) {
    gamma = 0.0;
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double dFz = (Fz - Fz0s) / Fz0s;
    double dPi = 0.0;
    if (m_PacCoeff.InflPres != m_PacCoeff.NomPres) {
        double dPi = (m_PacCoeff.InflPres - m_PacCoeff.NomPres) / m_PacCoeff.NomPres;
    }
    *Cy = m_PacCoeff.pcy1 * m_PacScal.lcy;
    double Mu = (m_PacCoeff.pdy1 + m_PacCoeff.pdy2 * dFz) * (1.0 - m_PacCoeff.pdy3 * pow(gamma, 2)) * (1.0 + m_PacCoeff.ppy3 * dPi + m_PacCoeff.ppy4 * pow(dPi, 2)) * m_PacScal.lmuy;
    double D = Mu * Fz * m_mu / m_PacCoeff.mu0;
    *Kya = m_PacCoeff.pky1 * Fz0s * (1.0 + m_PacCoeff.ppy1 * dPi) * sin(m_PacCoeff.pky4 * atan(Fz / ((m_PacCoeff.pky2 + m_PacCoeff.pky5 * pow(gamma, 2)) * (1.0 + m_PacCoeff.ppy2 * dPi) * Fz0s))) * (1.0 - m_PacCoeff.pky3 * std::abs(gamma)) * m_PacScal.lky;
    double Kg = (m_PacCoeff.pky6 + m_PacCoeff.pky7 * dFz) * (1.0 + m_PacCoeff.ppy5 * dPi) * Fz * m_PacScal.lkyc;
    *By = *Kya / (*Cy * D);
    double Sv0 = Fz * (m_PacCoeff.pvy1 + m_PacCoeff.pvy2 * dFz) * m_PacScal.lvy * m_PacScal.lmuy;
    double Svg = Fz * (m_PacCoeff.pvy3 + m_PacCoeff.pvy4 * dFz) * gamma * m_PacScal.lkyc * m_PacScal.lmuy;
    *Svy = Sv0 + Svg;
    double Sh0 = (m_PacCoeff.phy1 + m_PacCoeff.phy2 * dFz) * m_PacScal.lhy;
    double Shg = (Kg * gamma - Svg) / *Kya;
    *Shy = Sh0 + Shg;
    double alpha_y = alpha + *Shy;
    double X1 = ChClamp(*By * alpha_y, -CH_C_PI_2 + 0.001, CH_C_PI_2 - 0.001);  // Ensure that X1 stays within +/-90 deg minus a little bit
    double E = (m_PacCoeff.pey1 + m_PacCoeff.pey2 * dFz) *
               (1.0 + m_PacCoeff.pey5 * pow(gamma, 2) - (m_PacCoeff.pey3 + m_PacCoeff.pey4 * gamma) * ChSignum(alpha_y)) * m_PacScal.ley;
    // combined slip
    double Dvyk = Mu * Fz * (m_PacCoeff.rvy1 + m_PacCoeff.rvy2 * dFz + m_PacCoeff.rvy3 * gamma) *
                  cos(atan(m_PacCoeff.rvy4 * alpha));
    double Svyk = Dvyk * sin(m_PacCoeff.rvy5 * atan(m_PacCoeff.rvy6 * kappa)) * m_PacScal.lvyka;
    double Byk = (m_PacCoeff.rby1 + m_PacCoeff.rby4 * pow(gamma, 2)) * cos(atan(m_PacCoeff.rby2 * (alpha - m_PacCoeff.rby3))) * m_PacScal.lyka;
    double Cyk = m_PacCoeff.rcy1;
    double Eyk = m_PacCoeff.rey1 + m_PacCoeff.rey2 * dFz;
    double Shyk = m_PacCoeff.rhy1 + m_PacCoeff.rhy2 * dFz;
    double kappa_s = kappa + Shyk;
    *Gyk = cos(Cyk * atan(Byk * kappa_s - Eyk * (Byk * kappa_s - atan(Byk * kappa_s)))) / cos(Cyk * atan(Byk * Shyk - Eyk * (Byk * Shyk - atan(Byk * Shyk))));
    *Fyp = D * sin(*Cy * atan(X1 - E * (X1 - atan(X1)))) + *Svy;
}

double ChMFTyre62::CalcMzComb(double kappa, double alpha, double Fz, double gamma, double Fx, double Fy, double* Cy, double* Kya, double* By, double* Svy, double* Shy, double* Gyk, double* Fyp, double* Kxk) {
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double dFz = (Fz - Fz0s) / Fz0s;
    double dPi = 0.0;
    if (m_PacCoeff.InflPres != m_PacCoeff.NomPres) {
        double dPi = (m_PacCoeff.InflPres - m_PacCoeff.NomPres) / m_PacCoeff.NomPres;
    }
    double Sht = m_PacCoeff.qhz1 + m_PacCoeff.qhz2 * dFz + (m_PacCoeff.qhz3 + m_PacCoeff.qhz4 * dFz) * gamma;
    double alpha_t = alpha + Sht;
    double alpha_r = alpha + *Shy + *Svy / *Kya;
    double alpha_teq = atan(sqrt(pow(tan(alpha_t), 2) + pow(*Kxk, 2) * pow(kappa, 2) / pow(*Kya, 2))) * ChSignum(alpha_t);
    double alpha_req = atan(sqrt(pow(tan(alpha_r), 2) + pow(*Kxk, 2) * pow(kappa, 2) / pow(*Kya, 2))) * ChSignum(alpha_r);
    double s = (m_PacCoeff.ssz1 + m_PacCoeff.ssz2 * Fy / Fz0s + (m_PacCoeff.ssz3 + m_PacCoeff.ssz4 * dFz) * gamma) * m_PacCoeff.R0 * m_PacScal.ls;
    // Pneumatic trail
    double Ct = m_PacCoeff.qcz1;
    double Bt = (m_PacCoeff.qbz1 + m_PacCoeff.qbz2 * dFz + m_PacCoeff.qbz3 * pow(dFz, 2)) *
               (1.0 + m_PacCoeff.qbz4 + m_PacCoeff.qbz5 * std::abs(gamma)) * m_PacScal.lky / m_PacScal.lmuy;
    double Dt = (m_PacCoeff.qdz1 + m_PacCoeff.qdz2 * dFz) * (1.0 - m_PacCoeff.ppz1 * dPi) *
               (1.0 + m_PacCoeff.qdz3 * gamma + m_PacCoeff.qdz4 * pow(gamma, 2)) * Fz * m_PacCoeff.R0 / Fz0s *
               m_PacScal.ltr;
    double Et = (m_PacCoeff.qez1 + m_PacCoeff.qez2 * dFz + m_PacCoeff.qez3 * pow(dFz, 2)) *
               (1.0 + (m_PacCoeff.qez4 + m_PacCoeff.qez5 * gamma) * atan(Bt * Ct * alpha_t) / CH_C_PI_2);
    double X1 = Bt * alpha_teq;
    double t = Dt * cos(Ct * atan(X1 - Et * (X1 - atan(X1)))) * cos(alpha);
    // Residual moment
    double Br = (m_PacCoeff.qbz9 * m_PacScal.lky / m_PacScal.lmuy + m_PacCoeff.qbz10 * *By * *Cy);
    double Dr = ((m_PacCoeff.qdz6 + m_PacCoeff.qdz7 * dFz) * m_PacScal.ltr +
                (m_PacCoeff.qdz8 + m_PacCoeff.qdz9 * dFz) * (1.0 - m_PacCoeff.ppz2 * dPi) * gamma * m_PacScal.lkzc +
                (m_PacCoeff.qdz10 + m_PacCoeff.qdz11 * dFz) * gamma * std::abs(gamma) * m_PacScal.lkzc) *
                Fz * m_PacCoeff.R0 * m_PacScal.lmuy;
    double Mzr = Dr * cos(atan(Br * alpha_req)) * cos(alpha);
    double Mz = -t * *Fyp * *Gyk + Mzr;
    return Mz;
}

}  // end namespace vehicle
}  // namespace chrono
