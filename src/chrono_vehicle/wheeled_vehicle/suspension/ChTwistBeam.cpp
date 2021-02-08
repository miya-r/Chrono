// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for a 3-link independent rear suspension (non-steerable).
// This suspension has a trailing arm and 2 additional links connecting the
// arm to the chassis.
//
// The suspension subsystem is modeled with respect to a right-handed frame,
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The suspension reference frame is assumed to be always aligned with that of
// the vehicle.  When attached to a chassis, only an offset is provided.
//
// All point locations are assumed to be given for the left half of the
// suspension and will be mirrored (reflecting the y coordinates) to construct
// the right side.
//
// =============================================================================

#include <algorithm>

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/assets/ChColorAsset.h"

#include "chrono_vehicle/wheeled_vehicle/suspension/ChTwistBeam.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChTwistBeam::m_pointNames[] = {"SPINDLE ", "TA_CM",    "TA_S",
                                                    "SHOCK_C ", "SHOCK_A ", "SPRING_C", "SPRING_A"};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTwistBeam::ChTwistBeam(const std::string& name) : ChSuspension(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTwistBeam::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                const ChVector<>& location,
                                std::shared_ptr<ChBody> tierod_body,
                                int steering_index,
                                double left_ang_vel,
                                double right_ang_vel) {
    m_location = location;

    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> suspension_to_abs(location);
    suspension_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

    // Transform all hardpoints and directions to absolute frame.
    m_pointsL.resize(NUM_POINTS);
    m_pointsR.resize(NUM_POINTS);

    m_dirsL.resize(NUM_DIRS);
    m_dirsR.resize(NUM_DIRS);

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
        m_pointsL[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
        rel_pos.y() = -rel_pos.y();
        m_pointsR[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
    }

    for (int i = 0; i < NUM_DIRS; i++) {
        ChVector<> rel_dir = getDirection(static_cast<DirectionId>(i));
        m_dirsL[i] = suspension_to_abs.TransformDirectionLocalToParent(rel_dir);
        rel_dir.y() = -rel_dir.y();
        m_dirsR[i] = suspension_to_abs.TransformDirectionLocalToParent(rel_dir);
    }
    m_beamCOM_Loc = suspension_to_abs.TransformLocalToParent(getBeamCOM());


    // Initialize left and right sides.
    InitializeSide(LEFT, chassis, m_pointsL, m_dirsL, left_ang_vel);
    InitializeSide(RIGHT, chassis, m_pointsR, m_dirsR, right_ang_vel);

    // Create and initialize the revolute joint between left and right arms.
    ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();
    ChCoordsys<> rev_csys(m_beamCOM_Loc, chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_revoluteBeam = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revoluteBeam->SetNameString(m_name + "_revoluteBeam");
    m_revoluteBeam->Initialize(m_beam[0], m_beam[1], rev_csys);
    chassis->GetSystem()->AddLink(m_revoluteBeam);

    //GetLog() << getBeamSpringCoefficient() << "\n" << getBeamDampingCoefficient() << "\n";
    //m_revoluteBeam->GetForce_Rz().SetActive(true);
    //m_revoluteBeam->GetForce_Rz().SetK(getBeamSpringCoefficient());
    //m_revoluteBeam->GetForce_Rz().SetR(getBeamDampingCoefficient());


}

void ChTwistBeam::InitializeSide(VehicleSide side,
                                    std::shared_ptr<ChBodyAuxRef> chassis,
                                    const std::vector<ChVector<> >& points,
                                    const std::vector<ChVector<>>& dirs,
                                    double ang_vel) {
    std::string suffix = (side == LEFT) ? "_L" : "_R";

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();

    // Create and initialize spindle body (same orientation as the chassis)
    m_spindle[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_spindle[side]->SetNameString(m_name + "_spindle" + suffix);
    m_spindle[side]->SetPos(points[SPINDLE]);
    m_spindle[side]->SetRot(chassisRot);
    m_spindle[side]->SetWvel_loc(ChVector<>(0, ang_vel, 0));
    m_spindle[side]->SetMass(getSpindleMass());
    m_spindle[side]->SetInertiaXX(getSpindleInertia());
    chassis->GetSystem()->AddBody(m_spindle[side]);

    // Unit vectors for orientation matrices.
    ChVector<> u;
    ChVector<> v;
    ChVector<> w;
    ChMatrix33<> rot;

    // Create and initialize the trailing arm and the two link bodies.
    u = points[TA_C] - points[TA_S];
    u.Normalize();
    v = Vcross(ChVector<>(0, 0, 1), u);
    v.Normalize();
    w = Vcross(u, v);
    rot.Set_A_axis(u, v, w);

    m_arm[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_arm[side]->SetNameString(m_name + "_arm" + suffix);
    m_arm[side]->SetPos(points[TA_CM]);
    m_arm[side]->SetRot(rot);
    m_arm[side]->SetMass(getArmMass());
    m_arm[side]->SetInertiaXX(getArmInertia());
    chassis->GetSystem()->AddBody(m_arm[side]);

    // Create and initialize twistbeam
    m_beam[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_beam[side]->SetNameString(m_name + "_beam" + suffix);
    m_beam[side]->SetPos( points[TB_CM] );
    m_beam[side]->SetRot(chassisRot);
    m_beam[side]->SetMass(getBeamMass() / 2.0);
    m_beam[side]->SetInertiaXX(getBeamInertia());
    chassis->GetSystem()->AddBody(m_beam[side]);


    // Create and initialize the revolute joint between arm and spindle.
    ChCoordsys<> rev_csys(points[SPINDLE], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_arm[side], rev_csys);
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the spherical joint between chassis and arm.
//    //m_sphericalArm
//   ChCoordsys<> rev_ca_sys(points[SPINDLE], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
//    m_revArm[side] = chrono_types::make_shared<ChLinkLockRevolute>();
//    m_revArm[side]->SetNameString(m_name + "_revoluteArm" + suffix);
//    m_revArm[side]->Initialize(chassis, m_arm[side], rev_ca_sys);
//    chassis->GetSystem()->AddLink(m_revArm[side]);
    // Create and initialize the spherical joint between chassis and arm.
    m_sphericalArm[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_sphericalArm[side]->SetNameString(m_name + "_sphericalArm" + suffix);
    m_sphericalArm[side]->Initialize(chassis, m_arm[side], ChCoordsys<>(points[TA_C], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalArm[side]);
    //create LinkLockLock arm-beam
    ChCoordsys<> rev_ab_sys(points[TB_A], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_lockBeam[side] = chrono_types::make_shared<ChLinkLockLock>();
    m_lockBeam[side]->SetNameString(m_name + "_lockArm" + suffix);
    m_lockBeam[side]->Initialize(m_arm[side], m_beam[side], rev_ab_sys);
    chassis->GetSystem()->AddLink(m_lockBeam[side]);

    // Create and initialize the spring/damper.
    m_shock[side] = chrono_types::make_shared<ChLinkSpringCB>();
    m_shock[side]->SetNameString(m_name + "_shock" + suffix);
    m_shock[side]->Initialize(chassis, m_arm[side], false, points[SHOCK_C], points[SHOCK_A]);
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

    m_spring[side] = chrono_types::make_shared<ChLinkSpringCB>();
    m_spring[side]->SetNameString(m_name + "_spring" + suffix);
    m_spring[side]->Initialize(chassis, m_arm[side], false, points[SPRING_C], points[SPRING_A], false,
                               getSpringRestLength());
    m_spring[side]->RegisterForceFunctor(getSpringForceFunctor());
    chassis->GetSystem()->AddLink(m_spring[side]);

    // Create and initialize the axle shaft and its connection to the spindle. Note that the
    // spindle rotates about the Y axis.
    m_axle[side] = chrono_types::make_shared<ChShaft>();
    m_axle[side]->SetNameString(m_name + "_axle" + suffix);
    m_axle[side]->SetInertia(getAxleInertia());
    m_axle[side]->SetPos_dt(-ang_vel);
    chassis->GetSystem()->Add(m_axle[side]);

    m_axle_to_spindle[side] = chrono_types::make_shared<ChShaftsBody>();
    m_axle_to_spindle[side]->SetNameString(m_name + "_axle_to_spindle" + suffix);
    m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector<>(0, -1, 0));
    chassis->GetSystem()->Add(m_axle_to_spindle[side]);
}

// -----------------------------------------------------------------------------
// Get the total mass of the suspension subsystem.
// -----------------------------------------------------------------------------
double ChTwistBeam::GetMass() const {
    return 2 * (getSpindleMass() + getArmMass() );
}

// -----------------------------------------------------------------------------
// Get the current COM location of the suspension subsystem.
// -----------------------------------------------------------------------------
ChVector<> ChTwistBeam::GetCOMPos() const {
    ChVector<> com(0, 0, 0);

    com += getSpindleMass() * m_spindle[LEFT]->GetPos();
    com += getSpindleMass() * m_spindle[RIGHT]->GetPos();

    com += getArmMass() * m_arm[LEFT]->GetPos();
    com += getArmMass() * m_arm[RIGHT]->GetPos();

    return com / GetMass();
}

// -----------------------------------------------------------------------------
// Get the wheel track using the spindle local position.
// -----------------------------------------------------------------------------
double ChTwistBeam::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTwistBeam::LogHardpointLocations(const ChVector<>& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> pos = ref + unit * getLocation(static_cast<PointId>(i));

        GetLog() << "   " << m_pointNames[i].c_str() << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTwistBeam::LogConstraintViolations(VehicleSide side) {
    {
        ChVectorDynamic<> C = m_sphericalArm[side]->GetC();
        GetLog() << "Arm spherical         ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "\n";
    }

    {
        ChVectorDynamic<> C = m_revolute[side]->GetC();
        GetLog() << "Spindle revolute      ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "  ";
        GetLog() << "  " << C(4) << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTwistBeam::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    // Add visualization for trailing arms
    AddVisualizationArm(m_arm[LEFT], m_pointsL[TA_C], m_pointsL[TA_S], m_pointsL[TB_A], getArmRadius());
    AddVisualizationArm(m_arm[RIGHT], m_pointsR[TA_C], m_pointsR[TA_S], m_pointsR[TB_A], getArmRadius());
    AddVisualizationArm(m_beam[LEFT], m_pointsL[TB_A], m_beamCOM_Loc, m_pointsL[TB_CM], getBeamRadius());
    AddVisualizationArm(m_beam[RIGHT], m_pointsR[TB_A], m_beamCOM_Loc, m_pointsR[TB_CM], getBeamRadius());

    // Add visualization for the springs and shocks
    m_spring[LEFT]->AddAsset(chrono_types::make_shared<ChPointPointSpring>(0.06, 150, 15));
    m_spring[RIGHT]->AddAsset(chrono_types::make_shared<ChPointPointSpring>(0.06, 150, 15));

    m_shock[LEFT]->AddAsset(chrono_types::make_shared<ChPointPointSegment>());
    m_shock[RIGHT]->AddAsset(chrono_types::make_shared<ChPointPointSegment>());
}

void ChTwistBeam::RemoveVisualizationAssets() {
    ChSuspension::RemoveVisualizationAssets();

    m_arm[LEFT]->GetAssets().clear();
    m_arm[RIGHT]->GetAssets().clear();

    m_beam[LEFT]->GetAssets().clear();
    m_beam[RIGHT]->GetAssets().clear();

    m_spring[LEFT]->GetAssets().clear();
    m_spring[RIGHT]->GetAssets().clear();

    m_shock[LEFT]->GetAssets().clear();
    m_shock[RIGHT]->GetAssets().clear();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTwistBeam::AddVisualizationArm(std::shared_ptr<ChBody> body,
    const ChVector<>& pt_C,
    const ChVector<>& pt_S,
    const ChVector<>& pt_CM,
    double radius)
{
    static const double threshold2 = 1e-6;

    // Express hardpoint locations in body frame.
    ChVector<> p_C = body->TransformPointParentToLocal(pt_C);
    ChVector<> p_S = body->TransformPointParentToLocal(pt_S);
    ChVector<> p_CM = body->TransformPointParentToLocal(pt_CM);
    GetLog() << p_C << "\n" << p_S << "\n" << p_CM << "\n";
    auto cyl_1 = chrono_types::make_shared<ChCylinderShape>();
    cyl_1->GetCylinderGeometry().p1 = p_C;
    cyl_1->GetCylinderGeometry().p2 = p_CM;
    cyl_1->GetCylinderGeometry().rad = radius;
    body->AddAsset(cyl_1);

    auto cyl_2 = chrono_types::make_shared<ChCylinderShape>();
    cyl_2->GetCylinderGeometry().p1 = p_S;
    cyl_2->GetCylinderGeometry().p2 = p_CM;
    cyl_2->GetCylinderGeometry().rad = radius;
    body->AddAsset(cyl_2);


    auto col = chrono_types::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.2f, 0.8f, 0.2f));
    body->AddAsset(col);
}

void ChTwistBeam::AddVisualizationLink(std::shared_ptr<ChBody> body,
                                          const ChVector<>& pt_1,
                                          const ChVector<>& pt_2,
                                          const ChVector<>& pt_CM,
                                          double radius) {
    // Express hardpoint locations in body frame.
    ChVector<> p_1 = body->TransformPointParentToLocal(pt_1);
    ChVector<> p_2 = body->TransformPointParentToLocal(pt_2);
    ChVector<> p_CM = body->TransformPointParentToLocal(pt_CM);

    auto cyl_1 = chrono_types::make_shared<ChCylinderShape>();
    cyl_1->GetCylinderGeometry().p1 = p_1;
    cyl_1->GetCylinderGeometry().p2 = p_CM;
    cyl_1->GetCylinderGeometry().rad = radius;
    body->AddAsset(cyl_1);

    auto cyl_2 = chrono_types::make_shared<ChCylinderShape>();
    cyl_2->GetCylinderGeometry().p1 = p_2;
    cyl_2->GetCylinderGeometry().p2 = p_CM;
    cyl_2->GetCylinderGeometry().rad = radius;
    body->AddAsset(cyl_2);

    auto col = chrono_types::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.8f, 0.2f, 0.2f));
    body->AddAsset(col);
}
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTwistBeam::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_arm[0]);
    bodies.push_back(m_arm[1]);
    bodies.push_back(m_beam[0]);
    bodies.push_back(m_beam[1]);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    ChPart::ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_sphericalArm[0]);
    joints.push_back(m_sphericalArm[1]);
    joints.push_back(m_lockBeam[0]);
    joints.push_back(m_lockBeam[1]);
    joints.push_back(m_revoluteBeam);
    ChPart::ExportJointList(jsonDocument, joints);

    std::vector<std::shared_ptr<ChLinkSpringCB>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    ChPart::ExportLinSpringList(jsonDocument, springs);
}

void ChTwistBeam::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_arm[0]);
    bodies.push_back(m_arm[1]);
    bodies.push_back(m_beam[0]);
    bodies.push_back(m_beam[1]);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    database.WriteShafts(shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_sphericalArm[0]);
    joints.push_back(m_sphericalArm[1]);
    joints.push_back(m_lockBeam[0]);
    joints.push_back(m_lockBeam[1]);
    joints.push_back(m_revoluteBeam);
    database.WriteJoints(joints);

    std::vector<std::shared_ptr<ChLinkSpringCB>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    database.WriteLinSprings(springs);
}

}  // end namespace vehicle
}  // end namespace chrono
