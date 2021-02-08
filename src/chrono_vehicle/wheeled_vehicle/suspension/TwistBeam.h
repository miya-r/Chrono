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
// Three-link Independent Rear Suspension constructed with data from file.
//
// =============================================================================

#ifndef TWISTBEAM_H
#define TWISTBEAM_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChTwistBeam.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Three-link Independent Rear Suspension constructed with data from file.
class CH_VEHICLE_API TwistBeam : public ChTwistBeam {
  public:
    TwistBeam(const std::string& filename);
    TwistBeam(const rapidjson::Document& d);
    ~TwistBeam();

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getArmMass() const override { return m_armMass; }
    virtual double getBeamMass() const override { return m_beamMass; }

    virtual double getBeamSpringCoefficient() const override { return m_beamSpringcoef; }
    virtual double getBeamDampingCoefficient() const override { return m_beamDampingcoef; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getArmRadius() const override { return m_armRadius; }
    virtual double getBeamRadius() const override { return m_beamRadius; }

    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector<>& getArmInertia() const override { return m_armInertia; }
    virtual const ChVector<>& getBeamInertia() const override { return m_beamInertia; }

    virtual const ChVector<> getBeamCOM() const override { return m_beamCOM; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual ChLinkSpringCB::ForceFunctor* getSpringForceFunctor() const override { return m_springForceCB; }
    virtual ChLinkSpringCB::ForceFunctor* getShockForceFunctor() const override { return m_shockForceCB; }

  private:
    virtual const ChVector<> getLocation(PointId which) override { return m_points[which]; }
    virtual const ChVector<> getDirection(DirectionId which) override { return m_dirs[which]; }
    

    virtual void Create(const rapidjson::Document& d) override;

    ChLinkSpringCB::ForceFunctor* m_springForceCB;
    ChLinkSpringCB::ForceFunctor* m_shockForceCB;

    ChVector<> m_points[NUM_POINTS];
    ChVector<> m_dirs[NUM_DIRS];
    ChVector<> m_beamCOM;

    double m_spindleMass;
    double m_armMass;
    double m_beamMass;
    double m_beamSpringcoef;
    double m_beamDampingcoef;

    double m_spindleRadius;
    double m_spindleWidth;
    double m_armRadius;
    double m_beamRadius;

    ChVector<> m_spindleInertia;
    ChVector<> m_armInertia;
    ChVector<> m_beamInertia;

    double m_axleInertia;

    double m_springRestLength;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
