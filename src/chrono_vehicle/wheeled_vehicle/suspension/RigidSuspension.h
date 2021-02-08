#ifndef RIGIDSUSPENSION_H
#define RIGIDSUSPENSION_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChRigidSuspension.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

class CH_VEHICLE_API RigidSuspension : public ChRigidSuspension { 
  public:
    RigidSuspension(const std::string& filename);
    RigidSuspension(const rapidjson::Document& d);
    ~RigidSuspension() {}

    virtual double getSpindleMass() const override { return m_spindleMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }

    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }


  private:
    virtual const ChVector<> getLocation(PointId which) override { return m_points[which]; }

    virtual void Create(const rapidjson::Document& d) override;

    ChVector<> m_points[NUM_POINTS];
    double m_spindleMass;

    double m_spindleRadius;
    double m_spindleWidth;

    ChVector<> m_spindleInertia;

    double m_axleInertia;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif