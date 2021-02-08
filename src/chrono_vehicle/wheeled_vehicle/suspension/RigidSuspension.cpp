#include <cstdio>

#include "chrono_vehicle/wheeled_vehicle/suspension/RigidSuspension.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {
RigidSuspension::RigidSuspension(const std::string& filename)
    : ChRigidSuspension("") {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}


RigidSuspension::RigidSuspension(const rapidjson::Document& d)
    : ChRigidSuspension("") {
    Create(d);
}

// -----------------------------------------------------------------------------
// Worker function for creating a Rigid suspension using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void RigidSuspension::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read Spindle data
    assert(d.HasMember("Spindle"));
    assert(d["Spindle"].IsObject());

    m_spindleMass = d["Spindle"]["Mass"].GetDouble(); //OK
    m_points[SPINDLE] = ReadVectorJSON(d["Spindle"]["COM"]); //??
    m_spindleInertia = ReadVectorJSON(d["Spindle"]["Inertia"]); //OK
    m_spindleRadius = d["Spindle"]["Radius"].GetDouble();  //OK
    m_spindleWidth = d["Spindle"]["Width"].GetDouble();  //OK

    // Read axle inertia
    assert(d.HasMember("Axle"));
    assert(d["Axle"].IsObject());

    m_axleInertia = d["Axle"]["Inertia"].GetDouble();
}



}  // end namespace vehicle
}  // end namespace chrono
