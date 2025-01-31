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
// Global functions for accessing the Chrono::Vehicle model data.
//
// =============================================================================

#ifndef CH_VEHICLE_MODELDATA_H
#define CH_VEHICLE_MODELDATA_H

#include <string>

#include "chrono_vehicle/ChApiVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Set the path to the Chrono::Vehicle data directory (ATTENTION: not thread safe).
CH_VEHICLE_API void SetDataPath(const std::string& path);

/// Get the current path to the Chrono::Vehicle data directory (thread safe).
CH_VEHICLE_API const std::string& GetDataPath();

/// Get the complete path to the specified filename (thread safe).
/// The filename is assumed to be given relative to the Chrono::Vehicle model
/// data directory.
CH_VEHICLE_API std::string GetDataFile(const std::string& filename);

//current dir
CH_VEHICLE_API void SetCurrentDirPath(const std::string& path);

CH_VEHICLE_API const std::string& GetCurrentDirPath();

CH_VEHICLE_API std::string GetCurrentDirDataFile(const std::string& filename);

//input dir
CH_VEHICLE_API void SetinputDirPath(const std::string& input_dir_name);

CH_VEHICLE_API const std::string& GetinputDirPath();

CH_VEHICLE_API std::string GetinputDirDataFile(const std::string& filename);

//output dir
CH_VEHICLE_API void SetoutputDirPath(const std::string& output_dir_name);

CH_VEHICLE_API const std::string& GetoutputDirPath();

CH_VEHICLE_API std::string GetoutputDirDataFile(const std::string& filename);
/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
