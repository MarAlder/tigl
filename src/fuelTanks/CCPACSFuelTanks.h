/*
* Copyright (C) 2007-2022 German Aerospace Center (DLR/SC)
*
* Created: 2022-04-06 Anton Reiswich <Anton.Reiswich@dlr.de>
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
/**
* @file
* @brief  Implementation of CPACS fuelTanks handling routines.
*/

#pragma once

#include "generated/CPACSFuelTanks.h"

namespace tigl
{

class CCPACSFuelTanks : public generated::CPACSFuelTanks
{
public:
    TIGL_EXPORT CCPACSFuelTanks(CCPACSAircraftModel* parent, CTiglUIDManager* uidMgr);

    // Get a specific hull for a given uID
    TIGL_EXPORT CCPACSFuelTank const& GetFuelTank(std::string const& uID) const;
    TIGL_EXPORT CCPACSFuelTank& GetFuelTank(std::string const& uID);

    // Get a specific hull for a given index.
    TIGL_EXPORT CCPACSFuelTank& GetFuelTank(int index) const;

    // Returns the hull index for a given uID.
    TIGL_EXPORT int GetFuelTankIndex(const std::string& uID) const;

    // Returns the total count of fuselages in a configuration
    TIGL_EXPORT int GetFuelTanksCount() const;
};

} //namespace tigl