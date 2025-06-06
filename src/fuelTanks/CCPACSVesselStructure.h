/*
* Copyright (C) 2007-2025 German Aerospace Center (DLR/SC)
*
* Created: 2025-05-09 Marko Alder <Marko.Alder@dlr.de>
* 
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
* @brief  Implementation of CPACS vessels handling routines.
*/

#pragma once

#include "generated/CPACSVesselStructure.h"
#include "CTiglRelativelyPositionedComponent.h"
#include "ITiglFuselageDuctStructure.h"

namespace tigl
{

class CCPACSVesselStructure : public generated::CPACSVesselStructure, public ITiglFuselageDuctStructure
{
public:
    TIGL_EXPORT CCPACSVesselStructure(CCPACSVessel* parent, CTiglUIDManager* uidMgr);
};

} //namespace tigl
