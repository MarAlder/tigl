/* 
* Copyright (C) 2007-2013 German Aerospace Center (DLR/SC)
*
* Created: 2010-08-13 Markus Litz <Markus.Litz@dlr.de>
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
* @brief  Implementation of CPACS fuselage sections handling routines.
*/

#include "CCPACSFuselageSections.h"
#include "CCPACSFuselageSection.h"

#include "CTiglError.h"

namespace tigl
{
CCPACSFuselageSections::CCPACSFuselageSections(CCPACSFuselage* parent, CTiglUIDManager* uidMgr)
    : generated::CPACSFuselageSections(parent, uidMgr) {}

CCPACSFuselageSections::CCPACSFuselageSections(CCPACSDuct* parent, CTiglUIDManager* uidMgr)
    : generated::CPACSFuselageSections(parent, uidMgr) {}

CCPACSFuselageSections::CCPACSFuselageSections(CCPACSVessel* parent, CTiglUIDManager* uidMgr)
    : generated::CPACSFuselageSections(parent, uidMgr) {}

int CCPACSFuselageSections::GetSectionCount() const
{
    return static_cast<int>(m_sections.size());
}

CCPACSFuselageSection& CCPACSFuselageSections::GetSection(int index) const
{
    index--;
    if (index < 0 || index >= GetSectionCount()) {
        throw CTiglError("Invalid index in CCPACSFuselageSections::GetSection", TIGL_INDEX_ERROR);
    }
    return *m_sections[index];
}

// Gets a section by uid.
CCPACSFuselageSection& CCPACSFuselageSections::GetSection(const std::string& sectionUID)
{
    for (std::size_t i = 0; i < m_sections.size(); i++) {
        if (m_sections[i]->GetUID() == sectionUID) {
            return *m_sections[i];
        }
    }
    throw CTiglError("Invalid uid in CCPACSWingSections::GetSection", TIGL_UID_ERROR);
}


} // end namespace tigl
