// Copyright (c) 2020 RISC Software GmbH
//
// This file was generated by CPACSGen from CPACS XML Schema (c) German Aerospace Center (DLR/SC).
// Do not edit, all changes are lost when files are re-generated.
//
// Licensed under the Apache License, Version 2.0 (the "License")
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cassert>
#include <CCPACSFuselageSection.h>
#include "CCPACSDuct.h"
#include "CCPACSFuselage.h"
#include "CCPACSHull.h"
#include "CPACSFuselageSections.h"
#include "CTiglError.h"
#include "CTiglLogging.h"
#include "CTiglUIDManager.h"
#include "CTiglUIDObject.h"
#include "TixiHelper.h"

namespace tigl
{
namespace generated
{
    CPACSFuselageSections::CPACSFuselageSections(CCPACSDuct* parent, CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
    {
        //assert(parent != NULL);
        m_parent = parent;
        m_parentType = &typeid(CCPACSDuct);
    }

    CPACSFuselageSections::CPACSFuselageSections(CCPACSFuselage* parent, CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
    {
        //assert(parent != NULL);
        m_parent = parent;
        m_parentType = &typeid(CCPACSFuselage);
    }

    CPACSFuselageSections::CPACSFuselageSections(CCPACSHull* parent, CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
    {
        //assert(parent != NULL);
        m_parent = parent;
        m_parentType = &typeid(CCPACSHull);
    }

    CPACSFuselageSections::~CPACSFuselageSections()
    {
    }

    const CTiglUIDObject* CPACSFuselageSections::GetNextUIDParent() const
    {
        if (m_parent) {
            if (IsParent<CCPACSDuct>()) {
                return GetParent<CCPACSDuct>();
            }
            if (IsParent<CCPACSFuselage>()) {
                return GetParent<CCPACSFuselage>();
            }
            if (IsParent<CCPACSHull>()) {
                return GetParent<CCPACSHull>();
            }
        }
        return nullptr;
    }

    CTiglUIDObject* CPACSFuselageSections::GetNextUIDParent()
    {
        if (m_parent) {
            if (IsParent<CCPACSDuct>()) {
                return GetParent<CCPACSDuct>();
            }
            if (IsParent<CCPACSFuselage>()) {
                return GetParent<CCPACSFuselage>();
            }
            if (IsParent<CCPACSHull>()) {
                return GetParent<CCPACSHull>();
            }
        }
        return nullptr;
    }

    CTiglUIDManager& CPACSFuselageSections::GetUIDManager()
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    const CTiglUIDManager& CPACSFuselageSections::GetUIDManager() const
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    void CPACSFuselageSections::ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath)
    {
        // read element section
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/section")) {
            tixi::TixiReadElements(tixiHandle, xpath + "/section", m_sections, 2, tixi::xsdUnbounded, reinterpret_cast<CCPACSFuselageSections*>(this), m_uidMgr);
        }

    }

    void CPACSFuselageSections::WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const
    {
        // write element section
        tixi::TixiSaveElements(tixiHandle, xpath + "/section", m_sections);

    }

    const std::vector<std::unique_ptr<CCPACSFuselageSection>>& CPACSFuselageSections::GetSections() const
    {
        return m_sections;
    }

    std::vector<std::unique_ptr<CCPACSFuselageSection>>& CPACSFuselageSections::GetSections()
    {
        return m_sections;
    }

    CCPACSFuselageSection& CPACSFuselageSections::AddSection()
    {
        m_sections.push_back(make_unique<CCPACSFuselageSection>(reinterpret_cast<CCPACSFuselageSections*>(this), m_uidMgr));
        return *m_sections.back();
    }

    void CPACSFuselageSections::RemoveSection(CCPACSFuselageSection& ref)
    {
        for (std::size_t i = 0; i < m_sections.size(); i++) {
            if (m_sections[i].get() == &ref) {
                m_sections.erase(m_sections.begin() + i);
                return;
            }
        }
        throw CTiglError("Element not found");
    }

} // namespace generated
} // namespace tigl
