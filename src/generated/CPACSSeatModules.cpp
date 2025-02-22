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
#include "CPACSDeckComponent2DBase.h"
#include "CPACSSeatModules.h"
#include "CPACSStructuralElements.h"
#include "CTiglError.h"
#include "CTiglLogging.h"
#include "CTiglUIDManager.h"
#include "CTiglUIDObject.h"
#include "TixiHelper.h"

namespace tigl
{
namespace generated
{
    CPACSSeatModules::CPACSSeatModules(CPACSStructuralElements* parent, CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
    {
        //assert(parent != NULL);
        m_parent = parent;
    }

    CPACSSeatModules::~CPACSSeatModules()
    {
    }

    const CPACSStructuralElements* CPACSSeatModules::GetParent() const
    {
        return m_parent;
    }

    CPACSStructuralElements* CPACSSeatModules::GetParent()
    {
        return m_parent;
    }

    const CTiglUIDObject* CPACSSeatModules::GetNextUIDParent() const
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    CTiglUIDObject* CPACSSeatModules::GetNextUIDParent()
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    CTiglUIDManager& CPACSSeatModules::GetUIDManager()
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    const CTiglUIDManager& CPACSSeatModules::GetUIDManager() const
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    void CPACSSeatModules::ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath)
    {
        // read element seatModule
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/seatModule")) {
            tixi::TixiReadElements(tixiHandle, xpath + "/seatModule", m_seatModules, 1, tixi::xsdUnbounded, this, m_uidMgr);
        }

    }

    void CPACSSeatModules::WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const
    {
        // write element seatModule
        tixi::TixiSaveElements(tixiHandle, xpath + "/seatModule", m_seatModules);

    }

    const std::vector<std::unique_ptr<CPACSDeckComponent2DBase>>& CPACSSeatModules::GetSeatModules() const
    {
        return m_seatModules;
    }

    std::vector<std::unique_ptr<CPACSDeckComponent2DBase>>& CPACSSeatModules::GetSeatModules()
    {
        return m_seatModules;
    }

    CPACSDeckComponent2DBase& CPACSSeatModules::AddSeatModule()
    {
        m_seatModules.push_back(make_unique<CPACSDeckComponent2DBase>(this, m_uidMgr));
        return *m_seatModules.back();
    }

    void CPACSSeatModules::RemoveSeatModule(CPACSDeckComponent2DBase& ref)
    {
        for (std::size_t i = 0; i < m_seatModules.size(); i++) {
            if (m_seatModules[i].get() == &ref) {
                m_seatModules.erase(m_seatModules.begin() + i);
                return;
            }
        }
        throw CTiglError("Element not found");
    }

} // namespace generated
} // namespace tigl
