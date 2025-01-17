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
#include "CPACSElectricMotor.h"
#include "CPACSElectricMotors.h"
#include "CPACSSystemElements.h"
#include "CTiglError.h"
#include "CTiglLogging.h"
#include "CTiglUIDManager.h"
#include "CTiglUIDObject.h"
#include "TixiHelper.h"

namespace tigl
{
namespace generated
{
    CPACSElectricMotors::CPACSElectricMotors(CPACSSystemElements* parent, CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
    {
        //assert(parent != NULL);
        m_parent = parent;
    }

    CPACSElectricMotors::~CPACSElectricMotors()
    {
    }

    const CPACSSystemElements* CPACSElectricMotors::GetParent() const
    {
        return m_parent;
    }

    CPACSSystemElements* CPACSElectricMotors::GetParent()
    {
        return m_parent;
    }

    const CTiglUIDObject* CPACSElectricMotors::GetNextUIDParent() const
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    CTiglUIDObject* CPACSElectricMotors::GetNextUIDParent()
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    CTiglUIDManager& CPACSElectricMotors::GetUIDManager()
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    const CTiglUIDManager& CPACSElectricMotors::GetUIDManager() const
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    void CPACSElectricMotors::ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath)
    {
        // read element electricMotor
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/electricMotor")) {
            tixi::TixiReadElements(tixiHandle, xpath + "/electricMotor", m_electricMotors, 1, tixi::xsdUnbounded, this, m_uidMgr);
        }

    }

    void CPACSElectricMotors::WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const
    {
        // write element electricMotor
        tixi::TixiSaveElements(tixiHandle, xpath + "/electricMotor", m_electricMotors);

    }

    const std::vector<std::unique_ptr<CPACSElectricMotor>>& CPACSElectricMotors::GetElectricMotors() const
    {
        return m_electricMotors;
    }

    std::vector<std::unique_ptr<CPACSElectricMotor>>& CPACSElectricMotors::GetElectricMotors()
    {
        return m_electricMotors;
    }

    CPACSElectricMotor& CPACSElectricMotors::AddElectricMotor()
    {
        m_electricMotors.push_back(make_unique<CPACSElectricMotor>(this, m_uidMgr));
        return *m_electricMotors.back();
    }

    void CPACSElectricMotors::RemoveElectricMotor(CPACSElectricMotor& ref)
    {
        for (std::size_t i = 0; i < m_electricMotors.size(); i++) {
            if (m_electricMotors[i].get() == &ref) {
                m_electricMotors.erase(m_electricMotors.begin() + i);
                return;
            }
        }
        throw CTiglError("Element not found");
    }

} // namespace generated
} // namespace tigl