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
#include "CPACSGenericComponents.h"
#include "CPACSVehicleElementBase.h"
#include "CTiglError.h"
#include "CTiglLogging.h"
#include "CTiglUIDManager.h"
#include "TixiHelper.h"

namespace tigl
{
namespace generated
{
    CPACSVehicleElementBase::CPACSVehicleElementBase(CPACSGenericComponents* parent, CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
        , m_geometry(this, m_uidMgr)
    {
        //assert(parent != NULL);
        m_parent = parent;
    }

    CPACSVehicleElementBase::~CPACSVehicleElementBase()
    {
        if (m_uidMgr) m_uidMgr->TryUnregisterObject(m_uID);
    }

    const CPACSGenericComponents* CPACSVehicleElementBase::GetParent() const
    {
        return m_parent;
    }

    CPACSGenericComponents* CPACSVehicleElementBase::GetParent()
    {
        return m_parent;
    }

    const CTiglUIDObject* CPACSVehicleElementBase::GetNextUIDParent() const
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    CTiglUIDObject* CPACSVehicleElementBase::GetNextUIDParent()
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    CTiglUIDManager& CPACSVehicleElementBase::GetUIDManager()
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    const CTiglUIDManager& CPACSVehicleElementBase::GetUIDManager() const
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    void CPACSVehicleElementBase::ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath)
    {
        // read attribute uID
        if (tixi::TixiCheckAttribute(tixiHandle, xpath, "uID")) {
            m_uID = tixi::TixiGetAttribute<std::string>(tixiHandle, xpath, "uID");
            if (m_uID.empty()) {
                LOG(WARNING) << "Required attribute uID is empty at xpath " << xpath;
            }
        }
        else {
            LOG(ERROR) << "Required attribute uID is missing at xpath " << xpath;
        }

        // read element name
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/name")) {
            m_name = tixi::TixiGetElement<std::string>(tixiHandle, xpath + "/name");
            if (m_name.empty()) {
                LOG(WARNING) << "Required element name is empty at xpath " << xpath;
            }
        }
        else {
            LOG(ERROR) << "Required element name is missing at xpath " << xpath;
        }

        // read element description
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/description")) {
            m_description = tixi::TixiGetElement<std::string>(tixiHandle, xpath + "/description");
            if (m_description->empty()) {
                LOG(WARNING) << "Optional element description is present but empty at xpath " << xpath;
            }
        }

        // read element geometry
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/geometry")) {
            m_geometry.ReadCPACS(tixiHandle, xpath + "/geometry");
        }
        else {
            LOG(ERROR) << "Required element geometry is missing at xpath " << xpath;
        }

        // read element mass
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/mass")) {
            m_mass = boost::in_place(this, m_uidMgr);
            try {
                m_mass->ReadCPACS(tixiHandle, xpath + "/mass");
            } catch(const std::exception& e) {
                LOG(ERROR) << "Failed to read mass at xpath " << xpath << ": " << e.what();
                m_mass = boost::none;
            }
        }

        if (m_uidMgr && !m_uID.empty()) m_uidMgr->RegisterObject(m_uID, *this);
    }

    void CPACSVehicleElementBase::WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const
    {
        const std::vector<std::string> childElemOrder = { "name", "description", "geometry", "mass" };

        // write attribute uID
        tixi::TixiSaveAttribute(tixiHandle, xpath, "uID", m_uID);

        // write element name
        tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/name", childElemOrder);
        tixi::TixiSaveElement(tixiHandle, xpath + "/name", m_name);

        // write element description
        if (m_description) {
            tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/description", childElemOrder);
            tixi::TixiSaveElement(tixiHandle, xpath + "/description", *m_description);
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/description")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/description");
            }
        }

        // write element geometry
        tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/geometry", childElemOrder);
        m_geometry.WriteCPACS(tixiHandle, xpath + "/geometry");

        // write element mass
        if (m_mass) {
            tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/mass", childElemOrder);
            m_mass->WriteCPACS(tixiHandle, xpath + "/mass");
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/mass")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/mass");
            }
        }

    }

    const std::string& CPACSVehicleElementBase::GetUID() const
    {
        return m_uID;
    }

    void CPACSVehicleElementBase::SetUID(const std::string& value)
    {
        if (m_uidMgr && value != m_uID) {
            if (m_uID.empty()) {
                m_uidMgr->RegisterObject(value, *this);
            }
            else {
                m_uidMgr->UpdateObjectUID(m_uID, value);
            }
        }
        m_uID = value;
    }

    const std::string& CPACSVehicleElementBase::GetName() const
    {
        return m_name;
    }

    void CPACSVehicleElementBase::SetName(const std::string& value)
    {
        m_name = value;
    }

    const boost::optional<std::string>& CPACSVehicleElementBase::GetDescription() const
    {
        return m_description;
    }

    void CPACSVehicleElementBase::SetDescription(const boost::optional<std::string>& value)
    {
        m_description = value;
    }

    const CPACSElementGeometry& CPACSVehicleElementBase::GetGeometry() const
    {
        return m_geometry;
    }

    CPACSElementGeometry& CPACSVehicleElementBase::GetGeometry()
    {
        return m_geometry;
    }

    const boost::optional<CPACSElementMass>& CPACSVehicleElementBase::GetMass() const
    {
        return m_mass;
    }

    boost::optional<CPACSElementMass>& CPACSVehicleElementBase::GetMass()
    {
        return m_mass;
    }

    CPACSElementMass& CPACSVehicleElementBase::GetMass(CreateIfNotExistsTag)
    {
        if (!m_mass)
            m_mass = boost::in_place(this, m_uidMgr);
        return *m_mass;
    }

    void CPACSVehicleElementBase::RemoveMass()
    {
        m_mass = boost::none;
    }

} // namespace generated
} // namespace tigl
