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
#include "CPACSFuselageFuelTank.h"
#include "CPACSFuselageFuelTanks.h"
#include "CTiglError.h"
#include "CTiglLogging.h"
#include "CTiglUIDManager.h"
#include "TixiHelper.h"

namespace tigl
{
namespace generated
{
    CPACSFuselageFuelTank::CPACSFuselageFuelTank(CPACSFuselageFuelTanks* parent, CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
    {
        //assert(parent != NULL);
        m_parent = parent;
    }

    CPACSFuselageFuelTank::~CPACSFuselageFuelTank()
    {
        if (m_uidMgr) m_uidMgr->TryUnregisterObject(m_uID);
        if (m_uidMgr) {
            if (!m_compartmentUID.empty()) m_uidMgr->TryUnregisterReference(m_compartmentUID, *this);
        }
    }

    const CPACSFuselageFuelTanks* CPACSFuselageFuelTank::GetParent() const
    {
        return m_parent;
    }

    CPACSFuselageFuelTanks* CPACSFuselageFuelTank::GetParent()
    {
        return m_parent;
    }

    const CTiglUIDObject* CPACSFuselageFuelTank::GetNextUIDParent() const
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    CTiglUIDObject* CPACSFuselageFuelTank::GetNextUIDParent()
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    CTiglUIDManager& CPACSFuselageFuelTank::GetUIDManager()
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    const CTiglUIDManager& CPACSFuselageFuelTank::GetUIDManager() const
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    void CPACSFuselageFuelTank::ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath)
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
            if (m_name->empty()) {
                LOG(WARNING) << "Optional element name is present but empty at xpath " << xpath;
            }
        }

        // read element description
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/description")) {
            m_description = tixi::TixiGetElement<std::string>(tixiHandle, xpath + "/description");
            if (m_description->empty()) {
                LOG(WARNING) << "Optional element description is present but empty at xpath " << xpath;
            }
        }

        // read element compartmentUID
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/compartmentUID")) {
            m_compartmentUID = tixi::TixiGetElement<std::string>(tixiHandle, xpath + "/compartmentUID");
            if (m_compartmentUID.empty()) {
                LOG(WARNING) << "Required element compartmentUID is empty at xpath " << xpath;
            }
            if (m_uidMgr && !m_compartmentUID.empty()) m_uidMgr->RegisterReference(m_compartmentUID, *this);
        }
        else {
            LOG(ERROR) << "Required element compartmentUID is missing at xpath " << xpath;
        }

        // read element volume
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/volume")) {
            m_volume = boost::in_place(this);
            try {
                m_volume->ReadCPACS(tixiHandle, xpath + "/volume");
            } catch(const std::exception& e) {
                LOG(ERROR) << "Failed to read volume at xpath " << xpath << ": " << e.what();
                m_volume = boost::none;
            }
        }

        if (m_uidMgr && !m_uID.empty()) m_uidMgr->RegisterObject(m_uID, *this);
    }

    void CPACSFuselageFuelTank::WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const
    {
        // write attribute uID
        tixi::TixiSaveAttribute(tixiHandle, xpath, "uID", m_uID);

        // write element name
        if (m_name) {
            tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/name");
            tixi::TixiSaveElement(tixiHandle, xpath + "/name", *m_name);
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/name")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/name");
            }
        }

        // write element description
        if (m_description) {
            tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/description");
            tixi::TixiSaveElement(tixiHandle, xpath + "/description", *m_description);
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/description")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/description");
            }
        }

        // write element compartmentUID
        tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/compartmentUID");
        tixi::TixiSaveElement(tixiHandle, xpath + "/compartmentUID", m_compartmentUID);

        // write element volume
        if (m_volume) {
            tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/volume");
            m_volume->WriteCPACS(tixiHandle, xpath + "/volume");
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/volume")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/volume");
            }
        }

    }

    const std::string& CPACSFuselageFuelTank::GetUID() const
    {
        return m_uID;
    }

    void CPACSFuselageFuelTank::SetUID(const std::string& value)
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

    const boost::optional<std::string>& CPACSFuselageFuelTank::GetName() const
    {
        return m_name;
    }

    void CPACSFuselageFuelTank::SetName(const boost::optional<std::string>& value)
    {
        m_name = value;
    }

    const boost::optional<std::string>& CPACSFuselageFuelTank::GetDescription() const
    {
        return m_description;
    }

    void CPACSFuselageFuelTank::SetDescription(const boost::optional<std::string>& value)
    {
        m_description = value;
    }

    const std::string& CPACSFuselageFuelTank::GetCompartmentUID() const
    {
        return m_compartmentUID;
    }

    void CPACSFuselageFuelTank::SetCompartmentUID(const std::string& value)
    {
        if (m_uidMgr) {
            if (!m_compartmentUID.empty()) m_uidMgr->TryUnregisterReference(m_compartmentUID, *this);
            if (!value.empty()) m_uidMgr->RegisterReference(value, *this);
        }
        m_compartmentUID = value;
    }

    const boost::optional<CPACSFuelTankVolume>& CPACSFuselageFuelTank::GetVolume() const
    {
        return m_volume;
    }

    boost::optional<CPACSFuelTankVolume>& CPACSFuselageFuelTank::GetVolume()
    {
        return m_volume;
    }

    CPACSFuelTankVolume& CPACSFuselageFuelTank::GetVolume(CreateIfNotExistsTag)
    {
        if (!m_volume)
            m_volume = boost::in_place(this);
        return *m_volume;
    }

    void CPACSFuselageFuelTank::RemoveVolume()
    {
        m_volume = boost::none;
    }

    const CTiglUIDObject* CPACSFuselageFuelTank::GetNextUIDObject() const
    {
        return this;
    }

    void CPACSFuselageFuelTank::NotifyUIDChange(const std::string& oldUid, const std::string& newUid)
    {
        if (m_compartmentUID == oldUid) {
            m_compartmentUID = newUid;
        }
    }

} // namespace generated
} // namespace tigl
