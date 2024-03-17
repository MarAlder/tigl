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
#include "CCPACSGenericFuelTanks.h"
#include "CPACSGenericFuelTank.h"
#include "CTiglError.h"
#include "CTiglLogging.h"
#include "CTiglUIDManager.h"
#include "TixiHelper.h"

namespace tigl
{
namespace generated
{
    CPACSGenericFuelTank::CPACSGenericFuelTank(CCPACSGenericFuelTanks* parent, CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
        , m_transformation(reinterpret_cast<CCPACSGenericFuelTank*>(this), m_uidMgr)
    {
        //assert(parent != NULL);
        m_parent = parent;
    }

    CPACSGenericFuelTank::~CPACSGenericFuelTank()
    {
        if (m_uidMgr) m_uidMgr->TryUnregisterObject(m_uID);
    }

    const CCPACSGenericFuelTanks* CPACSGenericFuelTank::GetParent() const
    {
        return m_parent;
    }

    CCPACSGenericFuelTanks* CPACSGenericFuelTank::GetParent()
    {
        return m_parent;
    }

    const CTiglUIDObject* CPACSGenericFuelTank::GetNextUIDParent() const
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    CTiglUIDObject* CPACSGenericFuelTank::GetNextUIDParent()
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    CTiglUIDManager& CPACSGenericFuelTank::GetUIDManager()
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    const CTiglUIDManager& CPACSGenericFuelTank::GetUIDManager() const
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    void CPACSGenericFuelTank::ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath)
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

        // read element hulls
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/hulls")) {
            m_hulls_choice1 = boost::in_place(reinterpret_cast<CCPACSGenericFuelTank*>(this), m_uidMgr);
            try {
                m_hulls_choice1->ReadCPACS(tixiHandle, xpath + "/hulls");
            } catch(const std::exception& e) {
                LOG(ERROR) << "Failed to read hulls at xpath " << xpath << ": " << e.what();
                m_hulls_choice1 = boost::none;
            }
        }

        // read element designParameters
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/designParameters")) {
            m_designParameters_choice2 = boost::in_place(reinterpret_cast<CCPACSGenericFuelTank*>(this), m_uidMgr);
            try {
                m_designParameters_choice2->ReadCPACS(tixiHandle, xpath + "/designParameters");
            } catch(const std::exception& e) {
                LOG(ERROR) << "Failed to read designParameters at xpath " << xpath << ": " << e.what();
                m_designParameters_choice2 = boost::none;
            }
        }

        // read element transformation
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/transformation")) {
            m_transformation.ReadCPACS(tixiHandle, xpath + "/transformation");
        }
        else {
            LOG(ERROR) << "Required element transformation is missing at xpath " << xpath;
        }

        // read element volume
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/volume")) {
            m_volume = boost::in_place(reinterpret_cast<CCPACSGenericFuelTank*>(this));
            try {
                m_volume->ReadCPACS(tixiHandle, xpath + "/volume");
            } catch(const std::exception& e) {
                LOG(ERROR) << "Failed to read volume at xpath " << xpath << ": " << e.what();
                m_volume = boost::none;
            }
        }

        // read element burstPressure
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/burstPressure")) {
            m_burstPressure = tixi::TixiGetElement<double>(tixiHandle, xpath + "/burstPressure");
        }

        if (m_uidMgr && !m_uID.empty()) m_uidMgr->RegisterObject(m_uID, *this);
        if (!ValidateChoices()) {
            LOG(ERROR) << "Invalid choice configuration at xpath " << xpath;
        }
    }

    void CPACSGenericFuelTank::WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const
    {
        const std::vector<std::string> childElemOrder = { "name", "description", "hulls", "designParameters", "transformation", "volume", "burstPressure" };

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

        // write element hulls
        if (m_hulls_choice1) {
            tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/hulls", childElemOrder);
            m_hulls_choice1->WriteCPACS(tixiHandle, xpath + "/hulls");
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/hulls")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/hulls");
            }
        }

        // write element designParameters
        if (m_designParameters_choice2) {
            tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/designParameters", childElemOrder);
            m_designParameters_choice2->WriteCPACS(tixiHandle, xpath + "/designParameters");
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/designParameters")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/designParameters");
            }
        }

        // write element transformation
        tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/transformation", childElemOrder);
        m_transformation.WriteCPACS(tixiHandle, xpath + "/transformation");

        // write element volume
        if (m_volume) {
            tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/volume", childElemOrder);
            m_volume->WriteCPACS(tixiHandle, xpath + "/volume");
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/volume")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/volume");
            }
        }

        // write element burstPressure
        if (m_burstPressure) {
            tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/burstPressure", childElemOrder);
            tixi::TixiSaveElement(tixiHandle, xpath + "/burstPressure", *m_burstPressure);
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/burstPressure")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/burstPressure");
            }
        }

    }

    bool CPACSGenericFuelTank::ValidateChoices() const
    {
        return
        (
            (
                (
                    // mandatory elements of this choice must be there
                    m_hulls_choice1.is_initialized()
                    &&
                    // elements of other choices must not be there
                    !(
                        m_designParameters_choice2.is_initialized()
                    )
                )
                +
                (
                    // mandatory elements of this choice must be there
                    m_designParameters_choice2.is_initialized()
                    &&
                    // elements of other choices must not be there
                    !(
                        m_hulls_choice1.is_initialized()
                    )
                )
                == 1
            )
        )
        ;
    }

    const std::string& CPACSGenericFuelTank::GetUID() const
    {
        return m_uID;
    }

    void CPACSGenericFuelTank::SetUID(const std::string& value)
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

    const std::string& CPACSGenericFuelTank::GetName() const
    {
        return m_name;
    }

    void CPACSGenericFuelTank::SetName(const std::string& value)
    {
        m_name = value;
    }

    const boost::optional<std::string>& CPACSGenericFuelTank::GetDescription() const
    {
        return m_description;
    }

    void CPACSGenericFuelTank::SetDescription(const boost::optional<std::string>& value)
    {
        m_description = value;
    }

    const boost::optional<CCPACSHulls>& CPACSGenericFuelTank::GetHulls_choice1() const
    {
        return m_hulls_choice1;
    }

    boost::optional<CCPACSHulls>& CPACSGenericFuelTank::GetHulls_choice1()
    {
        return m_hulls_choice1;
    }

    const boost::optional<CPACSGenericFuelTankParameters>& CPACSGenericFuelTank::GetDesignParameters_choice2() const
    {
        return m_designParameters_choice2;
    }

    boost::optional<CPACSGenericFuelTankParameters>& CPACSGenericFuelTank::GetDesignParameters_choice2()
    {
        return m_designParameters_choice2;
    }

    const CCPACSTransformation& CPACSGenericFuelTank::GetTransformation() const
    {
        return m_transformation;
    }

    CCPACSTransformation& CPACSGenericFuelTank::GetTransformation()
    {
        return m_transformation;
    }

    const boost::optional<CPACSFuelTankVolume>& CPACSGenericFuelTank::GetVolume() const
    {
        return m_volume;
    }

    boost::optional<CPACSFuelTankVolume>& CPACSGenericFuelTank::GetVolume()
    {
        return m_volume;
    }

    const boost::optional<double>& CPACSGenericFuelTank::GetBurstPressure() const
    {
        return m_burstPressure;
    }

    void CPACSGenericFuelTank::SetBurstPressure(const boost::optional<double>& value)
    {
        m_burstPressure = value;
    }

    CCPACSHulls& CPACSGenericFuelTank::GetHulls_choice1(CreateIfNotExistsTag)
    {
        if (!m_hulls_choice1)
            m_hulls_choice1 = boost::in_place(reinterpret_cast<CCPACSGenericFuelTank*>(this), m_uidMgr);
        return *m_hulls_choice1;
    }

    void CPACSGenericFuelTank::RemoveHulls_choice1()
    {
        m_hulls_choice1 = boost::none;
    }

    CPACSGenericFuelTankParameters& CPACSGenericFuelTank::GetDesignParameters_choice2(CreateIfNotExistsTag)
    {
        if (!m_designParameters_choice2)
            m_designParameters_choice2 = boost::in_place(reinterpret_cast<CCPACSGenericFuelTank*>(this), m_uidMgr);
        return *m_designParameters_choice2;
    }

    void CPACSGenericFuelTank::RemoveDesignParameters_choice2()
    {
        m_designParameters_choice2 = boost::none;
    }

    CPACSFuelTankVolume& CPACSGenericFuelTank::GetVolume(CreateIfNotExistsTag)
    {
        if (!m_volume)
            m_volume = boost::in_place(reinterpret_cast<CCPACSGenericFuelTank*>(this));
        return *m_volume;
    }

    void CPACSGenericFuelTank::RemoveVolume()
    {
        m_volume = boost::none;
    }

} // namespace generated
} // namespace tigl
