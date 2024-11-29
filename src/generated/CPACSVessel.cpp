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
#include "CCPACSVessels.h"
#include "CPACSVessel.h"
#include "CTiglError.h"
#include "CTiglLogging.h"
#include "CTiglUIDManager.h"
#include "TixiHelper.h"

namespace tigl
{
namespace generated
{
    CPACSVessel::CPACSVessel(CCPACSVessels* parent, CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
        , m_transformation(reinterpret_cast<CCPACSVessel*>(this), m_uidMgr)
    {
        //assert(parent != NULL);
        m_parent = parent;
    }

    CPACSVessel::~CPACSVessel()
    {
        if (m_uidMgr) m_uidMgr->TryUnregisterObject(m_uID);
    }

    const CCPACSVessels* CPACSVessel::GetParent() const
    {
        return m_parent;
    }

    CCPACSVessels* CPACSVessel::GetParent()
    {
        return m_parent;
    }

    const CTiglUIDObject* CPACSVessel::GetNextUIDParent() const
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    CTiglUIDObject* CPACSVessel::GetNextUIDParent()
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    CTiglUIDManager& CPACSVessel::GetUIDManager()
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    const CTiglUIDManager& CPACSVessel::GetUIDManager() const
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    void CPACSVessel::ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath)
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

        // read element transformation
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/transformation")) {
            m_transformation.ReadCPACS(tixiHandle, xpath + "/transformation");
        }
        else {
            LOG(ERROR) << "Required element transformation is missing at xpath " << xpath;
        }

        // read element sections
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/sections")) {
            m_sections_choice1 = boost::in_place(reinterpret_cast<CCPACSVessel*>(this), m_uidMgr);
            try {
                m_sections_choice1->ReadCPACS(tixiHandle, xpath + "/sections");
            } catch(const std::exception& e) {
                LOG(ERROR) << "Failed to read sections at xpath " << xpath << ": " << e.what();
                m_sections_choice1 = boost::none;
            }
        }

        // read element segments
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/segments")) {
            m_segments_choice1 = boost::in_place(reinterpret_cast<CCPACSVessel*>(this), m_uidMgr);
            try {
                m_segments_choice1->ReadCPACS(tixiHandle, xpath + "/segments");
            } catch(const std::exception& e) {
                LOG(ERROR) << "Failed to read segments at xpath " << xpath << ": " << e.what();
                m_segments_choice1 = boost::none;
            }
        }

        // read element cylinderRadius
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/cylinderRadius")) {
            m_cylinderRadius_choice2 = tixi::TixiGetElement<double>(tixiHandle, xpath + "/cylinderRadius");
        }

        // read element cylinderLength
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/cylinderLength")) {
            m_cylinderLength_choice2 = tixi::TixiGetElement<double>(tixiHandle, xpath + "/cylinderLength");
        }

        // read element domeType
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/domeType")) {
            m_domeType_choice2 = boost::in_place(reinterpret_cast<CCPACSVessel*>(this));
            try {
                m_domeType_choice2->ReadCPACS(tixiHandle, xpath + "/domeType");
            } catch(const std::exception& e) {
                LOG(ERROR) << "Failed to read domeType at xpath " << xpath << ": " << e.what();
                m_domeType_choice2 = boost::none;
            }
        }

        // read element structure
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/structure")) {
            m_structure = boost::in_place(reinterpret_cast<CCPACSVessel*>(this), m_uidMgr);
            try {
                m_structure->ReadCPACS(tixiHandle, xpath + "/structure");
            } catch(const std::exception& e) {
                LOG(ERROR) << "Failed to read structure at xpath " << xpath << ": " << e.what();
                m_structure = boost::none;
            }
        }

        // read element volume
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/volume")) {
            m_volume = boost::in_place(reinterpret_cast<CCPACSVessel*>(this));
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

    void CPACSVessel::WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const
    {
        const std::vector<std::string> childElemOrder = { "name", "description", "transformation", "sections", "segments", "cylinderRadius", "cylinderLength", "domeType", "structure", "volume", "burstPressure" };

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

        // write element transformation
        tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/transformation", childElemOrder);
        m_transformation.WriteCPACS(tixiHandle, xpath + "/transformation");

        // write element sections
        if (m_sections_choice1) {
            tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/sections", childElemOrder);
            m_sections_choice1->WriteCPACS(tixiHandle, xpath + "/sections");
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/sections")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/sections");
            }
        }

        // write element segments
        if (m_segments_choice1) {
            tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/segments", childElemOrder);
            m_segments_choice1->WriteCPACS(tixiHandle, xpath + "/segments");
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/segments")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/segments");
            }
        }

        // write element cylinderRadius
        if (m_cylinderRadius_choice2) {
            tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/cylinderRadius", childElemOrder);
            tixi::TixiSaveElement(tixiHandle, xpath + "/cylinderRadius", *m_cylinderRadius_choice2);
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/cylinderRadius")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/cylinderRadius");
            }
        }

        // write element cylinderLength
        if (m_cylinderLength_choice2) {
            tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/cylinderLength", childElemOrder);
            tixi::TixiSaveElement(tixiHandle, xpath + "/cylinderLength", *m_cylinderLength_choice2);
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/cylinderLength")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/cylinderLength");
            }
        }

        // write element domeType
        if (m_domeType_choice2) {
            tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/domeType", childElemOrder);
            m_domeType_choice2->WriteCPACS(tixiHandle, xpath + "/domeType");
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/domeType")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/domeType");
            }
        }

        // write element structure
        if (m_structure) {
            tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/structure", childElemOrder);
            m_structure->WriteCPACS(tixiHandle, xpath + "/structure");
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/structure")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/structure");
            }
        }

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

    bool CPACSVessel::ValidateChoices() const
    {
        return
        (
            (
                (
                    // mandatory elements of this choice must be there
                    m_sections_choice1.is_initialized()
                    &&
                    m_segments_choice1.is_initialized()
                    &&
                    // elements of other choices must not be there
                    !(
                        m_cylinderRadius_choice2.is_initialized()
                        ||
                        m_cylinderLength_choice2.is_initialized()
                        ||
                        m_domeType_choice2.is_initialized()
                    )
                )
                +
                (
                    // mandatory elements of this choice must be there
                    m_cylinderRadius_choice2.is_initialized()
                    &&
                    m_cylinderLength_choice2.is_initialized()
                    &&
                    m_domeType_choice2.is_initialized()
                    &&
                    // elements of other choices must not be there
                    !(
                        m_sections_choice1.is_initialized()
                        ||
                        m_segments_choice1.is_initialized()
                    )
                )
                == 1
            )
        )
        ;
    }

    const std::string& CPACSVessel::GetUID() const
    {
        return m_uID;
    }

    void CPACSVessel::SetUID(const std::string& value)
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

    const std::string& CPACSVessel::GetName() const
    {
        return m_name;
    }

    void CPACSVessel::SetName(const std::string& value)
    {
        m_name = value;
    }

    const boost::optional<std::string>& CPACSVessel::GetDescription() const
    {
        return m_description;
    }

    void CPACSVessel::SetDescription(const boost::optional<std::string>& value)
    {
        m_description = value;
    }

    const CCPACSTransformation& CPACSVessel::GetTransformation() const
    {
        return m_transformation;
    }

    CCPACSTransformation& CPACSVessel::GetTransformation()
    {
        return m_transformation;
    }

    const boost::optional<CCPACSFuselageSections>& CPACSVessel::GetSections_choice1() const
    {
        return m_sections_choice1;
    }

    boost::optional<CCPACSFuselageSections>& CPACSVessel::GetSections_choice1()
    {
        return m_sections_choice1;
    }

    const boost::optional<CCPACSFuselageSegments>& CPACSVessel::GetSegments_choice1() const
    {
        return m_segments_choice1;
    }

    boost::optional<CCPACSFuselageSegments>& CPACSVessel::GetSegments_choice1()
    {
        return m_segments_choice1;
    }

    const boost::optional<double>& CPACSVessel::GetCylinderRadius_choice2() const
    {
        return m_cylinderRadius_choice2;
    }

    void CPACSVessel::SetCylinderRadius_choice2(const boost::optional<double>& value)
    {
        m_cylinderRadius_choice2 = value;
    }

    const boost::optional<double>& CPACSVessel::GetCylinderLength_choice2() const
    {
        return m_cylinderLength_choice2;
    }

    void CPACSVessel::SetCylinderLength_choice2(const boost::optional<double>& value)
    {
        m_cylinderLength_choice2 = value;
    }

    const boost::optional<CPACSDomeType>& CPACSVessel::GetDomeType_choice2() const
    {
        return m_domeType_choice2;
    }

    boost::optional<CPACSDomeType>& CPACSVessel::GetDomeType_choice2()
    {
        return m_domeType_choice2;
    }

    const boost::optional<CCPACSVesselStructure>& CPACSVessel::GetStructure() const
    {
        return m_structure;
    }

    boost::optional<CCPACSVesselStructure>& CPACSVessel::GetStructure()
    {
        return m_structure;
    }

    const boost::optional<CPACSFuelTankVolume>& CPACSVessel::GetVolume() const
    {
        return m_volume;
    }

    boost::optional<CPACSFuelTankVolume>& CPACSVessel::GetVolume()
    {
        return m_volume;
    }

    const boost::optional<double>& CPACSVessel::GetBurstPressure() const
    {
        return m_burstPressure;
    }

    void CPACSVessel::SetBurstPressure(const boost::optional<double>& value)
    {
        m_burstPressure = value;
    }

    CCPACSFuselageSections& CPACSVessel::GetSections_choice1(CreateIfNotExistsTag)
    {
        if (!m_sections_choice1)
            m_sections_choice1 = boost::in_place(reinterpret_cast<CCPACSVessel*>(this), m_uidMgr);
        return *m_sections_choice1;
    }

    void CPACSVessel::RemoveSections_choice1()
    {
        m_sections_choice1 = boost::none;
    }

    CCPACSFuselageSegments& CPACSVessel::GetSegments_choice1(CreateIfNotExistsTag)
    {
        if (!m_segments_choice1)
            m_segments_choice1 = boost::in_place(reinterpret_cast<CCPACSVessel*>(this), m_uidMgr);
        return *m_segments_choice1;
    }

    void CPACSVessel::RemoveSegments_choice1()
    {
        m_segments_choice1 = boost::none;
    }

    CPACSDomeType& CPACSVessel::GetDomeType_choice2(CreateIfNotExistsTag)
    {
        if (!m_domeType_choice2)
            m_domeType_choice2 = boost::in_place(reinterpret_cast<CCPACSVessel*>(this));
        return *m_domeType_choice2;
    }

    void CPACSVessel::RemoveDomeType_choice2()
    {
        m_domeType_choice2 = boost::none;
    }

    CCPACSVesselStructure& CPACSVessel::GetStructure(CreateIfNotExistsTag)
    {
        if (!m_structure)
            m_structure = boost::in_place(reinterpret_cast<CCPACSVessel*>(this), m_uidMgr);
        return *m_structure;
    }

    void CPACSVessel::RemoveStructure()
    {
        m_structure = boost::none;
    }

    CPACSFuelTankVolume& CPACSVessel::GetVolume(CreateIfNotExistsTag)
    {
        if (!m_volume)
            m_volume = boost::in_place(reinterpret_cast<CCPACSVessel*>(this));
        return *m_volume;
    }

    void CPACSVessel::RemoveVolume()
    {
        m_volume = boost::none;
    }

} // namespace generated
} // namespace tigl
