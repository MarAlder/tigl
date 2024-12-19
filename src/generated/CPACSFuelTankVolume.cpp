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
#include "CCPACSVessel.h"
#include "CPACSFuelTankVolume.h"
#include "CPACSFuselageFuelTank.h"
#include "CTiglError.h"
#include "CTiglLogging.h"
#include "CTiglUIDObject.h"
#include "TixiHelper.h"

namespace tigl
{
namespace generated
{
    CPACSFuelTankVolume::CPACSFuelTankVolume(CPACSFuselageFuelTank* parent)
        : m_optimalVolume(0)
    {
        //assert(parent != NULL);
        m_parent = parent;
        m_parentType = &typeid(CPACSFuselageFuelTank);
    }

    CPACSFuelTankVolume::CPACSFuelTankVolume(CCPACSVessel* parent)
        : m_optimalVolume(0)
    {
        //assert(parent != NULL);
        m_parent = parent;
        m_parentType = &typeid(CCPACSVessel);
    }

    CPACSFuelTankVolume::~CPACSFuelTankVolume()
    {
    }

    const CTiglUIDObject* CPACSFuelTankVolume::GetNextUIDParent() const
    {
        if (m_parent) {
            if (IsParent<CPACSFuselageFuelTank>()) {
                return GetParent<CPACSFuselageFuelTank>();
            }
            if (IsParent<CCPACSVessel>()) {
                return GetParent<CCPACSVessel>();
            }
        }
        return nullptr;
    }

    CTiglUIDObject* CPACSFuelTankVolume::GetNextUIDParent()
    {
        if (m_parent) {
            if (IsParent<CPACSFuselageFuelTank>()) {
                return GetParent<CPACSFuselageFuelTank>();
            }
            if (IsParent<CCPACSVessel>()) {
                return GetParent<CCPACSVessel>();
            }
        }
        return nullptr;
    }

    void CPACSFuelTankVolume::ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath)
    {
        // read element optimalVolume
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/optimalVolume")) {
            m_optimalVolume = tixi::TixiGetElement<double>(tixiHandle, xpath + "/optimalVolume");
        }
        else {
            LOG(ERROR) << "Required element optimalVolume is missing at xpath " << xpath;
        }

        // read element usableVolume
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/usableVolume")) {
            m_usableVolume_choice1 = tixi::TixiGetElement<double>(tixiHandle, xpath + "/usableVolume");
        }

        // read element realVolume
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/realVolume")) {
            m_realVolume_choice1 = tixi::TixiGetElement<double>(tixiHandle, xpath + "/realVolume");
        }

        // read element useableVolumeFactor
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/useableVolumeFactor")) {
            m_useableVolumeFactor_choice2 = tixi::TixiGetElement<double>(tixiHandle, xpath + "/useableVolumeFactor");
        }

        // read element realVolumeFactor
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/realVolumeFactor")) {
            m_realVolumeFactor_choice2 = tixi::TixiGetElement<double>(tixiHandle, xpath + "/realVolumeFactor");
        }

        if (!ValidateChoices()) {
            LOG(ERROR) << "Invalid choice configuration at xpath " << xpath;
        }
    }

    void CPACSFuelTankVolume::WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const
    {
        const std::vector<std::string> childElemOrder = { "optimalVolume", "usableVolume", "realVolume", "useableVolumeFactor", "realVolumeFactor" };

        // write element optimalVolume
        tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/optimalVolume", childElemOrder);
        tixi::TixiSaveElement(tixiHandle, xpath + "/optimalVolume", m_optimalVolume);

        // write element usableVolume
        if (m_usableVolume_choice1) {
            tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/usableVolume", childElemOrder);
            tixi::TixiSaveElement(tixiHandle, xpath + "/usableVolume", *m_usableVolume_choice1);
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/usableVolume")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/usableVolume");
            }
        }

        // write element realVolume
        if (m_realVolume_choice1) {
            tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/realVolume", childElemOrder);
            tixi::TixiSaveElement(tixiHandle, xpath + "/realVolume", *m_realVolume_choice1);
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/realVolume")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/realVolume");
            }
        }

        // write element useableVolumeFactor
        if (m_useableVolumeFactor_choice2) {
            tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/useableVolumeFactor", childElemOrder);
            tixi::TixiSaveElement(tixiHandle, xpath + "/useableVolumeFactor", *m_useableVolumeFactor_choice2);
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/useableVolumeFactor")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/useableVolumeFactor");
            }
        }

        // write element realVolumeFactor
        if (m_realVolumeFactor_choice2) {
            tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/realVolumeFactor", childElemOrder);
            tixi::TixiSaveElement(tixiHandle, xpath + "/realVolumeFactor", *m_realVolumeFactor_choice2);
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/realVolumeFactor")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/realVolumeFactor");
            }
        }

    }

    bool CPACSFuelTankVolume::ValidateChoices() const
    {
        return
        (
            (
                (
                    // mandatory elements of this choice must be there
                    true // m_usableVolume_choice1 is optional in choice
                    &&
                    true // m_realVolume_choice1 is optional in choice
                    &&
                    // elements of other choices must not be there
                    !(
                        m_useableVolumeFactor_choice2.is_initialized()
                        ||
                        m_realVolumeFactor_choice2.is_initialized()
                    )
                )
                +
                (
                    // mandatory elements of this choice must be there
                    true // m_useableVolumeFactor_choice2 is optional in choice
                    &&
                    true // m_realVolumeFactor_choice2 is optional in choice
                    &&
                    // elements of other choices must not be there
                    !(
                        m_usableVolume_choice1.is_initialized()
                        ||
                        m_realVolume_choice1.is_initialized()
                    )
                )
                == 1
            )
        )
        ;
    }

    const double& CPACSFuelTankVolume::GetOptimalVolume() const
    {
        return m_optimalVolume;
    }

    void CPACSFuelTankVolume::SetOptimalVolume(const double& value)
    {
        m_optimalVolume = value;
    }

    const boost::optional<double>& CPACSFuelTankVolume::GetUsableVolume_choice1() const
    {
        return m_usableVolume_choice1;
    }

    void CPACSFuelTankVolume::SetUsableVolume_choice1(const boost::optional<double>& value)
    {
        m_usableVolume_choice1 = value;
    }

    const boost::optional<double>& CPACSFuelTankVolume::GetRealVolume_choice1() const
    {
        return m_realVolume_choice1;
    }

    void CPACSFuelTankVolume::SetRealVolume_choice1(const boost::optional<double>& value)
    {
        m_realVolume_choice1 = value;
    }

    const boost::optional<double>& CPACSFuelTankVolume::GetUseableVolumeFactor_choice2() const
    {
        return m_useableVolumeFactor_choice2;
    }

    void CPACSFuelTankVolume::SetUseableVolumeFactor_choice2(const boost::optional<double>& value)
    {
        m_useableVolumeFactor_choice2 = value;
    }

    const boost::optional<double>& CPACSFuelTankVolume::GetRealVolumeFactor_choice2() const
    {
        return m_realVolumeFactor_choice2;
    }

    void CPACSFuelTankVolume::SetRealVolumeFactor_choice2(const boost::optional<double>& value)
    {
        m_realVolumeFactor_choice2 = value;
    }

} // namespace generated
} // namespace tigl