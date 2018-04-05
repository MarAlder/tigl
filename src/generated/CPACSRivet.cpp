// Copyright (c) 2018 RISC Software GmbH
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

#include "CPACSRivet.h"
#include "CTiglError.h"
#include "CTiglLogging.h"
#include "CTiglUIDManager.h"
#include "TixiHelper.h"

namespace tigl
{
namespace generated
{
    CPACSRivet::CPACSRivet(CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
        , m_tensileStrength(0)
        , m_shearStrength(0)
    {
    }

    CPACSRivet::~CPACSRivet()
    {
        if (m_uidMgr) m_uidMgr->TryUnregisterObject(m_uID);
    }

    CTiglUIDManager& CPACSRivet::GetUIDManager()
    {
        return *m_uidMgr;
    }

    const CTiglUIDManager& CPACSRivet::GetUIDManager() const
    {
        return *m_uidMgr;
    }

    void CPACSRivet::ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath)
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

        // read element tensileStrength
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/tensileStrength")) {
            m_tensileStrength = tixi::TixiGetElement<double>(tixiHandle, xpath + "/tensileStrength");
        }
        else {
            LOG(ERROR) << "Required element tensileStrength is missing at xpath " << xpath;
        }

        // read element shearStrength
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/shearStrength")) {
            m_shearStrength = tixi::TixiGetElement<double>(tixiHandle, xpath + "/shearStrength");
        }
        else {
            LOG(ERROR) << "Required element shearStrength is missing at xpath " << xpath;
        }

        if (m_uidMgr && !m_uID.empty()) m_uidMgr->RegisterObject(m_uID, *this);
    }

    void CPACSRivet::WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const
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

        // write element tensileStrength
        tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/tensileStrength");
        tixi::TixiSaveElement(tixiHandle, xpath + "/tensileStrength", m_tensileStrength);

        // write element shearStrength
        tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/shearStrength");
        tixi::TixiSaveElement(tixiHandle, xpath + "/shearStrength", m_shearStrength);

    }

    const std::string& CPACSRivet::GetUID() const
    {
        return m_uID;
    }

    void CPACSRivet::SetUID(const std::string& value)
    {
        if (m_uidMgr) {
            m_uidMgr->TryUnregisterObject(m_uID);
            m_uidMgr->RegisterObject(value, *this);
        }
        m_uID = value;
    }

    const boost::optional<std::string>& CPACSRivet::GetName() const
    {
        return m_name;
    }

    void CPACSRivet::SetName(const boost::optional<std::string>& value)
    {
        m_name = value;
    }

    const boost::optional<std::string>& CPACSRivet::GetDescription() const
    {
        return m_description;
    }

    void CPACSRivet::SetDescription(const boost::optional<std::string>& value)
    {
        m_description = value;
    }

    const double& CPACSRivet::GetTensileStrength() const
    {
        return m_tensileStrength;
    }

    void CPACSRivet::SetTensileStrength(const double& value)
    {
        m_tensileStrength = value;
    }

    const double& CPACSRivet::GetShearStrength() const
    {
        return m_shearStrength;
    }

    void CPACSRivet::SetShearStrength(const double& value)
    {
        m_shearStrength = value;
    }

} // namespace generated
} // namespace tigl