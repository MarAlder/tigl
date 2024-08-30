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
#include "CPACSVersionInfo.h"
#include "CPACSVersionInfos.h"
#include "CTiglError.h"
#include "CTiglLogging.h"
#include "CTiglUIDObject.h"
#include "TixiHelper.h"

namespace tigl
{
namespace generated
{
    CPACSVersionInfo::CPACSVersionInfo(CPACSVersionInfos* parent)
        : m_timestamp(0)
    {
        //assert(parent != NULL);
        m_parent = parent;
    }

    CPACSVersionInfo::~CPACSVersionInfo()
    {
    }

    const CPACSVersionInfos* CPACSVersionInfo::GetParent() const
    {
        return m_parent;
    }

    CPACSVersionInfos* CPACSVersionInfo::GetParent()
    {
        return m_parent;
    }

    const CTiglUIDObject* CPACSVersionInfo::GetNextUIDParent() const
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    CTiglUIDObject* CPACSVersionInfo::GetNextUIDParent()
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    void CPACSVersionInfo::ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath)
    {
        // read attribute version
        if (tixi::TixiCheckAttribute(tixiHandle, xpath, "version")) {
            m_version = tixi::TixiGetAttribute<std::string>(tixiHandle, xpath, "version");
            if (m_version.empty()) {
                LOG(WARNING) << "Required attribute version is empty at xpath " << xpath;
            }
        }
        else {
            LOG(ERROR) << "Required attribute version is missing at xpath " << xpath;
        }

        // read element cpacsVersion
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/cpacsVersion")) {
            m_cpacsVersion = tixi::TixiGetElement<std::string>(tixiHandle, xpath + "/cpacsVersion");
            if (m_cpacsVersion.empty()) {
                LOG(WARNING) << "Required element cpacsVersion is empty at xpath " << xpath;
            }
        }
        else {
            LOG(ERROR) << "Required element cpacsVersion is missing at xpath " << xpath;
        }

        // read element description
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/description")) {
            m_description = tixi::TixiGetElement<std::string>(tixiHandle, xpath + "/description");
            if (m_description.empty()) {
                LOG(WARNING) << "Required element description is empty at xpath " << xpath;
            }
        }
        else {
            LOG(ERROR) << "Required element description is missing at xpath " << xpath;
        }

        // read element timestamp
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/timestamp")) {
            m_timestamp = tixi::TixiGetElement<std::time_t>(tixiHandle, xpath + "/timestamp");
        }
        else {
            LOG(ERROR) << "Required element timestamp is missing at xpath " << xpath;
        }

        // read element creator
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/creator")) {
            m_creator = tixi::TixiGetElement<std::string>(tixiHandle, xpath + "/creator");
            if (m_creator.empty()) {
                LOG(WARNING) << "Required element creator is empty at xpath " << xpath;
            }
        }
        else {
            LOG(ERROR) << "Required element creator is missing at xpath " << xpath;
        }

    }

    void CPACSVersionInfo::WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const
    {
        // write attribute version
        tixi::TixiSaveAttribute(tixiHandle, xpath, "version", m_version);

        // write element cpacsVersion
        tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/cpacsVersion");
        tixi::TixiSaveElement(tixiHandle, xpath + "/cpacsVersion", m_cpacsVersion);

        // write element description
        tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/description");
        tixi::TixiSaveElement(tixiHandle, xpath + "/description", m_description);

        // write element timestamp
        tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/timestamp");
        tixi::TixiSaveElement(tixiHandle, xpath + "/timestamp", m_timestamp);

        // write element creator
        tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/creator");
        tixi::TixiSaveElement(tixiHandle, xpath + "/creator", m_creator);

    }

    const std::string& CPACSVersionInfo::GetVersion() const
    {
        return m_version;
    }

    void CPACSVersionInfo::SetVersion(const std::string& value)
    {
        m_version = value;
    }

    const std::string& CPACSVersionInfo::GetCpacsVersion() const
    {
        return m_cpacsVersion;
    }

    void CPACSVersionInfo::SetCpacsVersion(const std::string& value)
    {
        m_cpacsVersion = value;
    }

    const std::string& CPACSVersionInfo::GetDescription() const
    {
        return m_description;
    }

    void CPACSVersionInfo::SetDescription(const std::string& value)
    {
        m_description = value;
    }

    const std::time_t& CPACSVersionInfo::GetTimestamp() const
    {
        return m_timestamp;
    }

    void CPACSVersionInfo::SetTimestamp(const std::time_t& value)
    {
        m_timestamp = value;
    }

    const std::string& CPACSVersionInfo::GetCreator() const
    {
        return m_creator;
    }

    void CPACSVersionInfo::SetCreator(const std::string& value)
    {
        m_creator = value;
    }

} // namespace generated
} // namespace tigl