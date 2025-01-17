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
#include "CPACSElementGeometry.h"
#include "CPACSExternalGeometry.h"
#include "CTiglError.h"
#include "CTiglLogging.h"
#include "CTiglUIDManager.h"
#include "CTiglUIDObject.h"
#include "TixiHelper.h"

namespace tigl
{
namespace generated
{
    CPACSExternalGeometry::CPACSExternalGeometry(CPACSElementGeometry* parent, CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
        , m_linkToFile(this)
        , m_transformation(this, m_uidMgr)
    {
        //assert(parent != NULL);
        m_parent = parent;
    }

    CPACSExternalGeometry::~CPACSExternalGeometry()
    {
    }

    const CPACSElementGeometry* CPACSExternalGeometry::GetParent() const
    {
        return m_parent;
    }

    CPACSElementGeometry* CPACSExternalGeometry::GetParent()
    {
        return m_parent;
    }

    const CTiglUIDObject* CPACSExternalGeometry::GetNextUIDParent() const
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    CTiglUIDObject* CPACSExternalGeometry::GetNextUIDParent()
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    CTiglUIDManager& CPACSExternalGeometry::GetUIDManager()
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    const CTiglUIDManager& CPACSExternalGeometry::GetUIDManager() const
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    void CPACSExternalGeometry::ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath)
    {
        // read element linkToFile
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/linkToFile")) {
            m_linkToFile.ReadCPACS(tixiHandle, xpath + "/linkToFile");
        }
        else {
            LOG(ERROR) << "Required element linkToFile is missing at xpath " << xpath;
        }

        // read element transformation
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/transformation")) {
            m_transformation.ReadCPACS(tixiHandle, xpath + "/transformation");
        }
        else {
            LOG(ERROR) << "Required element transformation is missing at xpath " << xpath;
        }

    }

    void CPACSExternalGeometry::WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const
    {
        // write element linkToFile
        tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/linkToFile");
        m_linkToFile.WriteCPACS(tixiHandle, xpath + "/linkToFile");

        // write element transformation
        tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/transformation");
        m_transformation.WriteCPACS(tixiHandle, xpath + "/transformation");

    }

    const CPACSLinkToFile& CPACSExternalGeometry::GetLinkToFile() const
    {
        return m_linkToFile;
    }

    CPACSLinkToFile& CPACSExternalGeometry::GetLinkToFile()
    {
        return m_linkToFile;
    }

    const CCPACSTransformation& CPACSExternalGeometry::GetTransformation() const
    {
        return m_transformation;
    }

    CCPACSTransformation& CPACSExternalGeometry::GetTransformation()
    {
        return m_transformation;
    }

} // namespace generated
} // namespace tigl
