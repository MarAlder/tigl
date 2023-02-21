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
#include "CPACSLandingGearSupportBeamPosition.h"
#include "CPACSSupportBeam.h"
#include "CTiglError.h"
#include "CTiglLogging.h"
#include "CTiglUIDManager.h"
#include "CTiglUIDObject.h"
#include "TixiHelper.h"

namespace tigl
{
namespace generated
{
    CPACSLandingGearSupportBeamPosition::CPACSLandingGearSupportBeamPosition(CPACSSupportBeam* parent, CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
        , m_xsiInside(this, m_uidMgr)
        , m_etaOutside(this, m_uidMgr)
    {
        //assert(parent != NULL);
        m_parent = parent;
    }

    CPACSLandingGearSupportBeamPosition::~CPACSLandingGearSupportBeamPosition()
    {
    }

    const CPACSSupportBeam* CPACSLandingGearSupportBeamPosition::GetParent() const
    {
        return m_parent;
    }

    CPACSSupportBeam* CPACSLandingGearSupportBeamPosition::GetParent()
    {
        return m_parent;
    }

    const CTiglUIDObject* CPACSLandingGearSupportBeamPosition::GetNextUIDParent() const
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    CTiglUIDObject* CPACSLandingGearSupportBeamPosition::GetNextUIDParent()
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    CTiglUIDManager& CPACSLandingGearSupportBeamPosition::GetUIDManager()
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    const CTiglUIDManager& CPACSLandingGearSupportBeamPosition::GetUIDManager() const
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    void CPACSLandingGearSupportBeamPosition::ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath)
    {
        // read element xsiInside
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/xsiInside")) {
            m_xsiInside.ReadCPACS(tixiHandle, xpath + "/xsiInside");
        }
        else {
            LOG(ERROR) << "Required element xsiInside is missing at xpath " << xpath;
        }

        // read element etaOutside
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/etaOutside")) {
            m_etaOutside.ReadCPACS(tixiHandle, xpath + "/etaOutside");
        }
        else {
            LOG(ERROR) << "Required element etaOutside is missing at xpath " << xpath;
        }

    }

    void CPACSLandingGearSupportBeamPosition::WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const
    {
        const std::vector<std::string> childElemOrder = { "xsiInside", "etaOutside" };

        // write element xsiInside
        tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/xsiInside", childElemOrder);
        m_xsiInside.WriteCPACS(tixiHandle, xpath + "/xsiInside");

        // write element etaOutside
        tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/etaOutside", childElemOrder);
        m_etaOutside.WriteCPACS(tixiHandle, xpath + "/etaOutside");

    }

    const CCPACSXsiIsoLine& CPACSLandingGearSupportBeamPosition::GetXsiInside() const
    {
        return m_xsiInside;
    }

    CCPACSXsiIsoLine& CPACSLandingGearSupportBeamPosition::GetXsiInside()
    {
        return m_xsiInside;
    }

    const CCPACSEtaIsoLine& CPACSLandingGearSupportBeamPosition::GetEtaOutside() const
    {
        return m_etaOutside;
    }

    CCPACSEtaIsoLine& CPACSLandingGearSupportBeamPosition::GetEtaOutside()
    {
        return m_etaOutside;
    }

} // namespace generated
} // namespace tigl