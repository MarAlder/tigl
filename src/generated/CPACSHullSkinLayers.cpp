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
#include <CCPACSSkin.h>
#include "CPACSHullSkinLayers.h"
#include "CPACSHullStructure.h"
#include "CTiglError.h"
#include "CTiglLogging.h"
#include "CTiglUIDManager.h"
#include "CTiglUIDObject.h"
#include "TixiHelper.h"

namespace tigl
{
namespace generated
{
    CPACSHullSkinLayers::CPACSHullSkinLayers(CPACSHullStructure* parent, CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
    {
        //assert(parent != NULL);
        m_parent = parent;
    }

    CPACSHullSkinLayers::~CPACSHullSkinLayers()
    {
    }

    const CPACSHullStructure* CPACSHullSkinLayers::GetParent() const
    {
        return m_parent;
    }

    CPACSHullStructure* CPACSHullSkinLayers::GetParent()
    {
        return m_parent;
    }

    const CTiglUIDObject* CPACSHullSkinLayers::GetNextUIDParent() const
    {
        return m_parent;
    }

    CTiglUIDObject* CPACSHullSkinLayers::GetNextUIDParent()
    {
        return m_parent;
    }

    CTiglUIDManager& CPACSHullSkinLayers::GetUIDManager()
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    const CTiglUIDManager& CPACSHullSkinLayers::GetUIDManager() const
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    void CPACSHullSkinLayers::ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath)
    {
        // read element skinLayer
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/skinLayer")) {
            tixi::TixiReadElements(tixiHandle, xpath + "/skinLayer", m_skinLayers, 1, tixi::xsdUnbounded, this, m_uidMgr);
        }

    }

    void CPACSHullSkinLayers::WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const
    {
        // write element skinLayer
        tixi::TixiSaveElements(tixiHandle, xpath + "/skinLayer", m_skinLayers);

    }

    const std::vector<std::unique_ptr<CCPACSSkin>>& CPACSHullSkinLayers::GetSkinLayers() const
    {
        return m_skinLayers;
    }

    std::vector<std::unique_ptr<CCPACSSkin>>& CPACSHullSkinLayers::GetSkinLayers()
    {
        return m_skinLayers;
    }

    CCPACSSkin& CPACSHullSkinLayers::AddSkinLayer()
    {
        m_skinLayers.push_back(make_unique<CCPACSSkin>(this, m_uidMgr));
        return *m_skinLayers.back();
    }

    void CPACSHullSkinLayers::RemoveSkinLayer(CCPACSSkin& ref)
    {
        for (std::size_t i = 0; i < m_skinLayers.size(); i++) {
            if (m_skinLayers[i].get() == &ref) {
                m_skinLayers.erase(m_skinLayers.begin() + i);
                return;
            }
        }
        throw CTiglError("Element not found");
    }

} // namespace generated
} // namespace tigl