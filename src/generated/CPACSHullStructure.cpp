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
#include "CCPACSHull.h"
#include "CPACSHullStructure.h"
#include "CTiglError.h"
#include "CTiglLogging.h"
#include "CTiglUIDManager.h"
#include "TixiHelper.h"

namespace tigl
{
namespace generated
{
    CPACSHullStructure::CPACSHullStructure(CCPACSHull* parent, CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
    {
        //assert(parent != NULL);
        m_parent = parent;
    }

    CPACSHullStructure::~CPACSHullStructure()
    {
        if (m_uidMgr) m_uidMgr->TryUnregisterObject(m_uID);
    }

    const CCPACSHull* CPACSHullStructure::GetParent() const
    {
        return m_parent;
    }

    CCPACSHull* CPACSHullStructure::GetParent()
    {
        return m_parent;
    }

    const CTiglUIDObject* CPACSHullStructure::GetNextUIDParent() const
    {
        return m_parent;
    }

    CTiglUIDObject* CPACSHullStructure::GetNextUIDParent()
    {
        return m_parent;
    }

    CTiglUIDManager& CPACSHullStructure::GetUIDManager()
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    const CTiglUIDManager& CPACSHullStructure::GetUIDManager() const
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    void CPACSHullStructure::ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath)
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

        // read element stringers
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/stringers")) {
            m_stringers = boost::in_place(reinterpret_cast<CCPACSHullStructure*>(this), m_uidMgr);
            try {
                m_stringers->ReadCPACS(tixiHandle, xpath + "/stringers");
            } catch(const std::exception& e) {
                LOG(ERROR) << "Failed to read stringers at xpath " << xpath << ": " << e.what();
                m_stringers = boost::none;
            }
        }

        // read element frames
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/frames")) {
            m_frames = boost::in_place(reinterpret_cast<CCPACSHullStructure*>(this), m_uidMgr);
            try {
                m_frames->ReadCPACS(tixiHandle, xpath + "/frames");
            } catch(const std::exception& e) {
                LOG(ERROR) << "Failed to read frames at xpath " << xpath << ": " << e.what();
                m_frames = boost::none;
            }
        }

        // read element virtualStringers
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/virtualStringers")) {
            m_virtualStringers = boost::in_place(reinterpret_cast<CCPACSHullStructure*>(this), m_uidMgr);
            try {
                m_virtualStringers->ReadCPACS(tixiHandle, xpath + "/virtualStringers");
            } catch(const std::exception& e) {
                LOG(ERROR) << "Failed to read virtualStringers at xpath " << xpath << ": " << e.what();
                m_virtualStringers = boost::none;
            }
        }

        // read element virtualFrames
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/virtualFrames")) {
            m_virtualFrames = boost::in_place(reinterpret_cast<CCPACSHullStructure*>(this), m_uidMgr);
            try {
                m_virtualFrames->ReadCPACS(tixiHandle, xpath + "/virtualFrames");
            } catch(const std::exception& e) {
                LOG(ERROR) << "Failed to read virtualFrames at xpath " << xpath << ": " << e.what();
                m_virtualFrames = boost::none;
            }
        }

        // read element skinLayers
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/skinLayers")) {
            m_skinLayers = boost::in_place(reinterpret_cast<CCPACSHullStructure*>(this), m_uidMgr);
            try {
                m_skinLayers->ReadCPACS(tixiHandle, xpath + "/skinLayers");
            } catch(const std::exception& e) {
                LOG(ERROR) << "Failed to read skinLayers at xpath " << xpath << ": " << e.what();
                m_skinLayers = boost::none;
            }
        }

        // read element walls
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/walls")) {
            m_walls = boost::in_place(reinterpret_cast<CCPACSHullStructure*>(this), m_uidMgr);
            try {
                m_walls->ReadCPACS(tixiHandle, xpath + "/walls");
            } catch(const std::exception& e) {
                LOG(ERROR) << "Failed to read walls at xpath " << xpath << ": " << e.what();
                m_walls = boost::none;
            }
        }

        if (m_uidMgr && !m_uID.empty()) m_uidMgr->RegisterObject(m_uID, *this);
    }

    void CPACSHullStructure::WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const
    {
        // write attribute uID
        tixi::TixiSaveAttribute(tixiHandle, xpath, "uID", m_uID);

        // write element stringers
        if (m_stringers) {
            tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/stringers");
            m_stringers->WriteCPACS(tixiHandle, xpath + "/stringers");
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/stringers")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/stringers");
            }
        }

        // write element frames
        if (m_frames) {
            tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/frames");
            m_frames->WriteCPACS(tixiHandle, xpath + "/frames");
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/frames")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/frames");
            }
        }

        // write element virtualStringers
        if (m_virtualStringers) {
            tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/virtualStringers");
            m_virtualStringers->WriteCPACS(tixiHandle, xpath + "/virtualStringers");
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/virtualStringers")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/virtualStringers");
            }
        }

        // write element virtualFrames
        if (m_virtualFrames) {
            tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/virtualFrames");
            m_virtualFrames->WriteCPACS(tixiHandle, xpath + "/virtualFrames");
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/virtualFrames")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/virtualFrames");
            }
        }

        // write element skinLayers
        if (m_skinLayers) {
            tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/skinLayers");
            m_skinLayers->WriteCPACS(tixiHandle, xpath + "/skinLayers");
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/skinLayers")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/skinLayers");
            }
        }

        // write element walls
        if (m_walls) {
            tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/walls");
            m_walls->WriteCPACS(tixiHandle, xpath + "/walls");
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/walls")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/walls");
            }
        }

    }

    const std::string& CPACSHullStructure::GetUID() const
    {
        return m_uID;
    }

    void CPACSHullStructure::SetUID(const std::string& value)
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

    const boost::optional<CCPACSStringersAssembly>& CPACSHullStructure::GetStringers() const
    {
        return m_stringers;
    }

    boost::optional<CCPACSStringersAssembly>& CPACSHullStructure::GetStringers()
    {
        return m_stringers;
    }

    const boost::optional<CCPACSFramesAssembly>& CPACSHullStructure::GetFrames() const
    {
        return m_frames;
    }

    boost::optional<CCPACSFramesAssembly>& CPACSHullStructure::GetFrames()
    {
        return m_frames;
    }

    const boost::optional<CCPACSStringersAssembly>& CPACSHullStructure::GetVirtualStringers() const
    {
        return m_virtualStringers;
    }

    boost::optional<CCPACSStringersAssembly>& CPACSHullStructure::GetVirtualStringers()
    {
        return m_virtualStringers;
    }

    const boost::optional<CCPACSFramesAssembly>& CPACSHullStructure::GetVirtualFrames() const
    {
        return m_virtualFrames;
    }

    boost::optional<CCPACSFramesAssembly>& CPACSHullStructure::GetVirtualFrames()
    {
        return m_virtualFrames;
    }

    const boost::optional<CPACSHullSkinLayers>& CPACSHullStructure::GetSkinLayers() const
    {
        return m_skinLayers;
    }

    boost::optional<CPACSHullSkinLayers>& CPACSHullStructure::GetSkinLayers()
    {
        return m_skinLayers;
    }

    const boost::optional<CCPACSWalls>& CPACSHullStructure::GetWalls() const
    {
        return m_walls;
    }

    boost::optional<CCPACSWalls>& CPACSHullStructure::GetWalls()
    {
        return m_walls;
    }

    CCPACSStringersAssembly& CPACSHullStructure::GetStringers(CreateIfNotExistsTag)
    {
        if (!m_stringers)
            m_stringers = boost::in_place(reinterpret_cast<CCPACSHullStructure*>(this), m_uidMgr);
        return *m_stringers;
    }

    void CPACSHullStructure::RemoveStringers()
    {
        m_stringers = boost::none;
    }

    CCPACSFramesAssembly& CPACSHullStructure::GetFrames(CreateIfNotExistsTag)
    {
        if (!m_frames)
            m_frames = boost::in_place(reinterpret_cast<CCPACSHullStructure*>(this), m_uidMgr);
        return *m_frames;
    }

    void CPACSHullStructure::RemoveFrames()
    {
        m_frames = boost::none;
    }

    CCPACSStringersAssembly& CPACSHullStructure::GetVirtualStringers(CreateIfNotExistsTag)
    {
        if (!m_virtualStringers)
            m_virtualStringers = boost::in_place(reinterpret_cast<CCPACSHullStructure*>(this), m_uidMgr);
        return *m_virtualStringers;
    }

    void CPACSHullStructure::RemoveVirtualStringers()
    {
        m_virtualStringers = boost::none;
    }

    CCPACSFramesAssembly& CPACSHullStructure::GetVirtualFrames(CreateIfNotExistsTag)
    {
        if (!m_virtualFrames)
            m_virtualFrames = boost::in_place(reinterpret_cast<CCPACSHullStructure*>(this), m_uidMgr);
        return *m_virtualFrames;
    }

    void CPACSHullStructure::RemoveVirtualFrames()
    {
        m_virtualFrames = boost::none;
    }

    CPACSHullSkinLayers& CPACSHullStructure::GetSkinLayers(CreateIfNotExistsTag)
    {
        if (!m_skinLayers)
            m_skinLayers = boost::in_place(reinterpret_cast<CCPACSHullStructure*>(this), m_uidMgr);
        return *m_skinLayers;
    }

    void CPACSHullStructure::RemoveSkinLayers()
    {
        m_skinLayers = boost::none;
    }

    CCPACSWalls& CPACSHullStructure::GetWalls(CreateIfNotExistsTag)
    {
        if (!m_walls)
            m_walls = boost::in_place(reinterpret_cast<CCPACSHullStructure*>(this), m_uidMgr);
        return *m_walls;
    }

    void CPACSHullStructure::RemoveWalls()
    {
        m_walls = boost::none;
    }

} // namespace generated
} // namespace tigl
