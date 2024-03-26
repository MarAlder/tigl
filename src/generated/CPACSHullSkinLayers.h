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

#pragma once

#include <string>
#include <tixi.h>
#include <vector>
#include "tigl_internal.h"
#include "UniquePtr.h"

namespace tigl
{
class CTiglUIDManager;
class CTiglUIDObject;
class CCPACSSkin;
class CCPACSHullStructure;

namespace generated
{
    // This class is used in:
    // CPACSHullStructure

    /// @brief Skin Layers
    /// 
    /// 
    /// 
    class CPACSHullSkinLayers
    {
    public:
        TIGL_EXPORT CPACSHullSkinLayers(CCPACSHullStructure* parent, CTiglUIDManager* uidMgr);

        TIGL_EXPORT virtual ~CPACSHullSkinLayers();

        TIGL_EXPORT CCPACSHullStructure* GetParent();

        TIGL_EXPORT const CCPACSHullStructure* GetParent() const;

        TIGL_EXPORT virtual CTiglUIDObject* GetNextUIDParent();
        TIGL_EXPORT virtual const CTiglUIDObject* GetNextUIDParent() const;

        TIGL_EXPORT CTiglUIDManager& GetUIDManager();
        TIGL_EXPORT const CTiglUIDManager& GetUIDManager() const;

        TIGL_EXPORT virtual void ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath);
        TIGL_EXPORT virtual void WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const;

        TIGL_EXPORT virtual const std::vector<std::unique_ptr<CCPACSSkin>>& GetSkinLayers() const;
        TIGL_EXPORT virtual std::vector<std::unique_ptr<CCPACSSkin>>& GetSkinLayers();

        TIGL_EXPORT virtual CCPACSSkin& AddSkinLayer();
        TIGL_EXPORT virtual void RemoveSkinLayer(CCPACSSkin& ref);

    protected:
        CCPACSHullStructure* m_parent;

        CTiglUIDManager* m_uidMgr;

        std::vector<std::unique_ptr<CCPACSSkin>> m_skinLayers;

    private:
        CPACSHullSkinLayers(const CPACSHullSkinLayers&) = delete;
        CPACSHullSkinLayers& operator=(const CPACSHullSkinLayers&) = delete;

        CPACSHullSkinLayers(CPACSHullSkinLayers&&) = delete;
        CPACSHullSkinLayers& operator=(CPACSHullSkinLayers&&) = delete;
    };
} // namespace generated

// Aliases in tigl namespace
using CCPACSHullSkinLayers = generated::CPACSHullSkinLayers;
} // namespace tigl
