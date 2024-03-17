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
class CCPACSGenericFuelTank;

namespace generated
{
    class CPACSFuselageFuelTanks;

    // This class is used in:
    // CPACSFuselageFuelTanks

    /// @brief Generic fuel tank
    /// 
    /// 
    /// 
    class CPACSGenericFuelTanks
    {
    public:
        TIGL_EXPORT CPACSGenericFuelTanks(CPACSFuselageFuelTanks* parent, CTiglUIDManager* uidMgr);

        TIGL_EXPORT virtual ~CPACSGenericFuelTanks();

        TIGL_EXPORT CPACSFuselageFuelTanks* GetParent();

        TIGL_EXPORT const CPACSFuselageFuelTanks* GetParent() const;

        TIGL_EXPORT virtual CTiglUIDObject* GetNextUIDParent();
        TIGL_EXPORT virtual const CTiglUIDObject* GetNextUIDParent() const;

        TIGL_EXPORT CTiglUIDManager& GetUIDManager();
        TIGL_EXPORT const CTiglUIDManager& GetUIDManager() const;

        TIGL_EXPORT virtual void ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath);
        TIGL_EXPORT virtual void WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const;

        TIGL_EXPORT virtual const std::vector<std::unique_ptr<CCPACSGenericFuelTank>>& GetGenericFuelTanks() const;
        TIGL_EXPORT virtual std::vector<std::unique_ptr<CCPACSGenericFuelTank>>& GetGenericFuelTanks();

        TIGL_EXPORT virtual CCPACSGenericFuelTank& AddGenericFuelTank();
        TIGL_EXPORT virtual void RemoveGenericFuelTank(CCPACSGenericFuelTank& ref);

    protected:
        CPACSFuselageFuelTanks* m_parent;

        CTiglUIDManager* m_uidMgr;

        std::vector<std::unique_ptr<CCPACSGenericFuelTank>> m_genericFuelTanks;

    private:
        CPACSGenericFuelTanks(const CPACSGenericFuelTanks&) = delete;
        CPACSGenericFuelTanks& operator=(const CPACSGenericFuelTanks&) = delete;

        CPACSGenericFuelTanks(CPACSGenericFuelTanks&&) = delete;
        CPACSGenericFuelTanks& operator=(CPACSGenericFuelTanks&&) = delete;
    };
} // namespace generated

// CPACSGenericFuelTanks is customized, use type CCPACSGenericFuelTanks directly

// Aliases in tigl namespace
using CCPACSFuselageFuelTanks = generated::CPACSFuselageFuelTanks;
} // namespace tigl
