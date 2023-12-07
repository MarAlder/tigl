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

#include <boost/optional.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <string>
#include <tixi.h>
#include "CPACSGenericFuelTanks.h"
#include "CreateIfNotExists.h"
#include "tigl_internal.h"

namespace tigl
{
class CTiglUIDManager;
class CTiglUIDObject;
class CCPACSFuselage;

namespace generated
{
    // This class is used in:
    // CPACSFuselage

    /// @brief List of fuel tanks
    /// 
    /// 
    /// 
    class CPACSFuselageFuelTanks
    {
    public:
        TIGL_EXPORT CPACSFuselageFuelTanks(CCPACSFuselage* parent, CTiglUIDManager* uidMgr);

        TIGL_EXPORT virtual ~CPACSFuselageFuelTanks();

        TIGL_EXPORT CCPACSFuselage* GetParent();

        TIGL_EXPORT const CCPACSFuselage* GetParent() const;

        TIGL_EXPORT virtual CTiglUIDObject* GetNextUIDParent();
        TIGL_EXPORT virtual const CTiglUIDObject* GetNextUIDParent() const;

        TIGL_EXPORT CTiglUIDManager& GetUIDManager();
        TIGL_EXPORT const CTiglUIDManager& GetUIDManager() const;

        TIGL_EXPORT virtual void ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath);
        TIGL_EXPORT virtual void WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const;

        TIGL_EXPORT virtual const boost::optional<CPACSGenericFuelTanks>& GetGenericFuelTanks() const;
        TIGL_EXPORT virtual boost::optional<CPACSGenericFuelTanks>& GetGenericFuelTanks();

        TIGL_EXPORT virtual CPACSGenericFuelTanks& GetGenericFuelTanks(CreateIfNotExistsTag);
        TIGL_EXPORT virtual void RemoveGenericFuelTanks();

    protected:
        CCPACSFuselage* m_parent;

        CTiglUIDManager* m_uidMgr;

        boost::optional<CPACSGenericFuelTanks> m_genericFuelTanks;

    private:
        CPACSFuselageFuelTanks(const CPACSFuselageFuelTanks&) = delete;
        CPACSFuselageFuelTanks& operator=(const CPACSFuselageFuelTanks&) = delete;

        CPACSFuselageFuelTanks(CPACSFuselageFuelTanks&&) = delete;
        CPACSFuselageFuelTanks& operator=(CPACSFuselageFuelTanks&&) = delete;
    };
} // namespace generated

// Aliases in tigl namespace
using CCPACSFuselageFuelTanks = generated::CPACSFuselageFuelTanks;
} // namespace tigl
