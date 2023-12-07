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
#include "tigl_internal.h"

namespace tigl
{
class CTiglUIDObject;

namespace generated
{
    class CPACSDomeType;

    // This class is used in:
    // CPACSDomeType

    /// @brief Isotensoid dome
    /// 
    /// 
    class CPACSIsotensoidDome
    {
    public:
        TIGL_EXPORT CPACSIsotensoidDome(CPACSDomeType* parent);

        TIGL_EXPORT virtual ~CPACSIsotensoidDome();

        TIGL_EXPORT CPACSDomeType* GetParent();

        TIGL_EXPORT const CPACSDomeType* GetParent() const;

        TIGL_EXPORT virtual CTiglUIDObject* GetNextUIDParent();
        TIGL_EXPORT virtual const CTiglUIDObject* GetNextUIDParent() const;

        TIGL_EXPORT virtual void ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath);
        TIGL_EXPORT virtual void WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const;

        TIGL_EXPORT virtual const double& GetPolarOpeningRadius() const;
        TIGL_EXPORT virtual void SetPolarOpeningRadius(const double& value);

    protected:
        CPACSDomeType* m_parent;

        /// Radius of the fitting/smaller polar opening
        double m_polarOpeningRadius;

    private:
        CPACSIsotensoidDome(const CPACSIsotensoidDome&) = delete;
        CPACSIsotensoidDome& operator=(const CPACSIsotensoidDome&) = delete;

        CPACSIsotensoidDome(CPACSIsotensoidDome&&) = delete;
        CPACSIsotensoidDome& operator=(CPACSIsotensoidDome&&) = delete;
    };
} // namespace generated

// Aliases in tigl namespace
using CCPACSIsotensoidDome = generated::CPACSIsotensoidDome;
using CCPACSDomeType = generated::CPACSDomeType;
} // namespace tigl
