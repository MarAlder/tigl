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
    class CPACSElementGeometry;

    // This class is used in:
    // CPACSElementGeometry

    /// @brief Parallelepiped
    /// 
    /// The component coordinate system is located in the center of the cuboid.
    /// This means that "length" extends in the x-direction, "width" in the y-direction and "height" in the z-direction, half in the positive and half in the negative direction.
    /// 
    class CPACSParallelepiped
    {
    public:
        TIGL_EXPORT CPACSParallelepiped(CPACSElementGeometry* parent);

        TIGL_EXPORT virtual ~CPACSParallelepiped();

        TIGL_EXPORT CPACSElementGeometry* GetParent();

        TIGL_EXPORT const CPACSElementGeometry* GetParent() const;

        TIGL_EXPORT virtual CTiglUIDObject* GetNextUIDParent();
        TIGL_EXPORT virtual const CTiglUIDObject* GetNextUIDParent() const;

        TIGL_EXPORT virtual void ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath);
        TIGL_EXPORT virtual void WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const;

        TIGL_EXPORT virtual const double& GetLength() const;
        TIGL_EXPORT virtual void SetLength(const double& value);

        TIGL_EXPORT virtual const double& GetWidth() const;
        TIGL_EXPORT virtual void SetWidth(const double& value);

        TIGL_EXPORT virtual const double& GetHeight() const;
        TIGL_EXPORT virtual void SetHeight(const double& value);

        TIGL_EXPORT virtual const double& GetAlpha() const;
        TIGL_EXPORT virtual void SetAlpha(const double& value);

        TIGL_EXPORT virtual const double& GetBeta() const;
        TIGL_EXPORT virtual void SetBeta(const double& value);

        TIGL_EXPORT virtual const double& GetGamma() const;
        TIGL_EXPORT virtual void SetGamma(const double& value);

    protected:
        CPACSElementGeometry* m_parent;

        /// Length (in local x-direction) [m]
        double m_length;

        /// Width (in local y-direction) [m]
        double m_width;

        /// Height (in local z-direction) [m]
        double m_height;

        /// Angle between edges in width and height direction (default: 90deg) [deg]
        double m_alpha;

        /// Angle between edges in length and height direction (default: 90deg) [deg]
        double m_beta;

        /// Angle between edges in length and width direction (default: 90deg) [deg]
        double m_gamma;

    private:
        CPACSParallelepiped(const CPACSParallelepiped&) = delete;
        CPACSParallelepiped& operator=(const CPACSParallelepiped&) = delete;

        CPACSParallelepiped(CPACSParallelepiped&&) = delete;
        CPACSParallelepiped& operator=(CPACSParallelepiped&&) = delete;
    };
} // namespace generated

// Aliases in tigl namespace
using CCPACSParallelepiped = generated::CPACSParallelepiped;
using CCPACSElementGeometry = generated::CPACSElementGeometry;
} // namespace tigl