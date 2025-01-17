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
#include "tigl_internal.h"

namespace tigl
{
class CTiglUIDObject;

namespace generated
{
    class CPACSElementGeometry;

    // This class is used in:
    // CPACSElementGeometry

    /// @brief Ellipsoid
    /// 
    /// The local component coordinate system of the ellipsoid lies in its center.
    /// From there, the "radius" extends to the edge of the ellipsoid.
    /// 
    class CPACSEllipsoid
    {
    public:
        TIGL_EXPORT CPACSEllipsoid(CPACSElementGeometry* parent);

        TIGL_EXPORT virtual ~CPACSEllipsoid();

        TIGL_EXPORT CPACSElementGeometry* GetParent();

        TIGL_EXPORT const CPACSElementGeometry* GetParent() const;

        TIGL_EXPORT virtual CTiglUIDObject* GetNextUIDParent();
        TIGL_EXPORT virtual const CTiglUIDObject* GetNextUIDParent() const;

        TIGL_EXPORT virtual void ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath);
        TIGL_EXPORT virtual void WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const;

        TIGL_EXPORT virtual const double& GetRadiusX() const;
        TIGL_EXPORT virtual void SetRadiusX(const double& value);

        TIGL_EXPORT virtual const boost::optional<double>& GetRadiusY() const;
        TIGL_EXPORT virtual void SetRadiusY(const boost::optional<double>& value);

        TIGL_EXPORT virtual const boost::optional<double>& GetRadiusZ() const;
        TIGL_EXPORT virtual void SetRadiusZ(const boost::optional<double>& value);

    protected:
        CPACSElementGeometry* m_parent;

        /// Radius in x-direction [m]
        double                  m_radiusX;

        /// Radius in y-direction [m] (if not defined: equals radiusX)
        boost::optional<double> m_radiusY;

        /// Radius in z-direction [m] (if not defined: equals radiusX)
        boost::optional<double> m_radiusZ;

    private:
        CPACSEllipsoid(const CPACSEllipsoid&) = delete;
        CPACSEllipsoid& operator=(const CPACSEllipsoid&) = delete;

        CPACSEllipsoid(CPACSEllipsoid&&) = delete;
        CPACSEllipsoid& operator=(CPACSEllipsoid&&) = delete;
    };
} // namespace generated

// Aliases in tigl namespace
using CCPACSEllipsoid = generated::CPACSEllipsoid;
using CCPACSElementGeometry = generated::CPACSElementGeometry;
} // namespace tigl
