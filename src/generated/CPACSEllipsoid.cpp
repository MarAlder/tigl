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
#include "CPACSEllipsoid.h"
#include "CTiglError.h"
#include "CTiglLogging.h"
#include "CTiglUIDObject.h"
#include "TixiHelper.h"

namespace tigl
{
namespace generated
{
    CPACSEllipsoid::CPACSEllipsoid(CPACSElementGeometry* parent)
        : m_radiusX(0)
    {
        //assert(parent != NULL);
        m_parent = parent;
    }

    CPACSEllipsoid::~CPACSEllipsoid()
    {
    }

    const CPACSElementGeometry* CPACSEllipsoid::GetParent() const
    {
        return m_parent;
    }

    CPACSElementGeometry* CPACSEllipsoid::GetParent()
    {
        return m_parent;
    }

    const CTiglUIDObject* CPACSEllipsoid::GetNextUIDParent() const
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    CTiglUIDObject* CPACSEllipsoid::GetNextUIDParent()
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    void CPACSEllipsoid::ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath)
    {
        // read element radiusX
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/radiusX")) {
            m_radiusX = tixi::TixiGetElement<double>(tixiHandle, xpath + "/radiusX");
        }
        else {
            LOG(ERROR) << "Required element radiusX is missing at xpath " << xpath;
        }

        // read element radiusY
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/radiusY")) {
            m_radiusY = tixi::TixiGetElement<double>(tixiHandle, xpath + "/radiusY");
        }

        // read element radiusZ
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/radiusZ")) {
            m_radiusZ = tixi::TixiGetElement<double>(tixiHandle, xpath + "/radiusZ");
        }

    }

    void CPACSEllipsoid::WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const
    {
        // write element radiusX
        tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/radiusX");
        tixi::TixiSaveElement(tixiHandle, xpath + "/radiusX", m_radiusX);

        // write element radiusY
        if (m_radiusY) {
            tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/radiusY");
            tixi::TixiSaveElement(tixiHandle, xpath + "/radiusY", *m_radiusY);
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/radiusY")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/radiusY");
            }
        }

        // write element radiusZ
        if (m_radiusZ) {
            tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/radiusZ");
            tixi::TixiSaveElement(tixiHandle, xpath + "/radiusZ", *m_radiusZ);
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/radiusZ")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/radiusZ");
            }
        }

    }

    const double& CPACSEllipsoid::GetRadiusX() const
    {
        return m_radiusX;
    }

    void CPACSEllipsoid::SetRadiusX(const double& value)
    {
        m_radiusX = value;
    }

    const boost::optional<double>& CPACSEllipsoid::GetRadiusY() const
    {
        return m_radiusY;
    }

    void CPACSEllipsoid::SetRadiusY(const boost::optional<double>& value)
    {
        m_radiusY = value;
    }

    const boost::optional<double>& CPACSEllipsoid::GetRadiusZ() const
    {
        return m_radiusZ;
    }

    void CPACSEllipsoid::SetRadiusZ(const boost::optional<double>& value)
    {
        m_radiusZ = value;
    }

} // namespace generated
} // namespace tigl