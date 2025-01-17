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
#include "CCPACSTransformation.h"
#include "CPACSBoundingBox.h"
#include "CPACSControlSurfaceHingePoint.h"
#include "CPACSControlSurfaceStep.h"
#include "CPACSDeckElementMass.h"
#include "CPACSElementMass.h"
#include "CPACSPoint.h"
#include "CPACSPointList.h"
#include "CTiglError.h"
#include "CTiglLogging.h"
#include "CTiglUIDManager.h"
#include "TixiHelper.h"

namespace tigl
{
namespace generated
{
    CPACSPoint::CPACSPoint(CPACSBoundingBox* parent, CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
    {
        //assert(parent != NULL);
        m_parent = parent;
        m_parentType = &typeid(CPACSBoundingBox);
    }

    CPACSPoint::CPACSPoint(CPACSControlSurfaceHingePoint* parent, CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
    {
        //assert(parent != NULL);
        m_parent = parent;
        m_parentType = &typeid(CPACSControlSurfaceHingePoint);
    }

    CPACSPoint::CPACSPoint(CPACSControlSurfaceStep* parent, CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
    {
        //assert(parent != NULL);
        m_parent = parent;
        m_parentType = &typeid(CPACSControlSurfaceStep);
    }

    CPACSPoint::CPACSPoint(CPACSDeckElementMass* parent, CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
    {
        //assert(parent != NULL);
        m_parent = parent;
        m_parentType = &typeid(CPACSDeckElementMass);
    }

    CPACSPoint::CPACSPoint(CPACSElementMass* parent, CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
    {
        //assert(parent != NULL);
        m_parent = parent;
        m_parentType = &typeid(CPACSElementMass);
    }

    CPACSPoint::CPACSPoint(CPACSPointList* parent, CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
    {
        //assert(parent != NULL);
        m_parent = parent;
        m_parentType = &typeid(CPACSPointList);
    }

    CPACSPoint::CPACSPoint(CCPACSTransformation* parent, CTiglUIDManager* uidMgr)
        : m_uidMgr(uidMgr)
    {
        //assert(parent != NULL);
        m_parent = parent;
        m_parentType = &typeid(CCPACSTransformation);
    }

    CPACSPoint::~CPACSPoint()
    {
        if (m_uidMgr && m_uID) m_uidMgr->TryUnregisterObject(*m_uID);
    }

    const CTiglUIDObject* CPACSPoint::GetNextUIDParent() const
    {
        if (m_parent) {
            if (IsParent<CPACSBoundingBox>()) {
                return GetParent<CPACSBoundingBox>()->GetNextUIDParent();
            }
            if (IsParent<CPACSControlSurfaceHingePoint>()) {
                return GetParent<CPACSControlSurfaceHingePoint>()->GetNextUIDParent();
            }
            if (IsParent<CPACSControlSurfaceStep>()) {
                return GetParent<CPACSControlSurfaceStep>()->GetNextUIDParent();
            }
            if (IsParent<CPACSDeckElementMass>()) {
                return GetParent<CPACSDeckElementMass>();
            }
            if (IsParent<CPACSElementMass>()) {
                if (GetParent<CPACSElementMass>()->GetUID())
                    return GetParent<CPACSElementMass>();
                else
                    return GetParent<CPACSElementMass>()->GetNextUIDParent();
            }
            if (IsParent<CPACSPointList>()) {
                return GetParent<CPACSPointList>()->GetNextUIDParent();
            }
            if (IsParent<CCPACSTransformation>()) {
                if (GetParent<CCPACSTransformation>()->GetUID())
                    return GetParent<CCPACSTransformation>();
                else
                    return GetParent<CCPACSTransformation>()->GetNextUIDParent();
            }
        }
        return nullptr;
    }

    CTiglUIDObject* CPACSPoint::GetNextUIDParent()
    {
        if (m_parent) {
            if (IsParent<CPACSBoundingBox>()) {
                return GetParent<CPACSBoundingBox>()->GetNextUIDParent();
            }
            if (IsParent<CPACSControlSurfaceHingePoint>()) {
                return GetParent<CPACSControlSurfaceHingePoint>()->GetNextUIDParent();
            }
            if (IsParent<CPACSControlSurfaceStep>()) {
                return GetParent<CPACSControlSurfaceStep>()->GetNextUIDParent();
            }
            if (IsParent<CPACSDeckElementMass>()) {
                return GetParent<CPACSDeckElementMass>();
            }
            if (IsParent<CPACSElementMass>()) {
                if (GetParent<CPACSElementMass>()->GetUID())
                    return GetParent<CPACSElementMass>();
                else
                    return GetParent<CPACSElementMass>()->GetNextUIDParent();
            }
            if (IsParent<CPACSPointList>()) {
                return GetParent<CPACSPointList>()->GetNextUIDParent();
            }
            if (IsParent<CCPACSTransformation>()) {
                if (GetParent<CCPACSTransformation>()->GetUID())
                    return GetParent<CCPACSTransformation>();
                else
                    return GetParent<CCPACSTransformation>()->GetNextUIDParent();
            }
        }
        return nullptr;
    }

    CTiglUIDManager& CPACSPoint::GetUIDManager()
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    const CTiglUIDManager& CPACSPoint::GetUIDManager() const
    {
        if (!m_uidMgr) {
            throw CTiglError("UIDManager is null");
        }
        return *m_uidMgr;
    }

    void CPACSPoint::ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath)
    {
        // read attribute uID
        if (tixi::TixiCheckAttribute(tixiHandle, xpath, "uID")) {
            m_uID = tixi::TixiGetAttribute<std::string>(tixiHandle, xpath, "uID");
            if (m_uID->empty()) {
                LOG(WARNING) << "Optional attribute uID is present but empty at xpath " << xpath;
            }
        }

        // read element x
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/x")) {
            m_x = tixi::TixiGetElement<double>(tixiHandle, xpath + "/x");
        }

        // read element y
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/y")) {
            m_y = tixi::TixiGetElement<double>(tixiHandle, xpath + "/y");
        }

        // read element z
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/z")) {
            m_z = tixi::TixiGetElement<double>(tixiHandle, xpath + "/z");
        }

        if (m_uidMgr && m_uID) m_uidMgr->RegisterObject(*m_uID, *this);
    }

    void CPACSPoint::WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const
    {
        // write attribute uID
        if (m_uID) {
            tixi::TixiSaveAttribute(tixiHandle, xpath, "uID", *m_uID);
        }
        else {
            if (tixi::TixiCheckAttribute(tixiHandle, xpath, "uID")) {
                tixi::TixiRemoveAttribute(tixiHandle, xpath, "uID");
            }
        }

        // write element x
        if (m_x) {
            tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/x");
            tixi::TixiSaveElement(tixiHandle, xpath + "/x", *m_x);
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/x")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/x");
            }
        }

        // write element y
        if (m_y) {
            tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/y");
            tixi::TixiSaveElement(tixiHandle, xpath + "/y", *m_y);
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/y")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/y");
            }
        }

        // write element z
        if (m_z) {
            tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/z");
            tixi::TixiSaveElement(tixiHandle, xpath + "/z", *m_z);
        }
        else {
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/z")) {
                tixi::TixiRemoveElement(tixiHandle, xpath + "/z");
            }
        }

    }

    const boost::optional<std::string>& CPACSPoint::GetUID() const
    {
        return m_uID;
    }

    void CPACSPoint::SetUID(const boost::optional<std::string>& value)
    {
        if (m_uidMgr && value != m_uID) {
            if (!m_uID && value) {
                m_uidMgr->RegisterObject(*value, *this);
            }
            else if (m_uID && !value) {
                m_uidMgr->TryUnregisterObject(*m_uID);
            }
            else if (m_uID && value) {
                m_uidMgr->UpdateObjectUID(*m_uID, *value);
            }
        }
        m_uID = value;
    }

    const boost::optional<double>& CPACSPoint::GetX() const
    {
        return m_x;
    }

    void CPACSPoint::SetX(const boost::optional<double>& value)
    {
        m_x = value;
    }

    const boost::optional<double>& CPACSPoint::GetY() const
    {
        return m_y;
    }

    void CPACSPoint::SetY(const boost::optional<double>& value)
    {
        m_y = value;
    }

    const boost::optional<double>& CPACSPoint::GetZ() const
    {
        return m_z;
    }

    void CPACSPoint::SetZ(const boost::optional<double>& value)
    {
        m_z = value;
    }

} // namespace generated
} // namespace tigl
