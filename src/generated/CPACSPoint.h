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
#include <typeinfo>
#include "CTiglError.h"
#include "CTiglUIDObject.h"
#include "tigl_internal.h"

namespace tigl
{
class CTiglUIDManager;
class CCPACSTransformation;

namespace generated
{
    class CPACSControlSurfaceStep;
    class CPACSPointList;
    class CPACSSeatModule;

    // This class is used in:
    // CPACSControlSurfaceStep
    // CPACSPointList
    // CPACSSeatModule
    // CPACSTransformation

    /// @brief pointType
    /// 
    /// Point type, containing an xyz data triplet.
    /// 
    class CPACSPoint : public CTiglOptUIDObject
    {
    public:
        TIGL_EXPORT CPACSPoint(CPACSControlSurfaceStep* parent, CTiglUIDManager* uidMgr);
        TIGL_EXPORT CPACSPoint(CPACSPointList* parent, CTiglUIDManager* uidMgr);
        TIGL_EXPORT CPACSPoint(CPACSSeatModule* parent, CTiglUIDManager* uidMgr);
        TIGL_EXPORT CPACSPoint(CCPACSTransformation* parent, CTiglUIDManager* uidMgr);

        TIGL_EXPORT virtual ~CPACSPoint();

        template<typename P>
        bool IsParent() const
        {
            return m_parentType != NULL && *m_parentType == typeid(P);
        }

        template<typename P>
        P* GetParent()
        {
#ifdef HAVE_STDIS_SAME
            static_assert(std::is_same<P, CPACSControlSurfaceStep>::value || std::is_same<P, CPACSPointList>::value || std::is_same<P, CPACSSeatModule>::value || std::is_same<P, CCPACSTransformation>::value, "template argument for P is not a parent class of CPACSPoint");
#endif
            if (!IsParent<P>()) {
                throw CTiglError("bad parent");
            }
            return static_cast<P*>(m_parent);
        }

        template<typename P>
        const P* GetParent() const
        {
#ifdef HAVE_STDIS_SAME
            static_assert(std::is_same<P, CPACSControlSurfaceStep>::value || std::is_same<P, CPACSPointList>::value || std::is_same<P, CPACSSeatModule>::value || std::is_same<P, CCPACSTransformation>::value, "template argument for P is not a parent class of CPACSPoint");
#endif
            if (!IsParent<P>()) {
                throw CTiglError("bad parent");
            }
            return static_cast<P*>(m_parent);
        }

        TIGL_EXPORT virtual CTiglUIDObject* GetNextUIDParent();
        TIGL_EXPORT virtual const CTiglUIDObject* GetNextUIDParent() const;

        TIGL_EXPORT CTiglUIDManager& GetUIDManager();
        TIGL_EXPORT const CTiglUIDManager& GetUIDManager() const;

        TIGL_EXPORT virtual void ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath);
        TIGL_EXPORT virtual void WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const;

        TIGL_EXPORT virtual const boost::optional<std::string>& GetUID() const;
        TIGL_EXPORT virtual void SetUID(const boost::optional<std::string>& value);

        TIGL_EXPORT virtual const boost::optional<double>& GetX() const;
        TIGL_EXPORT virtual void SetX(const boost::optional<double>& value);

        TIGL_EXPORT virtual const boost::optional<double>& GetY() const;
        TIGL_EXPORT virtual void SetY(const boost::optional<double>& value);

        TIGL_EXPORT virtual const boost::optional<double>& GetZ() const;
        TIGL_EXPORT virtual void SetZ(const boost::optional<double>& value);

    protected:
        void* m_parent;
        const std::type_info* m_parentType;

        CTiglUIDManager* m_uidMgr;

        boost::optional<std::string> m_uID;

        /// X-Component
        boost::optional<double>      m_x;

        /// Y-Component
        boost::optional<double>      m_y;

        /// Z-Component
        boost::optional<double>      m_z;

    private:
        CPACSPoint(const CPACSPoint&) = delete;
        CPACSPoint& operator=(const CPACSPoint&) = delete;

        CPACSPoint(CPACSPoint&&) = delete;
        CPACSPoint& operator=(CPACSPoint&&) = delete;
    };
} // namespace generated

// CPACSPoint is customized, use type CCPACSPoint directly

// Aliases in tigl namespace
using CCPACSControlSurfaceStep = generated::CPACSControlSurfaceStep;
using CCPACSPointList = generated::CPACSPointList;
using CCPACSSeatModule = generated::CPACSSeatModule;
} // namespace tigl
