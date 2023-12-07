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
#include "ITiglUIDRefObject.h"
#include "tigl_internal.h"

namespace tigl
{
class CTiglUIDManager;
class CTiglUIDObject;
class CCPACSWingRibCrossSection;

namespace generated
{
    class CPACSCap;
    class CPACSGenericFuelTankParameters;
    class CPACSLateralCap;
    class CPACSSheetBasedStructuralElement;
    class CPACSTrackActuator;
    class CPACSWeb;
    class CPACSWingRibCell;
    class CPACSWingSkin;

    // This class is used in:
    // CPACSCap
    // CPACSGenericFuelTankParameters
    // CPACSLateralCap
    // CPACSSheetBasedStructuralElement
    // CPACSTrackActuator
    // CPACSWeb
    // CPACSWingRibCell
    // CPACSWingRibCrossSection
    // CPACSWingSkin

    /// @brief Material Definition
    /// 
    /// MaterialDefinition type, containing a material
    /// definition (Reference to material and thickness)
    /// 
    class CPACSMaterialDefinition : public ITiglUIDRefObject
    {
    public:
        TIGL_EXPORT CPACSMaterialDefinition(CPACSCap* parent, CTiglUIDManager* uidMgr);
        TIGL_EXPORT CPACSMaterialDefinition(CPACSGenericFuelTankParameters* parent, CTiglUIDManager* uidMgr);
        TIGL_EXPORT CPACSMaterialDefinition(CPACSLateralCap* parent, CTiglUIDManager* uidMgr);
        TIGL_EXPORT CPACSMaterialDefinition(CPACSSheetBasedStructuralElement* parent, CTiglUIDManager* uidMgr);
        TIGL_EXPORT CPACSMaterialDefinition(CPACSTrackActuator* parent, CTiglUIDManager* uidMgr);
        TIGL_EXPORT CPACSMaterialDefinition(CPACSWeb* parent, CTiglUIDManager* uidMgr);
        TIGL_EXPORT CPACSMaterialDefinition(CPACSWingRibCell* parent, CTiglUIDManager* uidMgr);
        TIGL_EXPORT CPACSMaterialDefinition(CCPACSWingRibCrossSection* parent, CTiglUIDManager* uidMgr);
        TIGL_EXPORT CPACSMaterialDefinition(CPACSWingSkin* parent, CTiglUIDManager* uidMgr);

        TIGL_EXPORT virtual ~CPACSMaterialDefinition();

        template<typename P>
        bool IsParent() const
        {
            return m_parentType != NULL && *m_parentType == typeid(P);
        }

        template<typename P>
        P* GetParent()
        {
            static_assert(std::is_same<P, CPACSCap>::value || std::is_same<P, CPACSGenericFuelTankParameters>::value || std::is_same<P, CPACSLateralCap>::value || std::is_same<P, CPACSSheetBasedStructuralElement>::value || std::is_same<P, CPACSTrackActuator>::value || std::is_same<P, CPACSWeb>::value || std::is_same<P, CPACSWingRibCell>::value || std::is_same<P, CCPACSWingRibCrossSection>::value || std::is_same<P, CPACSWingSkin>::value, "template argument for P is not a parent class of CPACSMaterialDefinition");
            if (!IsParent<P>()) {
                throw CTiglError("bad parent");
            }
            return static_cast<P*>(m_parent);
        }

        template<typename P>
        const P* GetParent() const
        {
            static_assert(std::is_same<P, CPACSCap>::value || std::is_same<P, CPACSGenericFuelTankParameters>::value || std::is_same<P, CPACSLateralCap>::value || std::is_same<P, CPACSSheetBasedStructuralElement>::value || std::is_same<P, CPACSTrackActuator>::value || std::is_same<P, CPACSWeb>::value || std::is_same<P, CPACSWingRibCell>::value || std::is_same<P, CCPACSWingRibCrossSection>::value || std::is_same<P, CPACSWingSkin>::value, "template argument for P is not a parent class of CPACSMaterialDefinition");
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

        TIGL_EXPORT bool ValidateChoices() const;

        TIGL_EXPORT virtual const boost::optional<std::string>& GetCompositeUID_choice1() const;
        TIGL_EXPORT virtual void SetCompositeUID_choice1(const boost::optional<std::string>& value);

        TIGL_EXPORT virtual const boost::optional<double>& GetOrthotropyDirection_choice1() const;
        TIGL_EXPORT virtual void SetOrthotropyDirection_choice1(const boost::optional<double>& value);

        TIGL_EXPORT virtual const boost::optional<double>& GetThicknessScaling_choice1() const;
        TIGL_EXPORT virtual void SetThicknessScaling_choice1(const boost::optional<double>& value);

        TIGL_EXPORT virtual const boost::optional<std::string>& GetMaterialUID_choice2() const;
        TIGL_EXPORT virtual void SetMaterialUID_choice2(const boost::optional<std::string>& value);

        TIGL_EXPORT virtual const boost::optional<double>& GetThickness_choice2() const;
        TIGL_EXPORT virtual void SetThickness_choice2(const boost::optional<double>& value);

    protected:
        void* m_parent;
        const std::type_info* m_parentType;

        CTiglUIDManager* m_uidMgr;

        /// uID of a composite definition.
        boost::optional<std::string> m_compositeUID_choice1;

        /// Orthotropy direction of the composite.
        boost::optional<double>      m_orthotropyDirection_choice1;

        /// Scaling factor of the composite thickness.
        /// Absolute thicknesses are defined in each composite material
        /// seperatly
        boost::optional<double>      m_thicknessScaling_choice1;

        /// uID of a material definition.
        boost::optional<std::string> m_materialUID_choice2;

        /// Absolute thickness of the material.
        boost::optional<double>      m_thickness_choice2;

    private:
        TIGL_EXPORT const CTiglUIDObject* GetNextUIDObject() const final;
        TIGL_EXPORT void NotifyUIDChange(const std::string& oldUid, const std::string& newUid) final;

        CPACSMaterialDefinition(const CPACSMaterialDefinition&) = delete;
        CPACSMaterialDefinition& operator=(const CPACSMaterialDefinition&) = delete;

        CPACSMaterialDefinition(CPACSMaterialDefinition&&) = delete;
        CPACSMaterialDefinition& operator=(CPACSMaterialDefinition&&) = delete;
    };
} // namespace generated

// CPACSMaterialDefinition is customized, use type CCPACSMaterialDefinition directly

// Aliases in tigl namespace
using CCPACSCap = generated::CPACSCap;
using CCPACSGenericFuelTankParameters = generated::CPACSGenericFuelTankParameters;
using CCPACSLateralCap = generated::CPACSLateralCap;
using CCPACSSheetBasedStructuralElement = generated::CPACSSheetBasedStructuralElement;
using CCPACSTrackActuator = generated::CPACSTrackActuator;
using CCPACSWeb = generated::CPACSWeb;
using CCPACSWingRibCell = generated::CPACSWingRibCell;
using CCPACSWingSkin = generated::CPACSWingSkin;
} // namespace tigl
