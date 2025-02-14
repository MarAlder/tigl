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
#include <CCPACSPoint.h>
#include <CCPACSPointAbsRel.h>
#include <string>
#include <tixi.h>
#include "CreateIfNotExists.h"
#include "CTiglUIDObject.h"
#include "tigl_internal.h"

namespace tigl
{
class CTiglUIDManager;

namespace generated
{
    class CPACSComponent;

    // This class is used in:
    // CPACSComponent

    /// @brief Transformation
    /// 
    /// Transformation type, containing a set of
    /// transformations. The order of the transformations is
    /// rotation -> translation, and they are executed in this
    /// order. Any of them can be omitted; it will be replaced by its
    /// defaults.
    /// 
    class CPACSTransformationRT : public CTiglOptUIDObject
    {
    public:
        TIGL_EXPORT CPACSTransformationRT(CPACSComponent* parent, CTiglUIDManager* uidMgr);

        TIGL_EXPORT virtual ~CPACSTransformationRT();

        TIGL_EXPORT CPACSComponent* GetParent();

        TIGL_EXPORT const CPACSComponent* GetParent() const;

        TIGL_EXPORT virtual CTiglUIDObject* GetNextUIDParent();
        TIGL_EXPORT virtual const CTiglUIDObject* GetNextUIDParent() const;

        TIGL_EXPORT CTiglUIDManager& GetUIDManager();
        TIGL_EXPORT const CTiglUIDManager& GetUIDManager() const;

        TIGL_EXPORT virtual void ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath);
        TIGL_EXPORT virtual void WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const;

        TIGL_EXPORT virtual const boost::optional<std::string>& GetUID() const;
        TIGL_EXPORT virtual void SetUID(const boost::optional<std::string>& value);

        TIGL_EXPORT virtual const boost::optional<CCPACSPoint>& GetRotation() const;
        TIGL_EXPORT virtual boost::optional<CCPACSPoint>& GetRotation();

        TIGL_EXPORT virtual const boost::optional<CCPACSPointAbsRel>& GetTranslation() const;
        TIGL_EXPORT virtual boost::optional<CCPACSPointAbsRel>& GetTranslation();

        TIGL_EXPORT virtual CCPACSPoint& GetRotation(CreateIfNotExistsTag);
        TIGL_EXPORT virtual void RemoveRotation();

        TIGL_EXPORT virtual CCPACSPointAbsRel& GetTranslation(CreateIfNotExistsTag);
        TIGL_EXPORT virtual void RemoveTranslation();

    protected:
        CPACSComponent* m_parent;

        CTiglUIDManager* m_uidMgr;

        boost::optional<std::string>       m_uID;

        /// Rotation data default: 0,0,0. The rotation
        /// angles are the three Euler angles to describe the orientation of
        /// the coordinate system. The order is always xyz in CPACS.
        /// Therefore the first rotation is around the x-axis, the second
        /// rotation is around the rotated y-axis (y') and the third
        /// rotation is around the two times rotated z-axis (z'').
        boost::optional<CCPACSPoint>       m_rotation;

        /// Translation data default: 0,0,0. Translations
        /// can either be made absolute in the global coordinate system
        /// (absGlobal) or absolute in the local Coordinate system (absLocal).
        boost::optional<CCPACSPointAbsRel> m_translation;

    private:
        CPACSTransformationRT(const CPACSTransformationRT&) = delete;
        CPACSTransformationRT& operator=(const CPACSTransformationRT&) = delete;

        CPACSTransformationRT(CPACSTransformationRT&&) = delete;
        CPACSTransformationRT& operator=(CPACSTransformationRT&&) = delete;
    };
} // namespace generated

// Aliases in tigl namespace
using CCPACSTransformationRT = generated::CPACSTransformationRT;
using CCPACSComponent = generated::CPACSComponent;
} // namespace tigl
