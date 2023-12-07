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
#include <CCPACSFuselageSections.h>
#include <CCPACSFuselageSegments.h>
#include <CCPACSTransformation.h>
#include <string>
#include <tixi.h>
#include "CPACSHullStructure.h"
#include "CreateIfNotExists.h"
#include "CTiglUIDObject.h"
#include "tigl_internal.h"

namespace tigl
{
class CTiglUIDManager;

namespace generated
{
    class CPACSHulls;

    // This class is used in:
    // CPACSHulls

    /// @brief Hulls
    /// 
    /// 
    class CPACSHull : public CTiglReqUIDObject
    {
    public:
        TIGL_EXPORT CPACSHull(CPACSHulls* parent, CTiglUIDManager* uidMgr);

        TIGL_EXPORT virtual ~CPACSHull();

        TIGL_EXPORT CPACSHulls* GetParent();

        TIGL_EXPORT const CPACSHulls* GetParent() const;

        TIGL_EXPORT virtual CTiglUIDObject* GetNextUIDParent();
        TIGL_EXPORT virtual const CTiglUIDObject* GetNextUIDParent() const;

        TIGL_EXPORT CTiglUIDManager& GetUIDManager();
        TIGL_EXPORT const CTiglUIDManager& GetUIDManager() const;

        TIGL_EXPORT virtual void ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath);
        TIGL_EXPORT virtual void WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const;

        TIGL_EXPORT virtual const std::string& GetUID() const;
        TIGL_EXPORT virtual void SetUID(const std::string& value);

        TIGL_EXPORT virtual const std::string& GetName() const;
        TIGL_EXPORT virtual void SetName(const std::string& value);

        TIGL_EXPORT virtual const boost::optional<std::string>& GetDescription() const;
        TIGL_EXPORT virtual void SetDescription(const boost::optional<std::string>& value);

        TIGL_EXPORT virtual const CCPACSTransformation& GetTransformation() const;
        TIGL_EXPORT virtual CCPACSTransformation& GetTransformation();

        TIGL_EXPORT virtual const CCPACSFuselageSections& GetSections() const;
        TIGL_EXPORT virtual CCPACSFuselageSections& GetSections();

        TIGL_EXPORT virtual const CCPACSFuselageSegments& GetSegments() const;
        TIGL_EXPORT virtual CCPACSFuselageSegments& GetSegments();

        TIGL_EXPORT virtual const boost::optional<CPACSHullStructure>& GetStructure() const;
        TIGL_EXPORT virtual boost::optional<CPACSHullStructure>& GetStructure();

        TIGL_EXPORT virtual CPACSHullStructure& GetStructure(CreateIfNotExistsTag);
        TIGL_EXPORT virtual void RemoveStructure();

    protected:
        CPACSHulls* m_parent;

        CTiglUIDManager* m_uidMgr;

        std::string                         m_uID;

        /// Name
        std::string                         m_name;

        /// Description
        boost::optional<std::string>        m_description;

        CCPACSTransformation                m_transformation;

        CCPACSFuselageSections              m_sections;

        CCPACSFuselageSegments              m_segments;

        boost::optional<CPACSHullStructure> m_structure;

    private:
        CPACSHull(const CPACSHull&) = delete;
        CPACSHull& operator=(const CPACSHull&) = delete;

        CPACSHull(CPACSHull&&) = delete;
        CPACSHull& operator=(CPACSHull&&) = delete;
    };
} // namespace generated

// Aliases in tigl namespace
using CCPACSHull = generated::CPACSHull;
using CCPACSHulls = generated::CPACSHulls;
} // namespace tigl
