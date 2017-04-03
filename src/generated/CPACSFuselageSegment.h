// Copyright (c) 2016 RISC Software GmbH
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

#include <tixi.h>
#include <string>
#include <boost/optional.hpp>
#include <boost/utility/in_place_factory.hpp>
#include "tigl_internal.h"
#include <CCPACSGuideCurves.h>

namespace tigl
{
    class CCPACSFuselageSegments;
    
    namespace generated
    {
        // This class is used in:
        // CPACSFuselageSegments
        
        // generated from /xsd:schema/xsd:complexType[370]
        class CPACSFuselageSegment
        {
        public:
            TIGL_EXPORT CPACSFuselageSegment(CCPACSFuselageSegments* parent);
            
            TIGL_EXPORT virtual ~CPACSFuselageSegment();
            
            TIGL_EXPORT CCPACSFuselageSegments* GetParent() const;
            
            TIGL_EXPORT virtual void ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath);
            TIGL_EXPORT virtual void WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const;
            
            TIGL_EXPORT const std::string& GetUID() const;
            TIGL_EXPORT void SetUID(const std::string& value);
            
            TIGL_EXPORT const std::string& GetName() const;
            TIGL_EXPORT void SetName(const std::string& value);
            
            TIGL_EXPORT const boost::optional<std::string>& GetDescription() const;
            TIGL_EXPORT void SetDescription(const std::string& value);
            TIGL_EXPORT void SetDescription(const boost::optional<std::string>& value);
            
            TIGL_EXPORT const std::string& GetFromElementUID() const;
            TIGL_EXPORT void SetFromElementUID(const std::string& value);
            
            TIGL_EXPORT const std::string& GetToElementUID() const;
            TIGL_EXPORT void SetToElementUID(const std::string& value);
            
            TIGL_EXPORT const boost::optional<CCPACSGuideCurves>& GetGuideCurves() const;
            TIGL_EXPORT boost::optional<CCPACSGuideCurves>& GetGuideCurves();
            
        protected:
            CCPACSFuselageSegments* m_parent;
            
            std::string                        m_uID;
            std::string                        m_name;
            boost::optional<std::string>       m_description;
            std::string                        m_fromElementUID;
            std::string                        m_toElementUID;
            boost::optional<CCPACSGuideCurves> m_guideCurves;
            
        private:
            #ifdef HAVE_CPP11
            CPACSFuselageSegment(const CPACSFuselageSegment&) = delete;
            CPACSFuselageSegment& operator=(const CPACSFuselageSegment&) = delete;
            
            CPACSFuselageSegment(CPACSFuselageSegment&&) = delete;
            CPACSFuselageSegment& operator=(CPACSFuselageSegment&&) = delete;
            #else
            CPACSFuselageSegment(const CPACSFuselageSegment&);
            CPACSFuselageSegment& operator=(const CPACSFuselageSegment&);
            #endif
        };
    }
    
    // CPACSFuselageSegment is customized, use type CCPACSFuselageSegment directly
}
