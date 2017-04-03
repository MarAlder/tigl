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

#include <string>
#include <cctype>

#include "CTiglError.h"
#include "to_string.h"

namespace tigl
{
    namespace generated
    {
        // This enum is used in:
        // CPACSWingRibsPositioning_ribCrossingBehaviour
        
        // generated from /xsd:schema/xsd:complexType[916]/xsd:complexContent/xsd:extension/xsd:sequence/xsd:element[4]/xsd:complexType/xsd:simpleContent
        #ifdef HAVE_CPP11
        enum class CPACSRibCrossingBehaviour
        #else
        enum CPACSRibCrossingBehaviour
        #endif
        {
            cross,
            end
        };
        
        #ifdef HAVE_CPP11
        inline std::string CPACSRibCrossingBehaviourToString(const CPACSRibCrossingBehaviour& value)
        {
            switch(value) {
            case CPACSRibCrossingBehaviour::cross: return "cross";
            case CPACSRibCrossingBehaviour::end: return "end";
            default: throw CTiglError("Invalid enum value \"" + std_to_string(static_cast<int>(value)) + "\" for enum type CPACSRibCrossingBehaviour");
            }
        }
        inline CPACSRibCrossingBehaviour stringToCPACSRibCrossingBehaviour(const std::string& value)
        {
            auto toLower = [](std::string str) { for (char& c : str) { c = std::tolower(c); } return str; };
            if (toLower(value) == "cross") { return CPACSRibCrossingBehaviour::cross; }
            if (toLower(value) == "end") { return CPACSRibCrossingBehaviour::end; }
            throw CTiglError("Invalid string value \"" + value + "\" for enum type CPACSRibCrossingBehaviour");
        }
        #else
        inline std::string CPACSRibCrossingBehaviourToString(const CPACSRibCrossingBehaviour& value)
        {
            switch(value) {
            case cross: return "cross";
            case end: return "end";
            default: throw CTiglError("Invalid enum value \"" + std_to_string(static_cast<int>(value)) + "\" for enum type CPACSRibCrossingBehaviour");
            }
        }
        inline CPACSRibCrossingBehaviour stringToCPACSRibCrossingBehaviour(const std::string& value)
        {
            struct ToLower { std::string operator()(std::string str) { for (std::size_t i = 0; i < str.length(); i++) { str[i] = std::tolower(str[i]); } return str; } } toLower;
            if (toLower(value) == "cross") { return cross; }
            if (toLower(value) == "end") { return end; }
            throw CTiglError("Invalid string value \"" + value + "\" for enum type CPACSRibCrossingBehaviour");
        }
        #endif
    }
    
    // Aliases in tigl namespace
    #ifdef HAVE_CPP11
    using ECPACSRibCrossingBehaviour = generated::CPACSRibCrossingBehaviour;
    #else
    typedef generated::CPACSRibCrossingBehaviour ECPACSRibCrossingBehaviour;
    #endif
}
