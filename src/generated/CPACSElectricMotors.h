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
#include <vector>
#include "tigl_internal.h"
#include "UniquePtr.h"

namespace tigl
{
class CTiglUIDManager;
class CTiglUIDObject;

namespace generated
{
    class CPACSElectricMotor;
    class CPACSSystemElements;

    // This class is used in:
    // CPACSSystemElements

    /// @brief Electric motors
    /// 
    /// 
    class CPACSElectricMotors
    {
    public:
        TIGL_EXPORT CPACSElectricMotors(CPACSSystemElements* parent, CTiglUIDManager* uidMgr);

        TIGL_EXPORT virtual ~CPACSElectricMotors();

        TIGL_EXPORT CPACSSystemElements* GetParent();

        TIGL_EXPORT const CPACSSystemElements* GetParent() const;

        TIGL_EXPORT virtual CTiglUIDObject* GetNextUIDParent();
        TIGL_EXPORT virtual const CTiglUIDObject* GetNextUIDParent() const;

        TIGL_EXPORT CTiglUIDManager& GetUIDManager();
        TIGL_EXPORT const CTiglUIDManager& GetUIDManager() const;

        TIGL_EXPORT virtual void ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath);
        TIGL_EXPORT virtual void WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const;

        TIGL_EXPORT virtual const std::vector<std::unique_ptr<CPACSElectricMotor>>& GetElectricMotors() const;
        TIGL_EXPORT virtual std::vector<std::unique_ptr<CPACSElectricMotor>>& GetElectricMotors();

        TIGL_EXPORT virtual CPACSElectricMotor& AddElectricMotor();
        TIGL_EXPORT virtual void RemoveElectricMotor(CPACSElectricMotor& ref);

    protected:
        CPACSSystemElements* m_parent;

        CTiglUIDManager* m_uidMgr;

        std::vector<std::unique_ptr<CPACSElectricMotor>> m_electricMotors;

    private:
        CPACSElectricMotors(const CPACSElectricMotors&) = delete;
        CPACSElectricMotors& operator=(const CPACSElectricMotors&) = delete;

        CPACSElectricMotors(CPACSElectricMotors&&) = delete;
        CPACSElectricMotors& operator=(CPACSElectricMotors&&) = delete;
    };
} // namespace generated

// Aliases in tigl namespace
using CCPACSElectricMotors = generated::CPACSElectricMotors;
using CCPACSElectricMotor = generated::CPACSElectricMotor;
using CCPACSSystemElements = generated::CPACSSystemElements;
} // namespace tigl
