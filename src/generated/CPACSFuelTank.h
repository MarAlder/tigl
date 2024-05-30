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
#include <CCPACSHulls.h>
#include <CCPACSTransformation.h>
#include <string>
#include <TiglSymmetryAxis.h>
#include <tixi.h>
#include "CPACSFuelTankVolume.h"
#include "CreateIfNotExists.h"
#include "CTiglUIDObject.h"
#include "ITiglUIDRefObject.h"
#include "tigl_internal.h"

namespace tigl
{
class CTiglUIDManager;
class CCPACSFuelTanks;

namespace generated
{
    // This class is used in:
    // CPACSFuelTanks

    /// @brief Fuel tank
    /// 
    /// 
    class CPACSFuelTank : public CTiglReqUIDObject, public ITiglUIDRefObject
    {
    public:
        TIGL_EXPORT CPACSFuelTank(CCPACSFuelTanks* parent, CTiglUIDManager* uidMgr);

        TIGL_EXPORT virtual ~CPACSFuelTank();

        TIGL_EXPORT CCPACSFuelTanks* GetParent();

        TIGL_EXPORT const CCPACSFuelTanks* GetParent() const;

        TIGL_EXPORT virtual CTiglUIDObject* GetNextUIDParent();
        TIGL_EXPORT virtual const CTiglUIDObject* GetNextUIDParent() const;

        TIGL_EXPORT CTiglUIDManager& GetUIDManager();
        TIGL_EXPORT const CTiglUIDManager& GetUIDManager() const;

        TIGL_EXPORT virtual void ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath);
        TIGL_EXPORT virtual void WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const;

        TIGL_EXPORT virtual const std::string& GetUID() const;
        TIGL_EXPORT virtual void SetUID(const std::string& value);

        TIGL_EXPORT virtual const boost::optional<TiglSymmetryAxis>& GetSymmetry() const;
        TIGL_EXPORT virtual void SetSymmetry(const boost::optional<TiglSymmetryAxis>& value);

        TIGL_EXPORT virtual const std::string& GetName() const;
        TIGL_EXPORT virtual void SetName(const std::string& value);

        TIGL_EXPORT virtual const boost::optional<std::string>& GetDescription() const;
        TIGL_EXPORT virtual void SetDescription(const boost::optional<std::string>& value);

        TIGL_EXPORT virtual const boost::optional<std::string>& GetParentUID() const;
        TIGL_EXPORT virtual void SetParentUID(const boost::optional<std::string>& value);

        TIGL_EXPORT virtual const CCPACSTransformation& GetTransformation() const;
        TIGL_EXPORT virtual CCPACSTransformation& GetTransformation();

        TIGL_EXPORT virtual const CCPACSHulls& GetHulls() const;
        TIGL_EXPORT virtual CCPACSHulls& GetHulls();

        TIGL_EXPORT virtual const boost::optional<CPACSFuelTankVolume>& GetVolume() const;
        TIGL_EXPORT virtual boost::optional<CPACSFuelTankVolume>& GetVolume();

        TIGL_EXPORT virtual const boost::optional<double>& GetBurstPressure() const;
        TIGL_EXPORT virtual void SetBurstPressure(const boost::optional<double>& value);

        TIGL_EXPORT virtual CPACSFuelTankVolume& GetVolume(CreateIfNotExistsTag);
        TIGL_EXPORT virtual void RemoveVolume();

    protected:
        CCPACSFuelTanks* m_parent;

        CTiglUIDManager* m_uidMgr;

        std::string                          m_uID;

        boost::optional<TiglSymmetryAxis>    m_symmetry;

        /// Name
        std::string                          m_name;

        /// Description
        boost::optional<std::string>         m_description;

        /// Parent component
        boost::optional<std::string>         m_parentUID;

        CCPACSTransformation                 m_transformation;

        CCPACSHulls                          m_hulls;

        /// Volume
        boost::optional<CPACSFuelTankVolume> m_volume;

        /// Burst pressure
        boost::optional<double>              m_burstPressure;

    private:
        TIGL_EXPORT const CTiglUIDObject* GetNextUIDObject() const final;
        TIGL_EXPORT void NotifyUIDChange(const std::string& oldUid, const std::string& newUid) final;

        CPACSFuelTank(const CPACSFuelTank&) = delete;
        CPACSFuelTank& operator=(const CPACSFuelTank&) = delete;

        CPACSFuelTank(CPACSFuelTank&&) = delete;
        CPACSFuelTank& operator=(CPACSFuelTank&&) = delete;
    };
} // namespace generated

// CPACSFuelTank is customized, use type CCPACSFuelTank directly
} // namespace tigl