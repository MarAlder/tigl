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
#include "CPACSParallelepiped.h"
#include "CTiglError.h"
#include "CTiglLogging.h"
#include "CTiglUIDObject.h"
#include "TixiHelper.h"

namespace tigl
{
namespace generated
{
    CPACSParallelepiped::CPACSParallelepiped(CPACSElementGeometry* parent)
        : m_length(0)
        , m_width(0)
        , m_height(0)
        , m_alpha(0)
        , m_beta(0)
        , m_gamma(0)
    {
        //assert(parent != NULL);
        m_parent = parent;
    }

    CPACSParallelepiped::~CPACSParallelepiped()
    {
    }

    const CPACSElementGeometry* CPACSParallelepiped::GetParent() const
    {
        return m_parent;
    }

    CPACSElementGeometry* CPACSParallelepiped::GetParent()
    {
        return m_parent;
    }

    const CTiglUIDObject* CPACSParallelepiped::GetNextUIDParent() const
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    CTiglUIDObject* CPACSParallelepiped::GetNextUIDParent()
    {
        if (m_parent) {
            return m_parent->GetNextUIDParent();
        }
        return nullptr;
    }

    void CPACSParallelepiped::ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath)
    {
        // read element length
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/length")) {
            m_length = tixi::TixiGetElement<double>(tixiHandle, xpath + "/length");
        }
        else {
            LOG(ERROR) << "Required element length is missing at xpath " << xpath;
        }

        // read element width
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/width")) {
            m_width = tixi::TixiGetElement<double>(tixiHandle, xpath + "/width");
        }
        else {
            LOG(ERROR) << "Required element width is missing at xpath " << xpath;
        }

        // read element height
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/height")) {
            m_height = tixi::TixiGetElement<double>(tixiHandle, xpath + "/height");
        }
        else {
            LOG(ERROR) << "Required element height is missing at xpath " << xpath;
        }

        // read element alpha
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/alpha")) {
            m_alpha = tixi::TixiGetElement<double>(tixiHandle, xpath + "/alpha");
        }
        else {
            LOG(ERROR) << "Required element alpha is missing at xpath " << xpath;
        }

        // read element beta
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/beta")) {
            m_beta = tixi::TixiGetElement<double>(tixiHandle, xpath + "/beta");
        }
        else {
            LOG(ERROR) << "Required element beta is missing at xpath " << xpath;
        }

        // read element gamma
        if (tixi::TixiCheckElement(tixiHandle, xpath + "/gamma")) {
            m_gamma = tixi::TixiGetElement<double>(tixiHandle, xpath + "/gamma");
        }
        else {
            LOG(ERROR) << "Required element gamma is missing at xpath " << xpath;
        }

    }

    void CPACSParallelepiped::WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const
    {
        const std::vector<std::string> childElemOrder = { "length", "width", "height", "alpha", "beta", "gamma" };

        // write element length
        tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/length", childElemOrder);
        tixi::TixiSaveElement(tixiHandle, xpath + "/length", m_length);

        // write element width
        tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/width", childElemOrder);
        tixi::TixiSaveElement(tixiHandle, xpath + "/width", m_width);

        // write element height
        tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/height", childElemOrder);
        tixi::TixiSaveElement(tixiHandle, xpath + "/height", m_height);

        // write element alpha
        tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/alpha", childElemOrder);
        tixi::TixiSaveElement(tixiHandle, xpath + "/alpha", m_alpha);

        // write element beta
        tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/beta", childElemOrder);
        tixi::TixiSaveElement(tixiHandle, xpath + "/beta", m_beta);

        // write element gamma
        tixi::TixiCreateSequenceElementIfNotExists(tixiHandle, xpath + "/gamma", childElemOrder);
        tixi::TixiSaveElement(tixiHandle, xpath + "/gamma", m_gamma);

    }

    const double& CPACSParallelepiped::GetLength() const
    {
        return m_length;
    }

    void CPACSParallelepiped::SetLength(const double& value)
    {
        m_length = value;
    }

    const double& CPACSParallelepiped::GetWidth() const
    {
        return m_width;
    }

    void CPACSParallelepiped::SetWidth(const double& value)
    {
        m_width = value;
    }

    const double& CPACSParallelepiped::GetHeight() const
    {
        return m_height;
    }

    void CPACSParallelepiped::SetHeight(const double& value)
    {
        m_height = value;
    }

    const double& CPACSParallelepiped::GetAlpha() const
    {
        return m_alpha;
    }

    void CPACSParallelepiped::SetAlpha(const double& value)
    {
        m_alpha = value;
    }

    const double& CPACSParallelepiped::GetBeta() const
    {
        return m_beta;
    }

    void CPACSParallelepiped::SetBeta(const double& value)
    {
        m_beta = value;
    }

    const double& CPACSParallelepiped::GetGamma() const
    {
        return m_gamma;
    }

    void CPACSParallelepiped::SetGamma(const double& value)
    {
        m_gamma = value;
    }

} // namespace generated
} // namespace tigl
