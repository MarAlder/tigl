/*
* Copyright (C) 2015 German Aerospace Center (DLR/SC)
*
* Created: 2025-04-22 Marko Alder <Marko.Alder@dlr.de>
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include "CCPACSParallelepiped.h"
#include "tiglcommonfunctions.h"
#include "CTiglError.h"

namespace tigl
{

CCPACSParallelepiped::CCPACSParallelepiped(CCPACSElementGeometry* parent)
    : generated::CPACSParallelepiped(parent)
    , _default_alpha(90.0)
    , _default_beta(90.0)
    , _default_gamma(90.0)
{
}

double CCPACSParallelepiped::getAlpha() const
{
    return m_alpha.get_value_or(_default_alpha);
}

double CCPACSParallelepiped::getBeta() const
{
    return m_beta.get_value_or(_default_alpha);
}

double CCPACSParallelepiped::getGamma() const
{
    return m_gamma.get_value_or(_default_gamma);
}

} // namespace tigl
