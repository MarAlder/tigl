// Copyright (c) 2018 RISC Software GmbH
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

#ifndef CCPACSXSIISOLINE
#define CCPACSXSIISOLINE


#include "generated/CPACSXsiIsoLine.h"

namespace tigl
{
class CCPACSXsiIsoLine : public generated::CPACSXsiIsoLine
{
public:
    TIGL_EXPORT CCPACSXsiIsoLine(CCPACSControlSurfaceBorderTrailingEdge* parent);

    TIGL_EXPORT virtual void SetXsi(const double& value);
    TIGL_EXPORT virtual void SetReferenceUID(const std::string& value);

    //TIGL_EXPORT double ComputeCSOrTEDXsi() const;

private:
    void InvalidateParent();
};

} // namespace tigl

#endif // CCPACSXSIISOLINE
