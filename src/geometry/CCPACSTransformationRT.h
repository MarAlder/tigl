/*
* Copyright (C) 2015 German Aerospace Center (DLR/SC)
*
* Created: 2015-05-27 Martin Siggel <Martin.Siggel@dlr.de>
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

#ifndef CCPACSTRANSFORMATIONRT_H
#define CCPACSTRANSFORMATIONRT_H

#include "generated/CPACSTransformationRT.h"

#include "CTiglTransformation.h"
#include "ECPACSTranslationType.h"
#include "Cache.h"

namespace tigl
{


class CCPACSTransformationRT : public generated::CPACSTransformationRT
{
public:
    TIGL_EXPORT CCPACSTransformationRT(CCPACSComponent* parent, CTiglUIDManager* uidMgr);

private:

};

} // namespace tigl
#endif // CCPACSTRANSFORMATIONRT_H
