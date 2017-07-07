/* 
* Copyright (C) 2014 Airbus Defense and Space
*
* Created: 2014-09-19 Roland Landertshamer <roland.landertshamer@risc-software.at>
* Changed: $Id: CCPACSModel.h 1434 2016-04-15 15:41:53Z rlandert $ 
*
* Version: $Revision: 1434 $
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
/**
* @file 
* @brief  Implementation of Model for use as root component in CTiglUIDManager
*/

#ifndef CCPACSMODEL_H
#define CCPACSMODEL_H

#include "CTiglAbstractPhysicalComponent.h"

namespace tigl
{

class CCPACSModel : public CTiglAbstractPhysicalComponent
{
public:
    // Construct
    TIGL_EXPORT CCPACSModel(void);

    // Virtual Destructor
    TIGL_EXPORT virtual ~CCPACSModel(void);

    // Returns the Geometric type of this component, e.g. Wing or Fuselage
    TIGL_EXPORT TiglGeometricComponentType GetComponentType(void);

protected:
    PNamedShape BuildLoft(void);
};

} // end namespace tigl

#endif // CCPACSMODEL_H