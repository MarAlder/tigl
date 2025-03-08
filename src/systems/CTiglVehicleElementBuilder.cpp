/*
 * Copyright (C) 2018 German Aerospace Center (DLR/SC)
 *
 * Created: 2018-11-16 Jan Kleinert <jan.kleinert@dlr.de>
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

#include "CTiglVehicleElementBuilder.h"
#include "UniquePtr.h"
#include "CNamedShape.h"
// #include "TopoDS_Compound.hxx"
// #include "TopoDS_Builder.hxx"
// #include "BRepBuilderAPI_Transform.hxx"

#include <BRepPrimAPI_MakeSphere.hxx>

namespace tigl
{

CTiglVehicleElementBuilder::CTiglVehicleElementBuilder(const CCPACSVehicleElementBase& vehicleElement)
    : m_vehicleElement(vehicleElement) {};

CTiglVehicleElementBuilder::CTiglVehicleElementBuilder(const CCPACSVehicleElementBase& vehicleElement,
                                                       const CTiglTransformation& transformation)
    : m_vehicleElement(vehicleElement)
    , m_transformation(&transformation) {};

PNamedShape CTiglVehicleElementBuilder::BuildShape()
{
    // Dummy implementation
    // ToDo: replace with actual implementation
    TopoDS_Shape elementShape;
    BRepPrimAPI_MakeSphere sph(0.1);
    elementShape = sph.Shape();

    // sysShape = GetTransformationMatrix().Transform(sysShape);
    std::string loftName = m_vehicleElement.GetUID().c_str();
    PNamedShape loft(new CNamedShape(elementShape, loftName));

    return loft;
};

CTiglVehicleElementBuilder::operator PNamedShape()
{
    return BuildShape();
};

} // namespace tigl
