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
#include "CPACSParallelepiped.h"

// #include "TopoDS_Compound.hxx"
// #include "TopoDS_Builder.hxx"
// #include "BRepBuilderAPI_Transform.hxx"

#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>

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
    

    if (auto& parallelepiped = m_vehicleElement.GetGeometry().GetParallelepiped_choice1()) {
        double abs_a = parallelepiped->GetA();
        double abs_b = parallelepiped->GetB();
        double abs_c = parallelepiped->GetC();

        double alphaRad = parallelepiped->GetAlpha() * M_PI / 180.0;
        double betaRad  = parallelepiped->GetBeta() * M_PI / 180.0;
        double gammaRad = parallelepiped->GetGamma() * M_PI / 180.0;

        // Basisvektoren a, b, c
        gp_Vec vec_a(abs_a, 0, 0);
        gp_Vec vec_b(abs_b * cos(gammaRad), abs_b * sin(gammaRad), 0);
        gp_Vec vec_c(abs_c * cos(betaRad), abs_c * (cos(alphaRad) - cos(betaRad) * cos(gammaRad)) / sin(gammaRad),
                     abs_c * sqrt(1 - pow(cos(betaRad), 2) -
                               pow((cos(alphaRad) - cos(betaRad) * cos(gammaRad)) / sin(gammaRad), 2)));

        // Ursprung
        gp_Pnt p0(0, 0, 0);

        // Eckpunkte
        gp_Pnt p1 = p0.Translated(vec_a);
        gp_Pnt p2 = p0.Translated(vec_b);
        gp_Pnt p3 = p0.Translated(vec_a + vec_b);
        gp_Pnt p4 = p0.Translated(vec_c);

        // Beispiel: Kante a + b + c als Shape erzeugen
        BRepBuilderAPI_MakePolygon poly;
        poly.Add(p0);
        poly.Add(p1);
        poly.Add(p3);
        poly.Add(p2);
        poly.Close();
        TopoDS_Face baseFace = BRepBuilderAPI_MakeFace(poly.Wire());

        // Extrudiere entlang Vektor c
        elementShape = BRepPrimAPI_MakePrism(baseFace, vec_c);


        //BRepPrimAPI_MakeBox box(gp_Pnt(0, 0, 0), length, width, height);
    }

    //auto frustum        = m_vehicleElement.GetGeometry().GetFrustum_choice2();
    //auto ellipsoid      = m_vehicleElement.GetGeometry().GetEllipsoid_choice3();


    //BRepPrimAPI_MakeSphere sph(0.1);
    //elementShape = sph.Shape();
    std::string loftName = m_vehicleElement.GetUID().c_str();
    PNamedShape loft(new CNamedShape(elementShape, loftName));

    if (m_transformation) {
        loft = m_transformation->Transform(loft);
    }

    return loft;
};

CTiglVehicleElementBuilder::operator PNamedShape()
{
    return BuildShape();
};

} // namespace tigl
