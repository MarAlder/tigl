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
#include "CCPACSFrustum.h"
#include "CCPACSEllipsoid.h"

#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_GTransform.hxx>

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
    const auto& geom = m_vehicleElement.GetGeometry();
    TopoDS_Shape elementShape;

    if (auto& p = geom.GetParallelepiped_choice1()) {
        elementShape = BuildParallelepipedShape(*p);
    }
    else if (auto& f = geom.GetFrustum_choice2()) {
        elementShape = BuildFrustumShape(*f);
    }
    else if (auto& e = geom.GetEllipsoid_choice3()) {
        elementShape = BuildEllipsoidShape(*e);
    }
    else {
        throw CTiglError("Unsupported geometry type");
    }

    std::string loftName = m_vehicleElement.GetUID().c_str();
    PNamedShape loft(new CNamedShape(elementShape, loftName));

    if (m_transformation) {
        loft = m_transformation->Transform(loft);
    }

    return loft;
};

TopoDS_Shape CTiglVehicleElementBuilder::BuildParallelepipedShape(const CCPACSParallelepiped& p)
{
    const double a = p.GetA();
    const double b = p.GetB();
    const double c = p.GetC();

    const double alpha = p.getAlpha() * M_PI / 180.0;
    const double beta  = p.getBeta() * M_PI / 180.0;
    const double gamma = p.getGamma() * M_PI / 180.0;

    gp_Vec vA{a, 0, 0};
    gp_Vec vB{b * std::cos(gamma), b * std::sin(gamma), 0};
    const double zC =
        c * std::sqrt(1 - std::pow(std::cos(beta), 2) -
                      std::pow((std::cos(alpha) - std::cos(beta) * std::cos(gamma)) / std::sin(gamma), 2));
    gp_Vec vC{c * std::cos(beta), c * (std::cos(alpha) - std::cos(beta) * std::cos(gamma)) / std::sin(gamma), zC};

    BRepBuilderAPI_MakePolygon poly;
    const gp_Pnt p0{0, 0, 0};
    poly.Add(p0);
    poly.Add(p0.Translated(vA));
    poly.Add(p0.Translated(vA).Translated(vB));
    poly.Add(p0.Translated(vB));
    poly.Close();
    TopoDS_Face base = BRepBuilderAPI_MakeFace(poly.Wire());

    TopoDS_Shape parallelepiped = BRepPrimAPI_MakePrism(base, vC).Shape();

    // Shift the shape so its centroid is at the origin
    gp_Vec center = (vA + vB + vC) * 0.5;
    gp_Trsf tr;
    tr.SetTranslation(gp_Vec(-center.X(), -center.Y(), -center.Z()));
    TopoDS_Shape centered_parallelepiped = BRepBuilderAPI_Transform(parallelepiped, tr, true).Shape();

    return centered_parallelepiped;
}

TopoDS_Shape CTiglVehicleElementBuilder::BuildFrustumShape(const CCPACSFrustum& f)
{
    const double lowerRadiusX = f.GetLowerRadiusX();
    const double lowerRadiusY = f.getLowerRadiusY();
    const double upperRadiusX = f.getUpperRadiusX();
    const double upperRadiusY = f.getUpperRadiusY();
    double height             = f.GetHeight();

    TopoDS_Shape frustum = BRepPrimAPI_MakeCylinder(lowerRadiusX, height);

    gp_Trsf tr;
    tr.SetTranslation(gp_Vec(0, 0, -height * 0.5));
    TopoDS_Shape centered_frustum = BRepBuilderAPI_Transform(frustum, tr, true).Shape();

    return centered_frustum;
}

TopoDS_Shape CTiglVehicleElementBuilder::BuildEllipsoidShape(const CCPACSEllipsoid& e)
{
    double radiusX = e.GetRadiusX();
    double radiusY = e.getRadiusY();
    double radiusZ = e.getRadiusZ();

    TopoDS_Shape sphere = BRepPrimAPI_MakeSphere(1.0).Shape();

    gp_Mat M(radiusX, 0.0, 0.0, 0.0, radiusY, 0.0, 0.0, 0.0, radiusZ);
    gp_GTrsf gtrsf(M, gp_XYZ(0.0, 0.0, 0.0));
    BRepBuilderAPI_GTransform transformer(sphere, gtrsf, true);
    return transformer.Shape();
}

CTiglVehicleElementBuilder::operator PNamedShape()
{
    return BuildShape();
};

} // namespace tigl
