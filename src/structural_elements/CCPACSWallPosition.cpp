/*
* Copyright (C) 2019 German Aerospace Center (DLR/SC)
*
* Created: 2019-10-16 Jan Kleinert <Jan.Kleinert@dlr.de>
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

#include "CNamedShape.h"
#include "CTiglUIDManager.h"
#include "CCPACSWallPosition.h"
#include "CCPACSWallSegment.h"
#include "CCPACSVessel.h"
#include "CCPACSVesselStructure.h"
#include "CCPACSWalls.h"
#include "CCPACSFuselageStructure.h"
#include "CCPACSFuselage.h"
#include "ITiglWallUtils.h"
#include "CTiglError.h"
#include "Bnd_Box.hxx"
#include "BRepBndLib.hxx"
#include "gp_Lin.hxx"
#include "IntCurvesFace_ShapeIntersector.hxx"
#include "Debugging.h"
#include <BRepBuilderAPI_MakeEdge.hxx>

namespace
{

// Computes the intersection point of a shape with a line
gp_Pnt GetIntersectionPoint(const TopoDS_Shape& shape, gp_Pnt basePoint, gp_Vec xDir, double bboxSize)
{
    gp_Pnt xyPoint = basePoint;
    gp_Lin xLine(xyPoint, xDir.XYZ());
    IntCurvesFace_ShapeIntersector intersector;
    intersector.Load(shape, 1e-5);
    // ToDo: not sure if I interpret PInf and PSup correctly:
    intersector.Perform(xLine, -bboxSize, bboxSize);
    int NbPnt = intersector.NbPnt();
    if (NbPnt == 1) {
        return intersector.Pnt(1);
    }
    else {
        DEBUG_SCOPE(onError);
        onError.addShape(shape, "shape");
        onError.addShape(BRepBuilderAPI_MakeEdge(xLine, -bboxSize, bboxSize), "line");

        throw tigl::CTiglError("Number of intersection points in GetIntersectionPoint is not 1. Instead it is " +
                               tigl::std_to_string(NbPnt));
    }
}

} // namespace

namespace tigl
{

CCPACSWallPosition::CCPACSWallPosition(CCPACSWallPositions* parent, CTiglUIDManager* uidMgr)
    : generated::CPACSWallPosition(parent, uidMgr)
{
}

void CCPACSWallPosition::CalcBasePointAndShape() const
{
    Bnd_Box boundingBox;
    TopoDS_Shape parentShape;
    CTiglTransformation transformationMatrix;

    bool isFuselage = GetParent()->GetParent()->IsParent<CCPACSFuselageStructure>();
    bool isVessel     = GetParent()->GetParent()->IsParent<CCPACSVesselStructure>();

    if (isFuselage) {
        const CCPACSFuselage& fuselage = GetFuselage();
        parentShape                    = fuselage.GetLoft()->Shape();
        transformationMatrix           = fuselage.GetTransformationMatrix();
    }
    else if (isVessel) {
        const CCPACSVessel& vessel = GetVessel();
        parentShape            = vessel.GetLoft()->Shape();
        transformationMatrix   = vessel.GetTransformationMatrix();
    }
    else {
        throw CTiglError("Parent of CCPACSWallPosition is neither a fuselage nor a vessel.");
    }

    BRepBndLib::Add(parentShape, boundingBox);
    double bboxSize = sqrt(boundingBox.SquareExtent());

    // Calculate base point and optional shape
    gp_Pnt basePointTmp(0., GetY(), GetZ());
    if (GetX_choice5()) {
        basePointTmp.SetX(GetX_choice5().value());
    }
    basePointTmp = transformationMatrix.Transform(basePointTmp);

    if (!GetX_choice5()) {
        if (GetBulkheadUID_choice1()) {
            _shape = GetUIDManager().GetGeometricComponent(GetBulkheadUID_choice1().value()).GetLoft()->Shape();
        }
        else if (GetWallSegmentUID_choice2()) {
            const CCPACSWallSegment& wall =
                GetUIDManager().ResolveObject<CCPACSWallSegment>(GetWallSegmentUID_choice2().value());
            if (wall.GetDefaultedUID() == GetWallSegmentUID_choice2().value()) {
                throw CTiglError("Fuselage wall references itself");
            }
            _shape = wall.GetLoft()->Shape();
        }
        else if (GetFuselageSectionUID_choice3()) {
            LOG(WARNING) << "DEPRECATED CPACS node: Use <sectionUID> instead.";
            _shape = GetFuselage().GetSectionFace(GetFuselageSectionUID_choice3().value());
        }
        else if (GetSectionUID_choice4()) {
            if (isFuselage) {
                _shape = GetFuselage().GetSectionFace(GetSectionUID_choice4().value());
            }
            else if (isVessel) {
                _shape = GetVessel().GetSectionFace(GetSectionUID_choice4().value());
            }
        }
        else {
            throw CTiglError("Cannot determine wall position.");
        }
        // Compute the base point by intersecting the shape with the line
        gp_Vec xDirGlobal = transformationMatrix.Transform(gp_Vec(1., 0., 0.));
        basePointTmp      = GetIntersectionPoint(*_shape, basePointTmp, xDirGlobal, bboxSize);
    }
    _basePoint = basePointTmp;
    _isBuilt   = true;
}

gp_Pnt& CCPACSWallPosition::GetBasePoint()
{
    return const_cast<gp_Pnt&>(const_cast<const CCPACSWallPosition*>(this)->GetBasePoint());
}

gp_Pnt const& CCPACSWallPosition::GetBasePoint() const
{
    if (!_isBuilt) {
        CalcBasePointAndShape();
    }
    return _basePoint;
}

boost::optional<TopoDS_Shape>& CCPACSWallPosition::GetShape()
{
    return const_cast<boost::optional<TopoDS_Shape>&>(const_cast<const CCPACSWallPosition*>(this)->GetShape());
}

boost::optional<TopoDS_Shape> const& CCPACSWallPosition::GetShape() const
{
    if (!_isBuilt) {
        CalcBasePointAndShape();
    }
    return _shape;
}

void CCPACSWallPosition::SetBulkheadUID_choice1(const boost::optional<std::string>& value)
{
    CPACSWallPosition::SetBulkheadUID_choice1(value);

    CPACSWallPosition::SetWallSegmentUID_choice2(boost::none);
    CPACSWallPosition::SetFuselageSectionUID_choice3(boost::none);
    CPACSWallPosition::SetSectionUID_choice4(boost::none);
    CPACSWallPosition::SetX_choice5(boost::none);

    Invalidate();
}

void CCPACSWallPosition::SetWallSegmentUID_choice2(const boost::optional<std::string>& value)
{
    CPACSWallPosition::SetWallSegmentUID_choice2(value);

    CPACSWallPosition::SetBulkheadUID_choice1(boost::none);
    CPACSWallPosition::SetFuselageSectionUID_choice3(boost::none);
    CPACSWallPosition::SetSectionUID_choice4(boost::none);
    CPACSWallPosition::SetX_choice5(boost::none);

    Invalidate();
}

void CCPACSWallPosition::SetFuselageSectionUID_choice3(const boost::optional<std::string>& value)
{
    CPACSWallPosition::SetFuselageSectionUID_choice3(value);

    CPACSWallPosition::SetBulkheadUID_choice1(boost::none);
    CPACSWallPosition::SetWallSegmentUID_choice2(boost::none);
    CPACSWallPosition::SetSectionUID_choice4(boost::none);
    CPACSWallPosition::SetX_choice5(boost::none);

    Invalidate();
}

void CCPACSWallPosition::SetSectionUID_choice4(const boost::optional<std::string>& value)
{
    CPACSWallPosition::SetSectionUID_choice4(value);

    CPACSWallPosition::SetBulkheadUID_choice1(boost::none);
    CPACSWallPosition::SetWallSegmentUID_choice2(boost::none);
    CPACSWallPosition::SetFuselageSectionUID_choice3(boost::none);
    CPACSWallPosition::SetX_choice5(boost::none);

    Invalidate();
}

void CCPACSWallPosition::SetX_choice5(const boost::optional<double>& value)
{
    CPACSWallPosition::SetX_choice5(value);

    CPACSWallPosition::SetBulkheadUID_choice1(boost::none);
    CPACSWallPosition::SetWallSegmentUID_choice2(boost::none);
    CPACSWallPosition::SetFuselageSectionUID_choice3(boost::none);
    CPACSWallPosition::SetSectionUID_choice4(boost::none);

    Invalidate();
}

void CCPACSWallPosition::SetY(const double& value)
{
    CPACSWallPosition::SetY(value);
    Invalidate();
}

void CCPACSWallPosition::SetZ(const double& value)
{
    CPACSWallPosition::SetZ(value);
    Invalidate();
}

void CCPACSWallPosition::InvalidateImpl(const boost::optional<std::string>& source) const
{
    _isBuilt = false;
    _shape   = boost::none;
}

const CCPACSWalls& CCPACSWallPosition::GetWalls() const
{
    return tigl::GetWalls(this);
}

const CCPACSFuselage& CCPACSWallPosition::GetFuselage() const
{
    return tigl::GetFuselage(this);
}

const CCPACSVessel& CCPACSWallPosition::GetVessel() const
{
    return tigl::GetVessel(this);
}

} // namespace tigl
