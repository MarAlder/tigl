/*
 * Copyright (C) 2007-2022 German Aerospace Center (DLR/SC)
 *
 * Created: 2023-12-29 Marko Alder <marko.alder@dlr.de>
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
 * @brief Tests for testing duct functions.
 */

#include "test.h"
#include "tigl.h"

#include "CCPACSFuelTank.h"
#include "CCPACSFuelTanks.h"
#include "CCPACSConfigurationManager.h"
#include "CTiglUIDManager.h"

#include "CCPACSVessel.h"

// #include "generated/CPACSVessels.h"
// #include "generated/CPACSVessel.h"

// #include "CNamedShape.h"
// #include "tiglcommonfunctions.h"

// #include <Bnd_Box.hxx>
// #include <BRepBndLib.hxx>
// #include <gp_Pnt.hxx>

// #include <TopoDS_Shape.hxx>

// #include <TopTools_IndexedMapOfShape.hxx>
// #include <TopExp.hxx>

// #include <TopoDS_Iterator.hxx>

class FuelTanks : public ::testing::Test
{
protected:
    static void SetUpTestCase()
    {
        const char* filename = "TestData/simpletest-fuelTanks.cpacs.xml";
        ReturnCode tixiRet;
        TiglReturnCode tiglRet;

        tiglHandle = -1;
        tixiHandle = -1;

        tixiRet = tixiOpenDocument(filename, &tixiHandle);
        ASSERT_TRUE(tixiRet == SUCCESS);
        tiglRet = tiglOpenCPACSConfiguration(tixiHandle, "testAircraft", &tiglHandle);
        ASSERT_TRUE(tiglRet == TIGL_SUCCESS);
    }

    static void TearDownTestCase()
    {
        ASSERT_EQ(tiglCloseCPACSConfiguration(tiglHandle), TIGL_SUCCESS);
        ASSERT_EQ(tixiCloseDocument(tixiHandle), SUCCESS);
        tiglHandle = -1;
        tixiHandle = -1;
    }

    void SetUp() override
    {
    }
    void TearDown() override
    {
    }

    static TixiDocumentHandle tixiHandle;
    static TiglCPACSConfigurationHandle tiglHandle;

    tigl::CTiglUIDManager& uidMgr =
        tigl::CCPACSConfigurationManager::GetInstance().GetConfiguration(FuelTanks::tiglHandle).GetUIDManager();

    // tank
    tigl::CCPACSFuelTank const* fuelTank = &uidMgr.ResolveObject<tigl::CCPACSFuelTank>("tank1");

    // vessels
    const tigl::CCPACSVessels& vessels    = fuelTank->GetVessels();
    tigl::CCPACSVessel* vessel_segments   = &uidMgr.ResolveObject<tigl::CCPACSVessel>("tank1_outerVessel");
    tigl::CCPACSVessel* vessel_guides     = &uidMgr.ResolveObject<tigl::CCPACSVessel>("tank2_outerVessel");
    tigl::CCPACSVessel* vessel_parametric = &uidMgr.ResolveObject<tigl::CCPACSVessel>("tank3_sphericalDome");

    tigl::CCPACSVessel* vessel_spherical     = &uidMgr.ResolveObject<tigl::CCPACSVessel>("tank3_sphericalDome");
    tigl::CCPACSVessel* vessel_ellipsoid     = &uidMgr.ResolveObject<tigl::CCPACSVessel>("tank3_ellipsoidDome");
    tigl::CCPACSVessel* vessel_torispherical = &uidMgr.ResolveObject<tigl::CCPACSVessel>("tank4_torisphericalDome");
    tigl::CCPACSVessel* vessel_isotensoid    = &uidMgr.ResolveObject<tigl::CCPACSVessel>("tank5_isotensoidDome");

    tigl::CCPACSWallSegment* wall_in_fuselage = &uidMgr.ResolveObject<tigl::CCPACSWallSegment>("wall_fuselage");

    const char* tankTypeExceptionString = "This method is only available for vessels with segments. No segment found.";
};

TixiDocumentHandle FuelTanks::tixiHandle           = 0;
TiglCPACSConfigurationHandle FuelTanks::tiglHandle = 0;

void CheckExceptionMessage(std::function<void()> func, const char* expectedMessage)
{
    try {
        func();
        FAIL() << "Expected tigl::CTiglError but no exception was thrown.";
    }
    catch (const tigl::CTiglError& e) {
        EXPECT_STREQ(e.what(), expectedMessage);
    }
    catch (...) {
        FAIL() << "Expected tigl::CTiglError but a different exception was thrown.";
    }
}

TEST_F(FuelTanks, configuration)
{
    auto& config    = fuelTank->GetConfiguration();
    std::string uID = "tank1";
    EXPECT_EQ(config.GetFuelTanksCount(), 6);
    EXPECT_EQ(config.GetFuelTank(1).GetDefaultedUID(), uID);
    EXPECT_NO_THROW(config.GetFuelTank(uID));
    EXPECT_EQ(config.GetFuelTankIndex(uID), 1);
}

TEST_F(FuelTanks, fuelTanks)
{
    std::string uID                        = "tank1";
    const tigl::CCPACSFuelTanks* fuelTanks = fuelTank->GetParent();
    EXPECT_EQ(fuelTanks->GetFuelTank(1).GetDefaultedUID(), uID);
    EXPECT_NO_THROW(fuelTanks->GetFuelTank(uID));
    EXPECT_EQ(fuelTanks->GetFuelTankIndex(uID), 1);
    EXPECT_EQ(fuelTanks->GetFuelTanksCount(), 6);
}

TEST_F(FuelTanks, fuelTank)
{
    EXPECT_NO_THROW(fuelTank->GetVessels());

    const std::string name = fuelTank->GetName();
    EXPECT_EQ(name, "Simple tank 1");
}

// ToDo: Check how to use pointer ->
TEST_F(FuelTanks, vessels)
{
    EXPECT_EQ(vessels.GetVesselsCount(), 2);
    EXPECT_EQ(vessels.GetVessel(1).GetDefaultedUID(), "tank1_outerVessel");
    EXPECT_EQ(vessels.GetVessel("tank1_outerVessel").GetDefaultedUID(), "tank1_outerVessel");
    EXPECT_EQ(vessels.GetVesselIndex("tank1_outerVessel"), 1);
    EXPECT_EQ(vessels.GetVessels().at(0)->GetDefaultedUID(), "tank1_outerVessel");
}

TEST_F(FuelTanks, vessel_general)
{
    EXPECT_EQ(vessel_segments->GetConfiguration().GetUID(), "testAircraft");
    EXPECT_EQ(vessel_segments->GetDefaultedUID(), "tank1_outerVessel");
}

TEST_F(FuelTanks, vessel_component_info)
{
    EXPECT_EQ(vessel_segments->GetComponentType(), TIGL_COMPONENT_FUSELAGE_TANK_HULL);
    EXPECT_EQ(vessel_segments->GetComponentIntent(), TIGL_INTENT_PHYSICAL);
}

TEST_F(FuelTanks, vessel_type_info)
{
    EXPECT_TRUE(vessel_segments->IsVesselViaSegments());
    EXPECT_FALSE(vessel_segments->IsVesselViaDesignParameters());
    EXPECT_FALSE(vessel_spherical->IsVesselViaSegments());
    EXPECT_TRUE(vessel_spherical->IsVesselViaDesignParameters());

    EXPECT_FALSE(vessel_segments->HasSphericalDome());
    EXPECT_FALSE(vessel_segments->HasEllipsoidDome());
    EXPECT_FALSE(vessel_segments->HasTorisphericalDome());
    EXPECT_FALSE(vessel_segments->HasIsotensoidDome());

    EXPECT_TRUE(vessel_spherical->HasSphericalDome());
    EXPECT_TRUE(vessel_spherical->HasEllipsoidDome());
    EXPECT_FALSE(vessel_spherical->HasTorisphericalDome());
    EXPECT_FALSE(vessel_spherical->HasIsotensoidDome());

    EXPECT_FALSE(vessel_ellipsoid->HasSphericalDome());
    EXPECT_TRUE(vessel_ellipsoid->HasEllipsoidDome());
    EXPECT_FALSE(vessel_ellipsoid->HasTorisphericalDome());
    EXPECT_FALSE(vessel_ellipsoid->HasIsotensoidDome());

    EXPECT_FALSE(vessel_torispherical->HasSphericalDome());
    EXPECT_FALSE(vessel_torispherical->HasEllipsoidDome());
    EXPECT_TRUE(vessel_torispherical->HasTorisphericalDome());
    EXPECT_FALSE(vessel_torispherical->HasIsotensoidDome());

    EXPECT_FALSE(vessel_isotensoid->HasSphericalDome());
    EXPECT_FALSE(vessel_isotensoid->HasEllipsoidDome());
    EXPECT_FALSE(vessel_isotensoid->HasTorisphericalDome());
    EXPECT_TRUE(vessel_isotensoid->HasIsotensoidDome());
}

TEST_F(FuelTanks, vessel_sections)
{
    const char* invalidIndexMessage    = "Invalid index in CCPACSFuselageSections::GetSection";
    const char* wrongSectionUIDMessage = "GetSectionFace: Could not find a fuselage section for the given UID";

    EXPECT_EQ(vessel_segments->GetSectionCount(), 3);
    EXPECT_EQ(vessel_parametric->GetSectionCount(), 0);

    EXPECT_NO_THROW(vessel_segments->GetSection(1));
    CheckExceptionMessage([&]() { vessel_segments->GetSection(4); }, invalidIndexMessage);
    CheckExceptionMessage([&]() { vessel_parametric->GetSection(2); }, tankTypeExceptionString);

    EXPECT_NO_THROW(vessel_segments->GetSectionFace("outerVessel_section3"));
    CheckExceptionMessage([&]() { vessel_segments->GetSectionFace("wrongSectionUID"); }, wrongSectionUIDMessage);
    CheckExceptionMessage([&]() { vessel_parametric->GetSectionFace("outerVessel_section3"); }, tankTypeExceptionString);
}

TEST_F(FuelTanks, vessel_segments)
{
    EXPECT_EQ(vessel_segments->GetSegmentCount(), 2);
    EXPECT_EQ(vessel_parametric->GetSegmentCount(), 0);

    EXPECT_NO_THROW(vessel_segments->GetSegment(1));
    CheckExceptionMessage([&]() { vessel_parametric->GetSegment(1); }, tankTypeExceptionString);

    EXPECT_NO_THROW(vessel_segments->GetSegment("outerVessel_segment1"));
    CheckExceptionMessage([&]() { vessel_segments->GetSegment(3); },
                          "Invalid index value in CCPACSFuselageSegments::GetSegment");
}

TEST_F(FuelTanks, vessel_guide_curves)
{
    auto points = vessel_guides->GetGuideCurvePoints();
    EXPECT_EQ(points.size(), 24);
    EXPECT_NEAR(points.at(1).X(), 3.5, 1e-2);
    EXPECT_NEAR(points.at(1).Y(), 0, 1e-5);
    EXPECT_NEAR(points.at(1).Z(), -0.65, 1e-2);
    CheckExceptionMessage([&]() { vessel_parametric->GetGuideCurvePoints(); }, tankTypeExceptionString);

    EXPECT_EQ(vessel_guides->GetGuideCurveSegment("tank2_seg1_upper").GetGuideCurveProfileUID(), "gc_upper");
    CheckExceptionMessage([&]() { vessel_parametric->GetGuideCurveSegment("tank2_seg1_upper"); },
                          tankTypeExceptionString);
}

TEST_F(FuelTanks, vessel_loft_evaluation)
{
    EXPECT_NEAR(vessel_segments->GetGeometricVolume(), 6.57, 1e-2);
    EXPECT_NEAR(vessel_parametric->GetGeometricVolume(), 18.1, 1e-2);

    EXPECT_NEAR(vessel_segments->GetSurfaceArea(), 11.15, 1e-2);
    EXPECT_NEAR(vessel_parametric->GetSurfaceArea(), 36.19, 1e-2);

    EXPECT_NEAR(vessel_segments->GetCircumference(1, 0.5), 7.43, 1e-2);
    CheckExceptionMessage([&]() { vessel_parametric->GetCircumference(1, 0.5); }, tankTypeExceptionString);

    EXPECT_NEAR(vessel_segments->GetPoint(1, 0.5, 0.5).X(), 1.54, 1e-2);
    EXPECT_NEAR(vessel_segments->GetPoint(1, 0.5, 0.5).Y(), 0, 1e-5);
    EXPECT_NEAR(vessel_segments->GetPoint(1, 0.5, 0.5).Z(), -1.2, 1e-1);
    CheckExceptionMessage([&]() { vessel_parametric->GetPoint(1, 0.5, 0.5); }, tankTypeExceptionString);

    EXPECT_EQ(vessel_segments->GetGetPointBehavior(), asParameterOnSurface);
    EXPECT_NO_THROW(vessel_segments->SetGetPointBehavior(onLinearLoft));
    EXPECT_EQ(vessel_segments->GetGetPointBehavior(), onLinearLoft);
}

TEST_F(FuelTanks, parametric_vessel)
{
    auto& loft = vessel_parametric->GetLoft();
}

TEST_F(FuelTanks, structure)
{
    auto& structure = vessel_segments->GetStructure();

    EXPECT_EQ(structure->GetFrames()->GetFrames().size(), 1);
    EXPECT_EQ(structure->GetUID(), "outerVesselStructure");
}

TEST_F(FuelTanks, walls)
{
    EXPECT_EQ(wall_in_fuselage->GetUID().get(), "wall_fuselage");
    auto loft = wall_in_fuselage->GetLoft();
}