/*
* Copyright (C) 2007-2013 German Aerospace Center (DLR/SC)
*
* Created: 2010-08-13 Markus Litz <Markus.Litz@dlr.de>
* Changed: $Id: TIGLScriptProxy.cpp 64 2012-10-23 23:54:09Z markus.litz $
*
* Version: $Revision: 64 $
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*   http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#define _USE_MATH_DEFINES

#include "TIGLScriptProxy.h"
#include "TIGLViewerDocument.h"
#include "TIGLScriptEngine.h"
#include "TIGLViewerWindow.h"
#include "CCPACSConfigurationManager.h"
#include "CNamedShape.h"

#include <QWidget>

#define qString2char(str) ((str).toStdString().c_str())

Q_DECLARE_METATYPE(TopoDS_Shape)

TIGLScriptProxy::TIGLScriptProxy(TIGLViewerWindow* app)
    : QObject(app)
{
    _app = app;
}

// Returns the CPACS configuration
tigl::CCPACSConfiguration& TIGLScriptProxy::GetConfiguration()
{
    tigl::CCPACSConfigurationManager& manager = tigl::CCPACSConfigurationManager::GetInstance();
    return manager.GetConfiguration(getTiglHandle());
}

TiglCPACSConfigurationHandle TIGLScriptProxy::getTiglHandle()
{
    if (!_app->getDocument()) {
        return -1;
    }
    else {
        return _app->getDocument()->getCpacsHandle();
    }
}

QScriptValue TIGLScriptProxy::getWingCount()
{
    int count = 0;
    TiglReturnCode ret = ::tiglGetWingCount(getTiglHandle(), &count);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return count;
    }
}

QString TIGLScriptProxy::getVersion()
{
    return(::tiglGetVersion());
}


QScriptValue TIGLScriptProxy::componentGetHashCode (QString componentUID)
{
    int hashCode;
    TiglReturnCode ret = ::tiglComponentGetHashCode (getTiglHandle(), qString2char(componentUID), &hashCode);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return hashCode;
    }
}

QScriptValue TIGLScriptProxy::exportComponent (QString uid, QString filename, double deflection)
{
    TiglReturnCode ret = ::tiglExportComponent(getTiglHandle(), qString2char(uid), qString2char(filename), deflection);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return QScriptValue::UndefinedValue;
    }
}

QScriptValue TIGLScriptProxy::exportConfiguration (QString filename, bool fuseAllShapes, double deflection)
{
    TiglReturnCode ret = ::tiglExportConfiguration(getTiglHandle(), qString2char(filename), fuseAllShapes ? TIGL_TRUE : TIGL_FALSE, deflection);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return QScriptValue::UndefinedValue;
    }
}

QScriptValue TIGLScriptProxy::exportFusedWingFuselageIGES (QString filenamePtr)
{
    TiglReturnCode ret = ::tiglExportFusedWingFuselageIGES(getTiglHandle(), qString2char(filenamePtr));
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return QScriptValue::UndefinedValue;
    }
}

QScriptValue TIGLScriptProxy::exportIGES (QString filenamePtr)
{
    TiglReturnCode ret = ::tiglExportIGES(getTiglHandle(), qString2char(filenamePtr));
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return QScriptValue::UndefinedValue;
    }
}

QScriptValue TIGLScriptProxy::exportSTEP (QString filenamePtr)
{
    TiglReturnCode ret = ::tiglExportSTEP(getTiglHandle(), qString2char(filenamePtr));
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return QScriptValue::UndefinedValue;
    }
}

QScriptValue TIGLScriptProxy::exportFusedSTEP (QString filenamePtr)
{
    TiglReturnCode ret = ::tiglExportFusedSTEP(getTiglHandle(), qString2char(filenamePtr));
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return QScriptValue::UndefinedValue;
    }
}

QScriptValue TIGLScriptProxy::exportMeshedFuselageSTL (int fuselageIndex, QString filename, double deflection)
{
    TiglReturnCode ret = ::tiglExportMeshedFuselageSTL(getTiglHandle(), fuselageIndex, qString2char(filename), deflection);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return QScriptValue::UndefinedValue;
    }
}

QScriptValue TIGLScriptProxy::exportMeshedFuselageVTKByIndex (int fuselageIndex, QString filenamePtr, double deflection)
{
    TiglReturnCode ret = ::tiglExportMeshedFuselageVTKByIndex(getTiglHandle(), fuselageIndex, qString2char(filenamePtr), deflection);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return QScriptValue::UndefinedValue;
    }
}

QScriptValue TIGLScriptProxy::exportMeshedFuselageVTKByUID (QString fuselageUID, QString filenamePtr, double deflection)
{
    TiglReturnCode ret = ::tiglExportMeshedFuselageVTKByUID(getTiglHandle(), qString2char(fuselageUID), qString2char(filenamePtr), deflection);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return QScriptValue::UndefinedValue;
    }
}

QScriptValue TIGLScriptProxy::setExportOptions(QString exporter, QString optionName, QString optionValue)
{
    TiglReturnCode ret = ::tiglSetExportOptions(qString2char(exporter), qString2char(optionName), qString2char(optionValue));
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return QScriptValue::UndefinedValue;
    }
}

QScriptValue TIGLScriptProxy::fuselageGetUID(int fuselageIndex)
{
    char* uid = NULL;
    TiglReturnCode ret = ::tiglFuselageGetUID(getTiglHandle(), fuselageIndex, &uid);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return uid;
    }
}

QScriptValue TIGLScriptProxy::fuselageGetCenterLineLength(QString fuselageUID)
{
    double length;
    TiglReturnCode ret = ::tiglFuselageGetCenterLineLength (getTiglHandle(), qString2char(fuselageUID), &length);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return length;
    }
}


QScriptValue TIGLScriptProxy::fuselageGetCircumference (int fuselageIndex, int segmentIndex, double eta)
{
    double circumference;
    TiglReturnCode ret = ::tiglFuselageGetCircumference (getTiglHandle(), fuselageIndex, segmentIndex, eta, &circumference);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return circumference;
    }
}

QScriptValue TIGLScriptProxy::fuselageGetPoint (int fuselageIndex, int segmentIndex, double eta, double zeta)
{
    double x,y,z;
    TiglReturnCode ret = ::tiglFuselageGetPoint (getTiglHandle(), fuselageIndex, segmentIndex, eta, zeta, &x, &y, &z);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        QScriptValue Point3dCtor = engine()->globalObject().property("Point3d");
        return Point3dCtor.construct(QScriptValueList() << x << y << z);
    }
}

QScriptValue TIGLScriptProxy::fuselageGetSegmentUID (int fuselageIndex, int segmentIndex)
{
    char *uidName;
    TiglReturnCode ret = ::tiglFuselageGetSegmentUID (getTiglHandle(), fuselageIndex, segmentIndex, &uidName);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return uidName;
    }
}

QScriptValue TIGLScriptProxy::fuselageGetSegmentVolume (int fuselageIndex, int segmentIndex)
{
    double volume;
    TiglReturnCode ret = ::tiglFuselageGetSegmentVolume (getTiglHandle(), fuselageIndex, segmentIndex, &volume);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return volume;
    }
}

QScriptValue TIGLScriptProxy::getFuselageCount()
{
    int count = 0;
    TiglReturnCode ret = ::tiglGetFuselageCount(getTiglHandle(), &count);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return count;
    }
}

QScriptValue TIGLScriptProxy::fuselageGetSegmentCount(int fuselageIndex)
{
    int count = 0;
    TiglReturnCode ret = ::tiglFuselageGetSegmentCount(getTiglHandle(), fuselageIndex, &count);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return count;
    }
}

QScriptValue TIGLScriptProxy::wingGetUID(int wingIndex)
{
    char* uid = NULL;
    TiglReturnCode ret = ::tiglWingGetUID(getTiglHandle(), wingIndex, &uid);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return uid;
    }
}


QScriptValue TIGLScriptProxy::wingGetUpperPoint(int wingIndex, int segmentIndex, double eta, double xsi)
{
    double px, py, pz;
    TiglReturnCode ret = ::tiglWingGetUpperPoint(getTiglHandle(), wingIndex, segmentIndex, eta, xsi, &px, &py, &pz);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        QScriptValue Point3dCtor = engine()->globalObject().property("Point3d");
        return Point3dCtor.construct(QScriptValueList() << px << py << pz);
    }
}

QScriptValue TIGLScriptProxy::wingGetLowerPoint(int wingIndex, int segmentIndex, double eta, double xsi)
{
    double px, py, pz;
    TiglReturnCode ret = ::tiglWingGetLowerPoint(getTiglHandle(), wingIndex, segmentIndex, eta, xsi, &px, &py, &pz);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        QScriptValue Point3dCtor = engine()->globalObject().property("Point3d");
        return Point3dCtor.construct(QScriptValueList() << px << py << pz);
    }
}

QScriptValue TIGLScriptProxy::wingGetChordPoint(int wingIndex, int segmentIndex, double eta, double xsi)
{
    double px, py, pz;
    TiglReturnCode ret = ::tiglWingGetChordPoint(getTiglHandle(), wingIndex, segmentIndex, eta, xsi, &px, &py, &pz);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        QScriptValue Point3dCtor = engine()->globalObject().property("Point3d");
        return Point3dCtor.construct(QScriptValueList() << px << py << pz);
    }
}

QScriptValue TIGLScriptProxy::wingGetChordNormal(int wingIndex, int segmentIndex, double eta, double xsi)
{
    double px, py, pz;
    TiglReturnCode ret = ::tiglWingGetChordNormal(getTiglHandle(), wingIndex, segmentIndex, eta, xsi, &px, &py, &pz);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        QScriptValue Point3dCtor = engine()->globalObject().property("Point3d");
        return Point3dCtor.construct(QScriptValueList() << px << py << pz);
    }
}

QScriptValue TIGLScriptProxy::wingComponentSegmentGetPoint(QString compSegUID, double eta, double xsi)
{
    double px, py, pz;
    TiglReturnCode ret = ::tiglWingComponentSegmentGetPoint(getTiglHandle(), qString2char(compSegUID), eta, xsi, &px, &py, &pz);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        QScriptValue Point3dCtor = engine()->globalObject().property("Point3d");
        return Point3dCtor.construct(QScriptValueList() << px << py << pz);
    }
}

QScriptValue TIGLScriptProxy::wingGetSegmentCount(int wingIndex)
{
    int count = 0;
    TiglReturnCode ret = ::tiglWingGetSegmentCount(getTiglHandle(), wingIndex, &count);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return count;
    }
}

QScriptValue TIGLScriptProxy::wingGetSegmentUID(int wingIndex, int segmentIndex)
{
    char *uidName;
    TiglReturnCode ret = ::tiglWingGetSegmentUID(getTiglHandle(), wingIndex, segmentIndex, &uidName);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return uidName;
    }
}

QString TIGLScriptProxy::getErrorString(int errorCode)
{
    return ::tiglGetErrorString((TiglReturnCode) errorCode);
}

QScriptValue TIGLScriptProxy::wingGetUpperPointAtDirection(int wingIndex, int segmentIndex, double eta, double xsi, double dirx, double diry, double dirz)
{
    
    double px, py, pz, distance;
    TiglReturnCode ret = ::tiglWingGetUpperPointAtDirection(getTiglHandle(), wingIndex, segmentIndex, eta, xsi, dirx, diry, dirz,&px, &py, &pz, &distance);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        QScriptValue Point3dCtor  = engine()->globalObject().property("Point3d");
        QScriptValue GPResultCtor = engine()->globalObject().property("GetPointDirectionResult");

        QScriptValue pnt = Point3dCtor.construct(QScriptValueList() << px << py << pz);
        QScriptValue result = GPResultCtor.construct(QScriptValueList() << pnt << distance);
        return result;
    }
}

QScriptValue TIGLScriptProxy::wingGetLowerPointAtDirection(int wingIndex, int segmentIndex, double eta, double xsi, double dirx, double diry, double dirz)
{
    
    double px, py, pz, distance;
    TiglReturnCode ret = ::tiglWingGetLowerPointAtDirection(getTiglHandle(), wingIndex, segmentIndex, eta, xsi, dirx, diry, dirz,&px, &py, &pz, &distance);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        QScriptValue Point3dCtor  = engine()->globalObject().property("Point3d");
        QScriptValue GPResultCtor = engine()->globalObject().property("GetPointDirectionResult");

        QScriptValue pnt = Point3dCtor.construct(QScriptValueList() << px << py << pz);
        QScriptValue result = GPResultCtor.construct(QScriptValueList() << pnt << distance);
        return result;
    }
}

QScriptValue TIGLScriptProxy::wingGetSpan(QString wingUID)
{
    double span = 0.;
    TiglReturnCode ret = ::tiglWingGetSpan(getTiglHandle(), qString2char(wingUID), &span);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return span;
    }
}

QScriptValue TIGLScriptProxy::wingGetSegmentVolume(int wingIndex, int segmentIndex)
{
    double volume;
    TiglReturnCode ret = ::tiglWingGetSegmentVolume(getTiglHandle(), wingIndex, segmentIndex, &volume);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return volume;
    }
}

QScriptValue TIGLScriptProxy::controlSurfaceSetControlParameter(QString controlSurfaceUID, double controlParameter)
{
    TiglReturnCode ret = ::tiglControlSurfaceSetControlParameter(getTiglHandle(), controlSurfaceUID.toStdString().c_str(), controlParameter);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return QScriptValue::UndefinedValue;
    }
}


QScriptValue TIGLScriptProxy::wingSetGetPointBehaviour (int behaviour)
{
    TiglGetPointBehavior b = onLinearLoft;
    if (behaviour == 0) {
        b = asParameterOnSurface;
    }
    else {
        b = onLinearLoft;
    }

    TiglReturnCode ret = ::tiglWingSetGetPointBehavior(getTiglHandle(), b);
    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        return QScriptValue::UndefinedValue;
    }
}

QScriptValue TIGLScriptProxy::wingGetSegmentEtaXsi(int wingIdx, double px, double py, double pz)
{
    double eta, xsi;
    int segmentIndex, isOnTop;

    TiglReturnCode ret = ::tiglWingGetSegmentEtaXsi(getTiglHandle(), wingIdx, px, py, pz, &segmentIndex, &eta, &xsi, &isOnTop);

    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        QScriptValue EtaXsiCtor = engine()->globalObject().property("EtaXsi");
        QScriptValue etaxsi = EtaXsiCtor.construct(QScriptValueList() << eta << xsi);

        QScriptValue obj = engine()->newObject();

        obj.setProperty("etaxsi", etaxsi);
        obj.setProperty("isOnTop", isOnTop);
        obj.setProperty("segmentIndex", segmentIndex);
        return obj;
    }

}

QScriptValue TIGLScriptProxy::componentTransformPointToGlobal(QString componentUID, double localX, double localY, double localZ)
{
    double gx, gy, gz;

    TiglReturnCode ret = ::tiglComponentTransformPointToGlobal(getTiglHandle(), componentUID.toStdString().c_str(),
                                                               localX, localY, localZ,
                                                               &gx, &gy, &gz);

    if (ret != TIGL_SUCCESS) {
        return context()->throwError(tiglGetErrorString(ret));
    }
    else {
        QScriptValue Point3dCtor = engine()->globalObject().property("Point3d");
        return Point3dCtor.construct(QScriptValueList() << gx << gy << gz);
    }
}


QScriptValue TIGLScriptProxy::getShape(QString uid)
{
    if (!_app->getDocument()) {
        return context()->throwError("No cpacs file opened.");
    }
    
    try {
        tigl::CCPACSConfiguration& config = _app->getDocument()->GetConfiguration();
        tigl::CTiglUIDManager& manager = config.GetUIDManager();
        if (manager.HasGeometricComponent(uid.toStdString()) ) {
            TopoDS_Shape shape = manager.GetGeometricComponent(uid.toStdString()).GetLoft()->Shape();
            return engine()->newVariant(QVariant::fromValue(shape));
        }
        else {
            return context()->throwError("No shape '" + uid + "'' on cpacs configuration.");
        }
    }
    catch(tigl::CTiglError& err) {
        TiglReturnCode errorCode = err.getCode();
        return context()->throwError(tiglGetErrorString(errorCode));
    }
    catch(...) {
        return context()->throwError("Unknown error");
    }
}
