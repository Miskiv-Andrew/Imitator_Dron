#include "pointdata.h"

PointData::PointData()
    : m_latitude(0.0)
    , m_longitude(0.0)
    , m_height(0.0)
    , m_pedLower(0.0)
    , m_pedUpper(0.0)
{
}

PointData::PointData(double latitude, double longitude, double height,
                     double pedLower, double pedUpper)
    : m_latitude(latitude)
    , m_longitude(longitude)
    , m_height(height)
    , m_pedLower(pedLower)
    , m_pedUpper(pedUpper)
{
}

double PointData::getLatitude() const { return m_latitude; }
double PointData::getLongitude() const { return m_longitude; }
double PointData::getHeight() const { return m_height; }
double PointData::getPedLower() const { return m_pedLower; }
double PointData::getPedUpper() const { return m_pedUpper; }

void PointData::setLatitude(double latitude) { m_latitude = latitude; }
void PointData::setLongitude(double longitude) { m_longitude = longitude; }
void PointData::setHeight(double height) { m_height = height; }
void PointData::setPedLower(double pedLower) { m_pedLower = pedLower; }
void PointData::setPedUpper(double pedUpper) { m_pedUpper = pedUpper; }
