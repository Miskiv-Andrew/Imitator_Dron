#ifndef POINTDATA_H
#define POINTDATA_H

class PointData
{
public:
    // Конструкторы
    PointData();
    PointData(double latitude, double longitude, double height,
              double pedLower, double pedUpper);

    // Геттеры
    double getLatitude() const;
    double getLongitude() const;
    double getHeight() const;
    double getPedLower() const;
    double getPedUpper() const;

    // Сеттеры
    void setLatitude(double latitude);
    void setLongitude(double longitude);
    void setHeight(double height);
    void setPedLower(double pedLower);
    void setPedUpper(double pedUpper);

private:
    double m_latitude;
    double m_longitude;
    double m_height;
    double m_pedLower;
    double m_pedUpper;
};

#endif // POINTDATA_H
