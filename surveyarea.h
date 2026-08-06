#ifndef SURVEYAREA_H
#define SURVEYAREA_H

#include <QList>
#include "pointdata.h"

class SurveyArea
{
public:
    // Конструктор
    SurveyArea();

    // =============== МЕТОДЫ ГЕНЕРАЦИИ ТОЧЕК ===============

    // Генерация точек для режима "Поверхность"
    // startLat, startLon - широта и долгота левого верхнего угла квадрата (градусы)
    // sideKm - ребро квадрата в километрах
    // numPoints - количество точек по горизонтали (общее количество = numPoints * numPoints)
    // maxPed - максимальное значение ПЕД в центре квадрата (мкЗв/ч)
    // bgPed - фоновое значение ПЕД (мкЗв/ч)
    // startHeight - начальная (минимальная) высота полёта дрона (метры)
    void generateSurfacePoints(double startLat, double startLon, double sideKm,
                               int numPoints, double maxPed, double bgPed,
                               double startHeight);

    // Генерация точек для режима "Облако"
    // Параметры поверхности те же, плюс:
    // cloudUpperLevel - верхняя граница облака (метры, относительно startHeight)
    // numLevels - количество вертикальных уровней (шагов подъёма)
    // cloudCoeff - коэффициент увеличения ПЕД в центре облака

    void generateCloudPoints(double startLat, double startLon, double sideKm,
                             int numPoints, double maxPed, double bgPed,
                             double startHeight, double cloudUpperLevel,
                             int numLevels, double cloudCoeff);

    // =============== МЕТОДЫ НАВИГАЦИИ ===============

    // Получить текущую точку (по текущим индексам узла и уровня)
    PointData getCurrentPoint() const;

    // Переход к следующему узлу по горизонтали (высота не меняется)
    // Возвращает true если переход выполнен, false если это была последняя точка
    bool moveForward();

    // Подняться на один уровень вверх (в текущем узле)
    // Возвращает true если подъём возможен, false если достигнут верхний уровень
    bool moveUp();

    // Опуститься на один уровень вниз (в текущем узле)
    // Возвращает true если спуск возможен, false если достигнут нижний уровень
    bool moveDown();

    // Сбросить высоту на минимальную (начальную) в текущем узле
    void resetToStartHeight();

    // Вернуться в самую первую точку (узел 0, уровень 0)
    void resetToStartPosition();

    // =============== МЕТОДЫ ПРОВЕРКИ ВОЗМОЖНОСТИ ДЕЙСТВИЙ ===============

    bool canMoveForward() const;   // есть ли следующий узел
    bool canMoveUp() const;        // можно ли подняться выше
    bool canMoveDown() const;      // можно ли опуститься ниже

    // =============== ГЕТТЕРЫ ДЛЯ ОТЛАДКИ ===============

    int getNumGridPoints() const;   // количество узлов по горизонтали
    int getNumLevels() const;       // количество вертикальных уровней
    int getCurrentGridIndex() const;
    int getCurrentLevelIndex() const;
    double getStartHeight() const;
    int getTotalPointsCount() const; // общее количество точек в списке

    // Преобразование километров в градусы широты
    double kmToLatitudeDegrees(double km);

    // Преобразование километров в градусы долготы с учётом широты
    double kmToLongitudeDegrees(double km, double latitude);



    void setPoints(const QList<PointData> &points, int numGridPoints, int numLevels, double startHeight);

    QList<PointData> getPoints() const { return m_points; }







private:
    // =============== ВСПОМОГАТЕЛЬНЫЕ МЕТОДЫ ===============

    // Рассчитать ПЕД по гауссовскому закону
    // distance - расстояние от эпицентра в километрах
    // maxPed - ПЕД в эпицентре
    // backgroundPed - фоновое ПЕД
    // maxDistance - максимальное расстояние, на котором ПЕД = фон (R/2)
    double calculateGaussianPed(double distance, double maxPed,
                                double backgroundPed, double maxDistance);

    // // Преобразование километров в градусы широты
    // double kmToLatitudeDegrees(double km);

    // // Преобразование километров в градусы долготы с учётом широты
    // double kmToLongitudeDegrees(double km, double latitude);

    // Вычисление индекса в плоском списке по индексам узла и уровня
    // Формула: gridIndex * numLevels + levelIndex
    int getPointIndex(int gridIndex, int levelIndex) const;

    // =============== ПОЛЯ КЛАССА ===============

    // Плоский список всех точек
    // Порядок: сначала все уровни первого узла (от уровня 0 до numLevels-1),
    // затем все уровни второго узла, и т.д.
    QList<PointData> m_points;

    int m_numGridPoints;    // количество узлов по горизонтали (N x N сетка)
    int m_numLevels;        // количество вертикальных уровней (одинаково для всех узлов)
    int m_currentGridIndex; // текущий индекс узла (0 ... m_numGridPoints-1)
    int m_currentLevelIndex;// текущий индекс уровня (0 ... m_numLevels-1)
    double m_startHeight;   // начальная (минимальная) высота в метрах
};

#endif // SURVEYAREA_H
