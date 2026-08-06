#include "surveyarea.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Конструктор: инициализация всех полей значениями по умолчанию
SurveyArea::SurveyArea()
    : m_numGridPoints(0)
    , m_numLevels(0)
    , m_currentGridIndex(0)
    , m_currentLevelIndex(0)
    , m_startHeight(0.0)
{
    m_points.clear();
}

// =============== ВСПОМОГАТЕЛЬНЫЕ МЕТОДЫ ===============

// Преобразование километров в градусы широты
// 1 градус широты ≈ 111.32 км
double SurveyArea::kmToLatitudeDegrees(double km)
{
    return km / 111.32;
}

// Преобразование километров в градусы долготы с учётом широты
// 1 градус долготы на данной широте ≈ 111.32 * cos(широта_в_радианах) км
double SurveyArea::kmToLongitudeDegrees(double km, double latitude)
{
    double latitudeRad = latitude * M_PI / 180.0;
    return km / (111.32 * cos(latitudeRad));
}

// =============== МЕТОДЫ НАВИГАЦИИ ===============

// Получить текущую точку (по текущим индексам узла и уровня)
// Возвращает копию объекта PointData
PointData SurveyArea::getCurrentPoint() const
{
    // Вычисляем индекс в плоском списке
    int index = getPointIndex(m_currentGridIndex, m_currentLevelIndex);
    // Проверяем, что индекс в пределах списка
    if (index >= 0 && index < m_points.size()) {
        return m_points[index];
    }
    // Если индекс некорректный, возвращаем пустую точку
    return PointData();
}

// Переход к следующему узлу по горизонтали (высота не меняется)
// Возвращает true если переход выполнен, false если это была последняя точка
bool SurveyArea::moveForward()
{
    // Проверяем, есть ли следующий узел
    if (!canMoveForward()) {
        return false;
    }
    // Переходим к следующему узлу (индекс узла увеличивается на 1)
    // Уровень высоты остаётся тем же
    m_currentGridIndex++;
    return true;
}

// Подняться на один уровень вверх (в текущем узле)
// Возвращает true если подъём возможен, false если достигнут верхний уровень
bool SurveyArea::moveUp()
{
    // Проверяем, можно ли подняться выше
    if (!canMoveUp()) {
        return false;
    }
    // Увеличиваем индекс уровня на 1
    m_currentLevelIndex++;
    return true;
}

// Опуститься на один уровень вниз (в текущем узле)
// Возвращает true если спуск возможен, false если достигнут нижний уровень
bool SurveyArea::moveDown()
{
    // Проверяем, можно ли опуститься ниже
    if (!canMoveDown()) {
        return false;
    }
    // Уменьшаем индекс уровня на 1
    m_currentLevelIndex--;
    return true;
}

// Сбросить высоту на минимальную (начальную) в текущем узле
// Устанавливает текущий уровень на 0 (самый нижний)
void SurveyArea::resetToStartHeight()
{
    m_currentLevelIndex = 0;
}

// Вернуться в самую первую точку (узел 0, уровень 0)
void SurveyArea::resetToStartPosition()
{
    m_currentGridIndex = 0;
    m_currentLevelIndex = 0;
}

// =============== МЕТОДЫ ПРОВЕРКИ ВОЗМОЖНОСТИ ДЕЙСТВИЙ ===============

// Проверка: есть ли следующий узел по горизонтали
bool SurveyArea::canMoveForward() const
{
    // Следующий узел существует, если текущий индекс узла + 1 меньше общего количества узлов
    return (m_currentGridIndex + 1 < m_numGridPoints);
}

// Проверка: можно ли подняться выше
bool SurveyArea::canMoveUp() const
{
    // Подъём возможен, если текущий индекс уровня + 1 меньше общего количества уровней
    return (m_currentLevelIndex + 1 < m_numLevels);
}

// Проверка: можно ли опуститься ниже
bool SurveyArea::canMoveDown() const
{
    // Спуск возможен, если текущий индекс уровня - 1 больше или равен 0
    return (m_currentLevelIndex - 1 >= 0);
}

// =============== ВСПОМОГАТЕЛЬНЫЕ МЕТОДЫ (дополнение) ===============

// Вычисление индекса в плоском списке по индексам узла и уровня
// Формула: gridIndex * numLevels + levelIndex
int SurveyArea::getPointIndex(int gridIndex, int levelIndex) const
{
    return gridIndex * m_numLevels + levelIndex;
}

// =============== ГЕТТЕРЫ ===============

int SurveyArea::getNumGridPoints() const { return m_numGridPoints; }
int SurveyArea::getNumLevels() const { return m_numLevels; }
int SurveyArea::getCurrentGridIndex() const { return m_currentGridIndex; }
int SurveyArea::getCurrentLevelIndex() const { return m_currentLevelIndex; }
double SurveyArea::getStartHeight() const { return m_startHeight; }
int SurveyArea::getTotalPointsCount() const { return m_points.size(); }

// =============== МЕТОДЫ ГЕНЕРАЦИИ  ===============

// =============== ГЕНЕРАЦИЯ ТОЧЕК ДЛЯ РЕЖИМА "ПОВЕРХНОСТЬ" ===============
//
// Входные параметры:
//   startLat, startLon - широта и долгота левого верхнего угла квадрата (градусы)
//   sideKm             - ребро квадрата в километрах
//   numPoints          - количество точек по горизонтали (общее количество = numPoints * numPoints)
//   maxPed             - максимальное значение ПЕД в эпицентре (центре квадрата) (мкЗв/ч)
//   bgPed              - фоновое значение ПЕД (мкЗв/ч)
//   startHeight        - начальная (минимальная) высота полёта дрона (метры)
//
// Алгоритм:
//   1. Очищаем список точек и устанавливаем базовые параметры
//   2. Вычисляем шаг сетки по широте и долготе (равномерное заполнение квадрата)
//   3. Находим координаты центра квадрата (эпицентра)
//   4. Вычисляем максимальное расстояние от центра до границы (sideKm / 2.0)
//   5. В двойном цикле по индексам i, j (от 0 до numPoints-1):
//      - Вычисляем широту и долготу текущей точки
//      - Вычисляем расстояние от точки до центра квадрата (в км)
//      - Рассчитываем pedLower по гауссовскому закону (pedUpper = bgPed)
//      - Создаём PointData с высотой startHeight и добавляем в список
//   6. Устанавливаем начальную позицию (узел 0, уровень 0)
//
void SurveyArea::generateSurfacePoints(double startLat, double startLon, double sideKm,
                                       int numPoints, double maxPed, double bgPed,
                                       double startHeight)
{
    // 1. Очищаем предыдущие данные
    m_points.clear();
    m_numGridPoints = numPoints * numPoints;   // общее количество узлов по горизонтали
    m_numLevels = 1;                           // для поверхности только один уровень
    m_startHeight = startHeight;

    // 2. Вычисляем шаг сетки в градусах
    //    Квадрат разбивается на (numPoints x numPoints) ячеек
    //    Шаг по широте: высота квадрата (sideKm км) переводится в градусы
    double stepLat = kmToLatitudeDegrees(sideKm) / (numPoints - 1);
    //    Шаг по долготе: ширина квадрата (sideKm км) переводится в градусы
    //    Используем широту левого верхнего угла для расчёта (приближение)
    double stepLon = kmToLongitudeDegrees(sideKm, startLat) / (numPoints - 1);

    // 3. Находим координаты центра квадрата (эпицентра)
    //    От левого верхнего угла отступаем половину ребра вниз и вправо
    double centerLat = startLat - kmToLatitudeDegrees(sideKm / 2.0);
    double centerLon = startLon + kmToLongitudeDegrees(sideKm / 2.0, startLat);

    // 4. Максимальное расстояние от центра до границы квадрата (в км)
    double maxDistance = sideKm / 2.0;

    // 5. Генерация точек в двойном цикле
    //    Внешний цикл - по широте (вертикаль, строки)
    //    Внутренний цикл - по долготе (горизонталь, столбцы)
    for (int i = 0; i < numPoints; ++i) {
        // Вычисляем широту текущей точки: от startLat (верхняя граница) вниз
        double lat = startLat - i * stepLat;

        for (int j = 0; j < numPoints; ++j) {
            // Вычисляем долготу текущей точки: от startLon (левая граница) вправо
            double lon = startLon + j * stepLon;

            // Вычисляем расстояние от текущей точки до центра квадрата (в км)
            // Сначала находим разницу в градусах
            double deltaLatDeg = lat - centerLat;
            double deltaLonDeg = lon - centerLon;
            // Переводим разницу в километры
            double deltaLatKm = deltaLatDeg * 111.32;
            double deltaLonKm = deltaLonDeg * (111.32 * cos(centerLat * M_PI / 180.0));
            // Евклидово расстояние
            double distance = sqrt(deltaLatKm * deltaLatKm + deltaLonKm * deltaLonKm);

            // Рассчитываем pedLower по гауссовскому закону
            double pedLower = calculateGaussianPed(distance, maxPed, bgPed, maxDistance);

            // Для режима "Поверхность" верхний детектор всегда показывает фон
            double pedUpper = bgPed;

            // Создаём точку и добавляем в список
            PointData point(lat, lon, startHeight, pedLower, pedUpper);
            m_points.append(point);
        }
    }

    // 6. Устанавливаем начальную позицию (первый узел, уровень 0)
    m_currentGridIndex = 0;
    m_currentLevelIndex = 0;
}


// =============== ГЕНЕРАЦИЯ ТОЧЕК ДЛЯ РЕЖИМА "ОБЛАКО" ===============
//
// Входные параметры:
//   startLat, startLon     - широта и долгота левого верхнего угла квадрата (градусы)
//   sideKm                 - ребро квадрата в километрах
//   numPoints              - количество точек по горизонтали (общее количество = numPoints * numPoints)
//   maxPed                 - максимальное значение ПЕД в эпицентре на нижнем уровне (мкЗв/ч)
//   bgPed                  - фоновое значение ПЕД (мкЗв/ч)
//   startHeight            - начальная (минимальная) высота полёта дрона (метры)
//   cloudLowerLevel        - нижняя граница облака (метры, относительно уровня земли)
//   cloudUpperLevel        - верхняя граница облака (метры, относительно уровня земли)
//   numLevels              - количество вертикальных уровней (шагов подъёма)
//   cloudCoeff             - коэффициент увеличения ПЕД в центре облака
//
// Алгоритм:
//   1. Очищаем список точек и устанавливаем базовые параметры
//   2. Вычисляем шаг сетки по широте и долготе (как в поверхности)
//   3. Находим координаты центра квадрата (эпицентра)
//   4. Вычисляем максимальное расстояние от центра до границы (sideKm / 2.0)
//   5. ПЕРВЫЙ ПРОХОД: создаём временный список basePedList для каждого узла
//      - Вычисляем координаты узла
//      - Вычисляем расстояние до центра квадрата
//      - Вычисляем basePed (ПЕД на нижнем уровне) по гауссу
//   6. ВТОРОЙ ПРОХОД: для каждого узла генерируем вертикальные уровни
//      - Если basePed <= bgPed (незаражённый узел):
//          Все уровни имеют pedLower = bgPed, pedUpper = bgPed
//      - Если basePed > bgPed (заражённый узел):
//          Для каждого уровня вычисляем высоту и коэффициент интерполяции
//          pedLower растёт от basePed до basePed * cloudCoeff, затем спадает до bgPed
//          pedUpper = pedLower следующего уровня (или bgPed для последнего)
//   7. Устанавливаем начальную позицию (узел 0, уровень 0)

void SurveyArea::generateCloudPoints(double startLat, double startLon, double sideKm,
                                     int numPoints, double maxPed, double bgPed,
                                     double startHeight, double cloudUpperLevel,
                                     int numLevels, double cloudCoeff)
{
    // 1. Очищаем предыдущие данные
    m_points.clear();
    m_numGridPoints = numPoints * numPoints;
    m_numLevels = numLevels;
    m_startHeight = startHeight;

    // 2. Вычисляем шаг сетки в градусах
    double stepLat = kmToLatitudeDegrees(sideKm) / (numPoints - 1);
    double stepLon = kmToLongitudeDegrees(sideKm, startLat) / (numPoints - 1);

    // 3. Находим координаты центра квадрата
    double centerLat = startLat - kmToLatitudeDegrees(sideKm / 2.0);
    double centerLon = startLon + kmToLongitudeDegrees(sideKm / 2.0, startLat);
    double maxDistance = sideKm / 2.0;

    // 4. Шаг по высоте между уровнями (от startHeight до cloudUpperLevel)
    double stepHeight = (cloudUpperLevel - startHeight) / (numLevels - 1);

    // 5. Генерация всех точек в тройном цикле
    for (int i = 0; i < numPoints; ++i) {
        double lat = startLat - i * stepLat;
        for (int j = 0; j < numPoints; ++j) {
            double lon = startLon + j * stepLon;

            // Расчёт расстояния до эпицентра
            double deltaLatDeg = lat - centerLat;
            double deltaLonDeg = lon - centerLon;
            double deltaLatKm = deltaLatDeg * 111.32;
            double deltaLonKm = deltaLonDeg * (111.32 * cos(centerLat * M_PI / 180.0));
            double distance = sqrt(deltaLatKm * deltaLatKm + deltaLonKm * deltaLonKm);

            // Базовое ПЕД на нижнем уровне (по гауссу)
            double basePed = calculateGaussianPed(distance, maxPed, bgPed, maxDistance);
            bool isInfected = (basePed > bgPed + 0.0001);

            // Генерация вертикальных уровней для текущего узла
            for (int level = 0; level < numLevels; ++level) {
                // Высота текущего уровня: от startHeight до cloudUpperLevel
                double height = startHeight + level * stepHeight;
                double pedLower, pedUpper;

                if (!isInfected) {
                    pedLower = bgPed;
                    pedUpper = bgPed;
                } else {
                    // Расчёт вертикального коэффициента (треугольное распределение)
                    double vertFactor = 0.0;
                    double currentHeightRel = height;  // относительная высота
                    double cloudCenter = (startHeight + cloudUpperLevel) / 2.0;
                    double halfThickness = (cloudUpperLevel - startHeight) / 2.0;

                    if (currentHeightRel <= startHeight) {
                        vertFactor = 0.0;
                    } else if (currentHeightRel >= cloudUpperLevel) {
                        vertFactor = 0.0;
                    } else {
                        double distToCenter = fabs(currentHeightRel - cloudCenter);
                        double normalizedDist = distToCenter / halfThickness;
                        vertFactor = 1.0 - normalizedDist;
                    }

                    double maxPedAtLevel = basePed * cloudCoeff;
                    pedLower = basePed + (maxPedAtLevel - basePed) * vertFactor;

                    if (level + 1 < numLevels) {
                        double nextHeight = startHeight + (level + 1) * stepHeight;
                        double nextVertFactor = 0.0;

                        if (nextHeight <= startHeight) {
                            nextVertFactor = 0.0;
                        } else if (nextHeight >= cloudUpperLevel) {
                            nextVertFactor = 0.0;
                        } else {
                            double distToCenter = fabs(nextHeight - cloudCenter);
                            double normalizedDist = distToCenter / halfThickness;
                            nextVertFactor = 1.0 - normalizedDist;
                        }

                        double nextPed = basePed + (maxPedAtLevel - basePed) * nextVertFactor;
                        pedUpper = nextPed;
                    } else {
                        pedUpper = bgPed;
                    }
                }

                PointData point(lat, lon, height, pedLower, pedUpper);
                m_points.append(point);
            }
        }
    }

    // 6. Устанавливаем начальную позицию
    m_currentGridIndex = 0;
    m_currentLevelIndex = 0;
}



// =============== РАСЧЁТ ПЕД ПО ГАУССОВСКОМУ ЗАКОНУ ===============

// Входные параметры:
//   distance       - расстояние от эпицентра (центра квадрата) в километрах
//   maxPed         - максимальное значение ПЕД в эпицентре (мкЗв/ч)
//   backgroundPed  - фоновое значение ПЕД (мкЗв/ч)
//   maxDistance    - максимальное расстояние, на котором ПЕД снижается до фона (км)
//                    В нашей модели maxDistance = ребро_квадрата / 2.0

double SurveyArea::calculateGaussianPed(double distance, double maxPed,
                                        double backgroundPed, double maxDistance)
{
    // Если расстояние больше или равно максимальному - возвращаем фон
    if (distance >= maxDistance) {
        return backgroundPed;
    }

    // Если максимальное ПЕД меньше или равно фону - возвращаем фон
    if (maxPed <= backgroundPed) {
        return backgroundPed;
    }

    // Вычисляем сигму (стандартное отклонение) для гауссовского распределения
    // Используем коэффициент 3.0 (при distance = maxDistance, значение падает до ~1%)
    // Более точный коэффициент = sqrt(-2 * ln(0.01)) ≈ 3.0349, но для простоты берём 3.0
    double sigma = maxDistance / 3.0;

    // Вычисляем экспоненту: exp(-0.5 * (distance / sigma)^2)
    double ratio = distance / sigma;
    double exponent = exp(-0.5 * ratio * ratio);

    // Линейная интерполяция между фоном и максимумом с весовым коэффициентом exponent
    double result = backgroundPed + (maxPed - backgroundPed) * exponent;

    return result;
}


void SurveyArea::setPoints(const QList<PointData> &points, int numGridPoints, int numLevels, double startHeight)
{
    m_points = points;
    m_numGridPoints = numGridPoints;
    m_numLevels = numLevels;
    m_startHeight = startHeight;
    m_currentGridIndex = 0;
    m_currentLevelIndex = 0;
}
