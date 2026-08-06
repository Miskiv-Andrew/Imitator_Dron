#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QDebug>
#include <QSerialPort>

#include "surveyarea.h"

struct SdRecord
{
    int lineNumber;          // номер строки (linec)
    double temp0;            // температура БДБГ-1 (нижний)
    double temp1;            // температура БДБГ-2 (верхний)
    double ped0;             // ПЕД нижний (pedLower)
    double ped1;             // ПЕД верхний (pedUpper)
    double longitude;        // долгота
    double latitude;         // широта
    int lidarDistance;       // расстояние LIDAR (см)
    int lidarStrength;       // интенсивность LIDAR
};



QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


private slots:

    void onCreateAreaClicked();           // Створити область розвідки
    void onStartReconClicked();           // Почати розвідку
    void onTakeStepClicked();             // Зробити крок
    void onUpLevelClicked();              // Піднятись на 1 рівень
    void onDownLevelClicked();            // Спуститись на 1 рівень
    void onDescendStartHeightClicked();   // Спуститись на початкову висоту
    void displayCurrentPoint();           // отображение текущей точки в UI
    void updateButtonsState();            // обновление состояния кнопок (активна/неактивна)

    void onMoveModeChanged(int index);    // Переключение режима ручной/авто
    void onAutoMoveTick();                // Таймер авто-шага

    void onIntervalChanged(int newValue);

    void onPortMonitorTick();            // Обработчик таймера мониторинга портов


    void on_pushButton_search_Ports_clicked();

    void onBdbgPortSelected(int index);

    void onBdbgDataReceived();


    int calcCrc(const QByteArray &data);    // расчёт CRC (из старого проекта) int calcCrc(const QByteArray &data);
    int writeBdbgData(QSerialPort *port, const QByteArray &data);  // отправка в порт

    void on_pushButton_clear_clicked();  

    void onLidarPortSelected(int index);    // Выбор порта LIDAR

    void onLidarTimerTick();                // Обработчик таймера LIDAR

    void onLidarFreqChanged(int index);     // Смена частоты (перезапуск таймера)

    void onGpsPortSelected(int index);   // Выбор порта GPS

    void onGpsTimerTick();               // Обработчик таймера GPS (отправка данных)

    QString formatLatLon(double value, bool isLatitude);  // Преобразование десятичных градусов в NMEA (ddmm.mmmm)

    QString calculateNmeaChecksum(const QString &sentence); // Расчёт контрольной суммы NMEA


    void onRadiolinePortSelected(int index); // ВЫБОР ПОРТА ДЛЯ РАДІОЛІНІЇ (ПРИЙОМ ДАНИХ ВІД ЛОГГЕРА)

    bool parseSdLine(const QString &line, SdRecord &record);   // ПАРСИНГ СТРОКИ ИЗ ФОРМАТА SD-КАРТЫ / РАДИОЛИНИИ


    void onLoadSdCardClicked(); // ЗАГРУЗКА ДАННЫХ С SD-КАРТЫ

    void onRadiolineDataReceived(); // ОБРАБОТЧИК ПРИЁМА ДАННЫХ ПО РАДИОЛИНИИ

    void onCompareDataClicked();  // СРАВНЕНИЕ ДАННЫХ (ЭТАЛОН, SD-КАРТА, РАДИОЛИНИЯ)

    void addEtalonRecord();    // ФОРМИРОВАНИЕ ЭТАЛОННОЙ ЗАПИСИ ДЛЯ ТЕКУЩЕЙ ТОЧКИ


    void generatePathPoints(double startLat, double startLon, double endLat, double endLon,
                            int count, double height, double bgPed, QList<PointData> &targetList); // для генерации точек пути


    void generatePolygonPoints(double startLat, double startLon, double sideKm);  // для генерации 12 точек полигона


    bool isPointInPolygon(const PointData &point, const QList<PointData> &polygon); // для проверки вхождения точки в полигон


    void on_pushButton_show_gps_packages_clicked();

    void on_pushButton_show_lidar_packages_clicked();

    void on_pushButton_show_instruction_clicked();


    void on_pushButton_send_comm_clicked();

    // CRC16 (CCITT) для расчёта контрольной суммы пакетов
    uint16_t calculateCrc16(const QByteArray &data);

    // // Формирование пакета по протоколу
    QByteArray buildCommandPacket(uint8_t commandId, const QByteArray &payload);

    // // Отправка команды на дрон
    void sendCommandToDrone(uint8_t commandId, const QByteArray &payload);



    // // Вспомогательные методы для отображения
    QString getCommandName(uint8_t id);
    QString getResponseText(uint8_t id);

    QString formatPayload(const QByteArray &payload, uint8_t id);


    // // Обработка принятого ответа от дрона
    void processDroneResponse(const QByteArray &packet);

private:

    Ui::MainWindow *ui;
    SurveyArea m_surveyArea;    
    QTimer *m_autoMoveTimer;           // Таймер для автоматического перемещения
    bool m_isAutoMoving;               // Флаг авторежима (для остановки)    

    QTimer *m_portMonitorTimer;          // Таймер мониторинга портов
    QStringList m_knownPorts;            // Список портов, которые уже были обнаружены

    QSerialPort *bdbg_port;             // порт для первого и второго дозиметров (БДБГ-15)

    QSerialPort *lidar_port;           // COM-порт для LIDAR
    QTimer *lidar_timer;               // Таймер для непрерывной отправки данных
    bool lidar_is_active;              // Флаг активности LIDAR (для контроля)


    QSerialPort *gps_port;          // COM-порт для GPS
    QTimer *gps_timer;              // Таймер для непрерывной отправки данных (1 Гц)
    bool gps_is_active;             // Флаг активности GPS

    bool m_isReconRunning;           // Флаг: запущена ли разведка

    QList<SdRecord> m_etalonRecords;      // эталонные записи (формируются во время разведки)
    QList<SdRecord> m_sdRecords;          // записи с SD-карты
    QList<SdRecord> m_radiolineRecords;   // записи из радиолинии

    QSerialPort *logger_rx_port;          // порт для приёма данных от логгера (радиолиния)

    QList<PointData> m_polygonPoints;    // 12 точек полигона (для проверки вхождения)

    QByteArray m_radioBuffer;            // Буфер для накопления данных с радиолинии



};
#endif // MAINWINDOW_H
