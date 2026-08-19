#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QDebug>
#include <QSerialPort>
#include <QVector>
#include <QDateTime>

#include "surveyarea.h"

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
    void onCreateAreaClicked();
    void onStartReconClicked();
    void onTakeStepClicked();
    void onUpLevelClicked();
    void onDownLevelClicked();
    void onDescendStartHeightClicked();
    void displayCurrentPoint();
    void updateButtonsState();

    void onMoveModeChanged(int index);
    void onAutoMoveTick();

    void onIntervalChanged(int newValue);

    void onPortMonitorTick();

    void on_pushButton_search_Ports_clicked();

    void onBdbgPortSelected(int index);
    void onBdbgDataReceived();

    int calcCrc(const QByteArray &data);
    int writeBdbgData(QSerialPort *port, const QByteArray &data);

    void on_pushButton_clear_clicked();

    void onLidarPortSelected(int index);
    void onLidarTimerTick();
    void onLidarFreqChanged(int index);

    void onGpsPortSelected(int index);
    void onGpsTimerTick();

    QString formatLatLon(double value, bool isLatitude);
    QString calculateNmeaChecksum(const QString &sentence);

    void onRadiolinePortSelected(int index);
    void onRadiolineDataReceived();

    void onLoadSdCardClicked();
    void onCompareDataClicked();

    void on_pushButton_show_gps_packages_clicked();
    void on_pushButton_show_lidar_packages_clicked();
    void on_pushButton_show_instruction_clicked();

    void setupSpectrumPlot();
    void updateSpectrumPlot(const QVector<double> &data);

    void on_pushButton_clicked();

    void loadEtalonSpectrum(const QString &fileName);
    void sendCommandToDrone(uint8_t commandId, const QByteArray &payload);
    void processDroneResponse(const QByteArray &packet);
    void on_pushButton_send_comm_clicked();
    QString getCommandName(uint8_t id);
    QString getResponseText(uint8_t id);
    QString formatPayload(const QByteArray &payload, uint8_t id);

    uint16_t calculateCrc16(const QByteArray &data);
    QByteArray buildCommandPacket(uint8_t commandId, const QByteArray &payload);

    void generatePolygonPoints(double startLat, double startLon, double sideKm);
    bool isPointInPolygon(const PointData &point, const QList<PointData> &polygon);
    void generatePathPoints(double startLat, double startLon,
                            double endLat, double endLon,
                            int count, double height, double bgPed,
                            QList<PointData> &targetList);

    // === LIDAR / GPS continuous and one-shot (точные имена, как в .cpp) ===
    void onCheckBoxLidarContinuousToggled(bool checked);
    void on_pushButton_send_lidar_once_clicked();
    void onCheckBoxGpsContinuousToggled(bool checked);
    void on_pushButton_send_gps_once_clicked();

    void sendSingleLidarPacket();
    void sendSingleGpsPacket();

private:
    Ui::MainWindow *ui;
    SurveyArea m_surveyArea;
    QTimer *m_autoMoveTimer;
    bool m_isAutoMoving;

    QTimer *m_portMonitorTimer;
    QStringList m_knownPorts;

    QSerialPort *bdbg_port;
    QSerialPort *lidar_port;
    QTimer *lidar_timer;
    bool lidar_is_active;

    QSerialPort *gps_port;
    QTimer *gps_timer;
    bool gps_is_active;

    bool m_isReconRunning;

    QList<PointData> m_polygonPoints;

    QByteArray m_radioBuffer;

    QVector<double> m_spectrumData;

    QByteArray m_etalonSpectrumReply;

    qint64 m_spectrumStartTime;
    qint64 m_prevSpectrumRequestTime;

    // === Добавлен порт для радиолинии ===
    QSerialPort *logger_rx_port;

    // Списки для хранения эталонных и принятых пакетов
    QList<QByteArray> m_etalonPayloads;     // Эталонные пакеты (Command ID + Payload)
    QList<QByteArray> m_radiolinePayloads;  // Пакеты, принятые по радиолинии

    // Индексы для предотвращения дублирования
    int m_lastSavedPointIndex;               // Для Пакета №1 (внутри зоны)
    int m_lastPathPointIndex;                // Для Пакета №5 (вне зоны)
    bool m_isEntryLogged;                    // Для Пакета №6 (первый вход в зону)
};

#endif // MAINWINDOW_H
