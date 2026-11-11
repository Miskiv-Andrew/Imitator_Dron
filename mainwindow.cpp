
#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "mapcore.h"

#include <QSerialPortInfo>
#include <QRandomGenerator>
#include <QDateTime>
#include <QFile>
#include <QDir>
#include <QFileDialog>
#include <QVector>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    showMaximized();

    m_draw.setwindow(ui->p_map);

    // Подключение кнопок
    connect(ui->create_reconn_area_but, &QPushButton::clicked, this, &MainWindow::onCreateAreaClicked);
    connect(ui->start_reconn_but, &QPushButton::clicked, this, &MainWindow::onStartReconClicked);
    connect(ui->take_step_but, &QPushButton::clicked, this, &MainWindow::onTakeStepClicked);
    connect(ui->up_1_level_but, &QPushButton::clicked, this, &MainWindow::onUpLevelClicked);
    connect(ui->put_1_level_bu, &QPushButton::clicked, this, &MainWindow::onDownLevelClicked);
    connect(ui->descend_start_height_but, &QPushButton::clicked, this, &MainWindow::onDescendStartHeightClicked);

    // Инициализация таймера авто-перемещения
    m_autoMoveTimer = new QTimer(this);
    connect(m_autoMoveTimer, &QTimer::timeout, this, &MainWindow::onAutoMoveTick);
    m_isAutoMoving = false;

    // Подключение сигнала изменения режима перемещения
    connect(ui->move_mode_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onMoveModeChanged);

    onMoveModeChanged(0);

    connect(ui->req_interval_box, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MainWindow::onIntervalChanged);

    connect(ui->start_height_box, QOverload<int>::of(&QSpinBox::valueChanged),
            ui->cloud__lower_level_box, &QSpinBox::setValue);

    // Заполнение комбобоксов портами
    for (int i = 1; i <= 40; ++i) {
        ui->comboBox_BDBG_1->addItem(QString("COM%1").arg(i));
        ui->comboBox_LIDAR->addItem(QString("COM%1").arg(i));
        ui->comboBox_GPS->addItem(QString("COM%1").arg(i));
        ui->comboBox_radioline->addItem(QString("COM%1").arg(i));
    }

    // Инициализация таймера мониторинга портов
    m_portMonitorTimer = new QTimer(this);
    connect(m_portMonitorTimer, &QTimer::timeout, this, &MainWindow::onPortMonitorTick);

    QList<QSerialPortInfo> initialPorts = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &port : initialPorts) {
        m_knownPorts.append(port.portName());
    }

    m_portMonitorTimer->start(500);

    // Инициализация портов
    bdbg_port = new QSerialPort(this);
    connect(bdbg_port, &QSerialPort::readyRead, this, &MainWindow::onBdbgDataReceived);
    connect(ui->comboBox_BDBG_1, QOverload<int>::of(&QComboBox::activated),
            this, &MainWindow::onBdbgPortSelected);

    // LIDAR
    lidar_port = new QSerialPort(this);
    lidar_timer = new QTimer(this);
    lidar_timer->setSingleShot(false);
    connect(lidar_timer, &QTimer::timeout, this, &MainWindow::onLidarTimerTick);
    lidar_is_active = false;

    ui->comboBox_freq_lidar->addItem("1 Гц");
    ui->comboBox_freq_lidar->addItem("100 Гц");
    ui->comboBox_freq_lidar->setCurrentIndex(1);

    ui->comboBox_imit_mode_lidar->addItem("Нормальне вимірювання");
    ui->comboBox_imit_mode_lidar->addItem("Об'єкт не виявлено");
    ui->comboBox_imit_mode_lidar->addItem("Слабке відбиття");
    ui->comboBox_imit_mode_lidar->addItem("Сильне відбиття");
    ui->comboBox_imit_mode_lidar->addItem("Об'єкт дуже близько");
    ui->comboBox_imit_mode_lidar->addItem("Об'єкт занадто далеко");
    ui->comboBox_imit_mode_lidar->setCurrentIndex(0);

    connect(ui->comboBox_LIDAR, QOverload<int>::of(&QComboBox::activated), this, &MainWindow::onLidarPortSelected);
    connect(ui->comboBox_freq_lidar, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onLidarFreqChanged);

    // GPS
    gps_port = new QSerialPort(this);
    gps_timer = new QTimer(this);
    gps_timer->setSingleShot(false);
    connect(gps_timer, &QTimer::timeout, this, &MainWindow::onGpsTimerTick);
    gps_is_active = false;

    ui->comboBox_freq_gps->addItem("1 Гц");
    ui->comboBox_freq_gps->setCurrentIndex(0);
    connect(ui->comboBox_GPS, QOverload<int>::of(&QComboBox::activated),
            this, &MainWindow::onGpsPortSelected);

    // Радиолиния
    for (int i = 1; i <= 40; ++i) {
        ui->comboBox_radioline->addItem(QString("COM%1").arg(i));
    }

    logger_rx_port = new QSerialPort(this);
    connect(logger_rx_port, &QSerialPort::readyRead, this, &MainWindow::onRadiolineDataReceived);
    connect(ui->comboBox_radioline, QOverload<int>::of(&QComboBox::activated),
            this, &MainWindow::onRadiolinePortSelected);

    // Кнопки SD и сравнения пока оставляем, но закомментируем их реализацию (пока заглушки)
    connect(ui->pushButton_load_SD_data, &QPushButton::clicked,
            this, &MainWindow::onLoadSdCardClicked);
    connect(ui->pushButton_compare_data, &QPushButton::clicked,
            this, &MainWindow::onCompareDataClicked);

    // Настройка графика спектра
    setupSpectrumPlot();

    // Чекбоксы непрерывной передачи
    connect(ui->checkBox_lidar_continuous, &QCheckBox::toggled,
            this, &MainWindow::onCheckBoxLidarContinuousToggled);
    connect(ui->checkBox_gps_continuous, &QCheckBox::toggled,
            this, &MainWindow::onCheckBoxGpsContinuousToggled);

    // Загрузка эталонного спектра
    loadEtalonSpectrum("D:\\DRON ECOTEST\\Imitation\\Devices\\stage_1\\build\\Desktop_Qt_6_7_3_MinGW_64_bit-Debug\\debug\\packet_Cs.txt");


    m_spectreNumber = 0;

    m_lastLidarDistance = 0;
    m_lastLidarStrength = 0;
    m_lastGpsLatitude = 0.0;
    m_lastGpsLongitude = 0.0;
    m_lastGpsUtcTime = QTime(0, 0, 0);

    m_gpsJournal.clear();

    m_lastRadioPacket.clear();
    m_lastRadioPacketTime = 0;
}

MainWindow::~MainWindow()
{
    if (m_autoMoveTimer) {
        m_autoMoveTimer->stop();
        m_autoMoveTimer->deleteLater();
    }

    if (bdbg_port->isOpen()) bdbg_port->close();
    bdbg_port->deleteLater();

    if (lidar_timer && lidar_timer->isActive()) lidar_timer->stop();
    if (lidar_port && lidar_port->isOpen()) lidar_port->close();
    lidar_timer->deleteLater();
    lidar_port->deleteLater();

    if (gps_timer && gps_timer->isActive()) gps_timer->stop();
    if (gps_port && gps_port->isOpen()) gps_port->close();
    gps_timer->deleteLater();
    gps_port->deleteLater();

    if (logger_rx_port && logger_rx_port->isOpen()) logger_rx_port->close();
    logger_rx_port->deleteLater();

    delete ui;
}


void MainWindow::displayCurrentPoint()
{
    PointData currentPoint = m_surveyArea.getCurrentPoint();

    // Обновляем поля текущих координат и высоты
    ui->curr_lat_box->setValue(currentPoint.getLatitude());
    ui->curr_lon_box->setValue(currentPoint.getLongitude());
    ui->curr_height_box->setValue(currentPoint.getHeight());

    // Определяем тип угрозы для отображения на карте
    int dangerType = 0;
    double bgPed = ui->paed_phone_box->value();
    if (currentPoint.getPedLower() > bgPed + 0.0001 ||
        currentPoint.getPedUpper() > bgPed + 0.0001) {
        dangerType = 1;
    }
    ui->p_map->setdronepos({currentPoint.getLongitude(), currentPoint.getLatitude()}, dangerType);

    // Определяем, находится ли точка в зоне (для статуса)
    bool inZone = isPointInPolygon(currentPoint, m_polygonPoints);

    // ============================================================
    // ВЫВОД ИНФОРМАЦИИ О ТОЧКЕ
    // ============================================================

    QString info = QString("Поточна точка:\n"
                           "  Широта: %1\n"
                           "  Довгота: %2\n"
                           "  Висота: %3 м\n"
                           "  ПЕД нижній (вниз): %4 мкЗв/ч\n"
                           "  ПЕД верхній (вгору): %5 мкЗв/ч\n"
                           "  Індекс вузла: %6, Індекс рівня: %7\n"
                           "----------------------------------------")
                       .arg(currentPoint.getLatitude(), 0, 'f', 6)
                       .arg(currentPoint.getLongitude(), 0, 'f', 6)
                       .arg(currentPoint.getHeight())
                       .arg(currentPoint.getPedLower(), 0, 'f', 3)
                       .arg(currentPoint.getPedUpper(), 0, 'f', 3)
                       .arg(m_surveyArea.getCurrentGridIndex())
                       .arg(m_surveyArea.getCurrentLevelIndex());

    if (inZone) {
        info += "\n  Статус: В зоні";
    } else {
        info += "\n  Статус: Поза зоною";
    }

    info += "\n----------------------------------------";

    ui->textEdit->append(info);
}




void MainWindow::updateButtonsState()
{
    QString reconMode = ui->recogn_mode_combo->currentText();
    bool isCloud = (reconMode == "Хмара");
    bool isManual = (ui->move_mode_combo->currentIndex() == 0);

    ui->take_step_but->setEnabled(isManual && m_surveyArea.canMoveForward());
    ui->up_1_level_but->setEnabled(isManual && isCloud && m_surveyArea.canMoveUp());
    ui->put_1_level_bu->setEnabled(isManual && isCloud && m_surveyArea.canMoveDown());
    ui->descend_start_height_but->setEnabled(isManual && isCloud);
}

void MainWindow::onMoveModeChanged(int index)
{
    bool isManual = (index == 0);
    QString reconMode = ui->recogn_mode_combo->currentText();
    bool isCloud = (reconMode == "Хмара");

    ui->take_step_but->setEnabled(isManual && m_surveyArea.canMoveForward());
    ui->up_1_level_but->setEnabled(isManual && isCloud && m_surveyArea.canMoveUp());
    ui->put_1_level_bu->setEnabled(isManual && isCloud && m_surveyArea.canMoveDown());
    ui->descend_start_height_but->setEnabled(isManual && isCloud);

    if (isManual && m_isAutoMoving) {
        m_autoMoveTimer->stop();
        m_isAutoMoving = false;
        ui->start_reconn_but->setText("Почати розвідку");
        ui->textEdit->append("Авторежим зупинено (ручне керування)");
    }
}

void MainWindow::onIntervalChanged(int newValue)
{
    if (m_isAutoMoving && newValue > 0) {
        m_autoMoveTimer->start(newValue * 1000);
        ui->textEdit->append(QString("Інтервал опиту змінено на %1 сек").arg(newValue));
    }
}

void MainWindow::onPortMonitorTick()
{
    QList<QSerialPortInfo> currentPorts = QSerialPortInfo::availablePorts();
    QStringList currentPortNames;

    for (const QSerialPortInfo &port : currentPorts) {
        currentPortNames.append(port.portName());
    }

    for (const QString &portName : currentPortNames) {
        if (!m_knownPorts.contains(portName)) {
            ui->lineEdit_COM->setText(portName);
            m_knownPorts.append(portName);
        }
    }

    for (int i = m_knownPorts.size() - 1; i >= 0; --i) {
        if (!currentPortNames.contains(m_knownPorts[i])) {
            m_knownPorts.removeAt(i);
        }
    }
}

void MainWindow::onAutoMoveTick()
{
    QString mode = ui->recogn_mode_combo->currentText();

    if (mode == "Поверхня") {
        if (m_surveyArea.moveForward()) {
            displayCurrentPoint();
            updateButtonsState();
        } else {
            m_autoMoveTimer->stop();
            m_isAutoMoving = false;
            ui->start_reconn_but->setText("Почати розвідку");
            ui->move_mode_combo->setCurrentIndex(0);
            ui->textEdit->append("Авторежим завершено! Пройдено всі точки.");
        }
    }
    else if (mode == "Хмара") {
        int currentLevel = m_surveyArea.getCurrentLevelIndex();
        int numLevels = m_surveyArea.getNumLevels();

        if (currentLevel + 1 < numLevels) {
            m_surveyArea.moveUp();
            displayCurrentPoint();
            updateButtonsState();
        } else {
            if (m_surveyArea.moveForward()) {
                m_surveyArea.resetToStartHeight();
                displayCurrentPoint();
                updateButtonsState();
            } else {
                m_autoMoveTimer->stop();
                m_isAutoMoving = false;
                ui->start_reconn_but->setText("Почати розвідку");
                ui->move_mode_combo->setCurrentIndex(0);
                ui->textEdit->append("Авторежим завершено! Пройдено всі точки на всіх рівнях.");
            }
        }
    }
}










void MainWindow::onCreateAreaClicked()
{
    ui->textEdit->clear();

    // --- 1. Считываем общие параметры ---
    double startLat = ui->start_lat_box->value();
    double startLon = ui->start_lon_box->value();
    double sideKm = ui->edge_square_box->value();
    int numPoints = ui->point_number_box->value();
    double maxPed = ui->paed_max_box->value();
    double bgPed = ui->paed_phone_box->value();
    double startHeight = ui->start_height_box->value();
    QString mode = ui->recogn_mode_combo->currentText();

    // --- 2. Генерируем полигон (12 точек) ---
    generatePolygonPoints(startLat, startLon, sideKm);

    // --- 3. Генерируем точки измерений ---
    QList<PointData> measurementPoints;

    if (mode == "Поверхня") {
        m_surveyArea.generateSurfacePoints(startLat, startLon, sideKm,
                                           numPoints, maxPed, bgPed, startHeight);
    } else if (mode == "Хмара") {
        double cloudUpperLevel = ui->cloud__upper_level_box->value();
        int numLevels = ui->num_lift_step_box->value();
        double cloudCoeff = ui->cloud__increment_coeff_box_2->value();
        m_surveyArea.generateCloudPoints(startLat, startLon, sideKm,
                                         numPoints, maxPed, bgPed, startHeight,
                                         cloudUpperLevel, numLevels, cloudCoeff);
    } else {
        ui->textEdit->append("ПОМИЛКА: Невідомий режим розвідки!");
        return;
    }

    measurementPoints = m_surveyArea.getPoints();
    if (measurementPoints.isEmpty()) {
        ui->textEdit->append("ПОМИЛКА: не створено жодної точки вимірювань!");
        return;
    }

    // --- 4. Создаём внешнюю стартовую точку ---
    double offsetLat = 0.01;
    double offsetLon = 0.01;
    double startLatOutside = startLat + offsetLat;
    double startLonOutside = startLon - offsetLon;

    // --- 5. Читаем количество точек пути ---
    int pathCount = ui->spinBox_path_to_zone->value();

    // --- 6. Генерируем путь к зоне ---
    QList<PointData> pathToZone;
    PointData firstMeasPoint = measurementPoints.first();

    generatePathPoints(startLatOutside, startLonOutside,
                       firstMeasPoint.getLatitude(), firstMeasPoint.getLongitude(),
                       pathCount, startHeight, bgPed, pathToZone);

    // --- 7. Генерируем путь обратно (в 2 раза больше) ---
    int pathBackCount = pathCount * 2;
    QList<PointData> pathFromZone;
    PointData lastMeasPoint = measurementPoints.last();

    generatePathPoints(lastMeasPoint.getLatitude(), lastMeasPoint.getLongitude(),
                       startLatOutside, startLonOutside,
                       pathBackCount, startHeight, bgPed, pathFromZone);

    // --- 8. Формируем полный маршрут ---
    QList<PointData> fullRoute;
    fullRoute.append(pathToZone);

    for (int i = 1; i < measurementPoints.size(); ++i) {
        fullRoute.append(measurementPoints[i]);
    }

    for (int i = 1; i < pathFromZone.size(); ++i) {
        fullRoute.append(pathFromZone[i]);
    }

    // --- 9. Передаём в SurveyArea ---
    int totalGridPoints = fullRoute.size();
    int numLevels = (mode == "Хмара") ? ui->num_lift_step_box->value() : 1;
    m_surveyArea.setPoints(fullRoute, totalGridPoints, numLevels, startHeight);

    // --- 10. Вывод информации ---
    ui->textEdit->append("=== РЕЖИМ " + mode.toUpper() + " ===");
    ui->textEdit->append(QString("Точок шляху до зони: %1").arg(pathCount));
    ui->textEdit->append(QString("Точок вимірювань: %1").arg(measurementPoints.size()));
    ui->textEdit->append(QString("Точок шляху назад: %1").arg(pathCount));
    ui->textEdit->append(QString("Загальна кількість точок маршруту: %1").arg(fullRoute.size()));
    ui->textEdit->append("----------------------------------------");

    // ============================================================
    // ОЧИСТКА СПИСКОВ ПРИ СОЗДАНИИ НОВОЙ ОБЛАСТИ
    // ============================================================
    m_etalonPayloads.clear();
    m_radiolinePayloads.clear();
    m_gpsJournal.clear();

    ui->textEdit_radioline->clear();

    ui->lineEdit_ID_spectre->clear();
    ui->lineEdit_num_spectre->clear();
    ui->lineEdit_coords_spectre->clear();
    ui->lineEdit_high_spectre->clear();

    // --- 11. Отображаем первую точку ---
    displayCurrentPoint();
    updateButtonsState();
}



void MainWindow::onStartReconClicked()
{
    if (!m_isReconRunning) {
        if (m_surveyArea.getTotalPointsCount() == 0) {
            ui->textEdit->append("Помилка: спочатку створіть область розвідки");
            return;
        }

        // Запуск LIDAR (если чекбокс включён)
        if (ui->checkBox_lidar_continuous->isChecked() && lidar_port && lidar_port->isOpen()) {
            int freqIndex = ui->comboBox_freq_lidar->currentIndex();
            int intervalMs = (freqIndex == 0) ? 1000 : 10;
            lidar_timer->start(intervalMs);
            lidar_is_active = true;
            ui->textEdit->append(QString("LIDAR: запущено (частота %1)")
                                     .arg(ui->comboBox_freq_lidar->currentText()));
        } else if (!ui->checkBox_lidar_continuous->isChecked()) {
            ui->textEdit->append("LIDAR: безперервна передача вимкнена");
        } else {
            ui->textEdit->append("LIDAR: порт не відкрито, запуск неможливий");
        }

        // Запуск GPS
        if (ui->checkBox_gps_continuous->isChecked() && gps_port && gps_port->isOpen()) {
            gps_timer->start(1000);
            gps_is_active = true;
            ui->textEdit->append("GPS: запущено (частота 1 Гц)");
        } else if (!ui->checkBox_gps_continuous->isChecked()) {
            ui->textEdit->append("GPS: безперервна передача вимкнена");
        } else {
            ui->textEdit->append("GPS: порт не відкрито, запуск неможливий");
        }

        // Авторежим
        if (ui->move_mode_combo->currentIndex() == 1) {
            int interval = ui->req_interval_box->value() * 1000;
            if (interval <= 0) {
                ui->textEdit->append("Помилка: інтервал опиту має бути більше 0");
                if (lidar_timer->isActive()) {
                    lidar_timer->stop();
                    lidar_is_active = false;
                }
                if (gps_timer->isActive()) {
                    gps_timer->stop();
                    gps_is_active = false;
                }
                return;
            }
            m_autoMoveTimer->start(interval);
            m_isAutoMoving = true;
            ui->textEdit->append(QString("Авторежим запущено (інтервал %1 сек)")
                                     .arg(ui->req_interval_box->value()));
            ui->move_mode_combo->setCurrentIndex(1);
        } else {
            ui->textEdit->append("Ручний режим: використовуйте кнопку 'Зробити крок'");
        }

        m_isReconRunning = true;
        ui->start_reconn_but->setText("Зупинити розвідку");
        displayCurrentPoint();

    } else {
        // Остановка
        if (lidar_timer->isActive()) {
            lidar_timer->stop();
        }
        lidar_is_active = false;
        ui->textEdit->append("LIDAR: зупинено");

        if (gps_timer->isActive()) {
            gps_timer->stop();
        }
        gps_is_active = false;
        ui->textEdit->append("GPS: зупинено");

        if (m_autoMoveTimer->isActive()) {
            m_autoMoveTimer->stop();
            ui->textEdit->append("Авторежим зупинено");
        }
        m_isAutoMoving = false;

        m_isReconRunning = false;
        ui->move_mode_combo->setCurrentIndex(0);
        ui->start_reconn_but->setText("Почати розвідку");
        updateButtonsState();
        ui->textEdit->append("Розвідку зупинено");
    }
}

void MainWindow::onTakeStepClicked()
{
    if (m_surveyArea.moveForward()) {
        displayCurrentPoint();
        updateButtonsState();
    } else {
        ui->textEdit->append("Немає наступної точки! Розвідку завершено.");
    }
}

void MainWindow::onUpLevelClicked()
{
    if (m_surveyArea.moveUp()) {
        displayCurrentPoint();
        updateButtonsState();
    } else {
        ui->textEdit->append("Досягнуто максимального рівня!");
    }
}

void MainWindow::onDownLevelClicked()
{
    if (m_surveyArea.moveDown()) {
        displayCurrentPoint();
        updateButtonsState();
    } else {
        ui->textEdit->append("Досягнуто мінімального рівня!");
    }
}

void MainWindow::onDescendStartHeightClicked()
{
    m_surveyArea.resetToStartHeight();
    displayCurrentPoint();
    updateButtonsState();
    ui->textEdit->append("Висота скинута на початкову");
}

void MainWindow::on_pushButton_search_Ports_clicked()
{
    if (!m_isAutoMoving && !m_portMonitorTimer->isActive()) {
        m_portMonitorTimer->start(500);
    }
}

// ======================================================================
// ВЫБОР ПОРТА ДЛЯ БДБГ-1
// ======================================================================
void MainWindow::onBdbgPortSelected(int index)
{
    Q_UNUSED(index)
    QString portName = ui->comboBox_BDBG_1->currentText();
    if (portName == "NOT SELECT" || portName.isEmpty()) return;

    if (bdbg_port->isOpen()) bdbg_port->close();

    bdbg_port->setPortName(portName);
    bdbg_port->setBaudRate(QSerialPort::Baud19200);
    bdbg_port->setDataBits(QSerialPort::Data8);
    bdbg_port->setParity(QSerialPort::NoParity);
    bdbg_port->setStopBits(QSerialPort::OneStop);
    bdbg_port->setFlowControl(QSerialPort::NoFlowControl);

    if (bdbg_port->open(QSerialPort::ReadWrite)) {
        ui->textEdit->append(QString("БДБГ-1 порт %1 відкрито").arg(portName));
    } else {
        ui->textEdit->append(QString("Помилка відкриття порту %1 для БДБГ-1").arg(portName));
    }
}















// ============================================================
// ОБРАБОТЧИК ВХОДЯЩИХ ДАННЫХ ОТ БДБГ (ОДНА ШИНА, ДВА АДРЕСА)
// ============================================================
void MainWindow::onBdbgDataReceived()
{
    // --- 1. ЧИТАЕМ ДАННЫЕ ИЗ ПОРТА ---

    QByteArray receiveBuff;

    QEventLoop loop;
    QTimer::singleShot(100, &loop, [&loop]() {
        loop.quit();
    });
    loop.exec();

    receiveBuff.append(bdbg_port->readAll());

    while (bdbg_port->waitForReadyRead(50)) {
        receiveBuff.append(bdbg_port->readAll());
    }

    bdbg_port->clear();

    if (receiveBuff.isEmpty()) {
        return;
    }

    QString hexReceived = receiveBuff.toHex().toUpper();
    ui->textEdit->append("БДБГ прийнято: " + hexReceived);

    // --- 2. ПРОВЕРКА CRC ---

    if (receiveBuff.size() < 2) {
        ui->textEdit->append("БДБГ: пакет занадто короткий");
        return;
    }

    QByteArray dataWithoutCrc = receiveBuff.left(receiveBuff.size() - 1);
    char receivedCrc = receiveBuff[receiveBuff.size() - 1];

    int calculatedCrc = calcCrc(dataWithoutCrc);

    if (static_cast<char>(calculatedCrc) != receivedCrc) {
        ui->textEdit->append("БДБГ: помилка CRC");
        return;
    }

    // --- 3. ИЗВЛЕКАЕМ АДРЕС ---

    if (receiveBuff.size() <= 3) {
        ui->textEdit->append("БДБГ: пакет занадто короткий для визначення адреси");
        return;
    }

    quint8 address = static_cast<quint8>(receiveBuff[3]);

    // --- 4. ПРОВЕРКА ЧЕКБОКСА "ВІДСУТНІСТЬ ЗВ'ЯЗКУ" ---

    if (address == 0x01) {
        if (ui->checkBox_not_ans_1->isChecked()) {
            ui->textEdit->append("БДБГ: імітація відмови (адр.1) — немає відповіді");
            return;
        }
    } else if (address == 0x02) {
        if (ui->checkBox_not_ans_2->isChecked()) {
            ui->textEdit->append("БДБГ: імітація відмови (адр.2) — немає відповіді");
            return;
        }
    } else {
        ui->textEdit->append(QString("БДБГ: невідома адреса 0x%1").arg(address, 2, 16, QChar('0')));
        return;
    }

    // --- 5. ПОЛУЧАЕМ ДАННЫЕ ИЗ ТЕКУЩЕЙ ТОЧКИ ---

    PointData currentPoint = m_surveyArea.getCurrentPoint();

    double pedValue = 0.0;
    if (address == 0x01) {
        pedValue = currentPoint.getPedLower();
    } else if (address == 0x02) {
        pedValue = currentPoint.getPedUpper();
    }

    // --- 6. ОПРЕДЕЛЯЕМ КОМАНДУ И ФОРМИРУЕМ ОТВЕТ ---

    if (receiveBuff.size() <= 4) {
        ui->textEdit->append("БДБГ: пакет занадто короткий для визначення команди");
        return;
    }

    quint8 command = static_cast<quint8>(receiveBuff[4]);

    QByteArray response;

    // ===== ЗАПИТ ТЕМПЕРАТУРЫ (0x08) =====
    if (command == 0x08) {
        response.append(QByteArray::fromHex("55AA70"));
        response.append(static_cast<char>(address));
        response.append(QByteArray::fromHex("085A01"));

        int crc = calcCrc(response);

        if (address == 0x01 && ui->checkBox_uncorrect_1->isChecked()) {
            crc += 0x02;
        } else if (address == 0x02 && ui->checkBox_uncorrect_2->isChecked()) {
            crc += 0x02;
        }

        response.append(static_cast<char>(crc & 0xFF));

        QString hexResponse = response.toHex().toUpper();
        ui->textEdit->append("БДБГ відправлено: " + hexResponse);

        writeBdbgData(bdbg_port, response);

        ui->textEdit->append("БДБГ: відповідь на запит температури відправлено (адр. " +
                             QString::number(address) + ")");
    }

    // ===== ЗАПИТ ПАЕД (0x00) =====
    else if (command == 0x00) {
        response.append(QByteArray::fromHex("55AA70"));
        response.append(static_cast<char>(address));
        response.append(QByteArray::fromHex("01"));

        float pedFloat = static_cast<float>(pedValue);
        const char* pedBytes = reinterpret_cast<const char*>(&pedFloat);
        response.append(pedBytes, sizeof(float));

        quint8 statusByte = 0x11;

        if (address == 0x01) {
            if (ui->checkBox_high_sens_1->isChecked()) statusByte |= 0x01;
            if (ui->checkBox_low_sens_1->isChecked()) statusByte |= 0x02;
        } else if (address == 0x02) {
            if (ui->checkBox_high_sens_2->isChecked()) statusByte |= 0x01;
            if (ui->checkBox_low_sens_2->isChecked()) statusByte |= 0x02;
        }

        response.append(static_cast<char>(statusByte));
        response.append(static_cast<char>(0x00));

        int crc = calcCrc(response);

        if (address == 0x01 && ui->checkBox_uncorrect_1->isChecked()) {
            crc += 0x02;
        } else if (address == 0x02 && ui->checkBox_uncorrect_2->isChecked()) {
            crc += 0x02;
        }

        response.append(static_cast<char>(crc & 0xFF));

        writeBdbgData(bdbg_port, response);

        ui->textEdit->append(QString("БДБГ: відповідь на запит ПЕД відправлено (адр. %1, ПЕД = %2)")
                                 .arg(address).arg(pedValue, 0, 'f', 3));

        // ============================================================
        // ФОРМИРОВАНИЕ ПАКЕТА №1 (ID 0x16) ДЛЯ ЭТАЛОНА
        // Payload 31 байт, пакет [ID][Payload] = 32 байта
        // ============================================================

        QByteArray packet;
        packet.append(static_cast<char>(0x16));  // Command ID

        // 1. ПАЕД (float 4)
        float pedFloat2 = static_cast<float>(pedValue);
        const char* pedBytes2 = reinterpret_cast<const char*>(&pedFloat2);
        packet.append(pedBytes2, sizeof(float));

        // 2. Температура (uint16 LE, 0x015A)
        packet.append(static_cast<char>(0x5A));
        packet.append(static_cast<char>(0x01));

        // 3. Состояние (тот же байт, что ушёл в ответе на ПАЕД)
        packet.append(static_cast<char>(statusByte));

        // 4. Высота LIDAR (uint16 LE) — последний отправленный
        packet.append(static_cast<char>(m_lastLidarDistance & 0xFF));
        packet.append(static_cast<char>((m_lastLidarDistance >> 8) & 0xFF));

        // 5. Мощность LIDAR (uint16 LE) — последняя отправленная
        packet.append(static_cast<char>(m_lastLidarStrength & 0xFF));
        packet.append(static_cast<char>((m_lastLidarStrength >> 8) & 0xFF));

        // 6. Широта (float 4) — из последнего отправленного GPS
        float lat = static_cast<float>(m_lastGpsLatitude);
        const char* latBytes = reinterpret_cast<const char*>(&lat);
        packet.append(latBytes, sizeof(float));

        // 7. Долгота (float 4) — из последнего отправленного GPS
        float lon = static_cast<float>(m_lastGpsLongitude);
        const char* lonBytes = reinterpret_cast<const char*>(&lon);
        packet.append(lonBytes, sizeof(float));

        // 8. Высота GPS (float 4) = 0.0
        float gpsAlt = 0.0f;
        const char* altBytes = reinterpret_cast<const char*>(&gpsAlt);
        packet.append(altBytes, sizeof(float));

        // 9. Количество спутников (uint8_t) = 0
        packet.append(static_cast<char>(0x00));

        // 10. Напряжение батареи (float 4) — 11.1 V
        float batteryVoltage = 11.1f;
        const char* battBytes = reinterpret_cast<const char*>(&batteryVoltage);
        packet.append(battBytes, sizeof(float));

        // 11. Время (3 байта: HH MM SS) — UTC из последнего отправленного GPS
        packet.append(static_cast<char>(m_lastGpsUtcTime.hour()));
        packet.append(static_cast<char>(m_lastGpsUtcTime.minute()));
        packet.append(static_cast<char>(m_lastGpsUtcTime.second()));

        m_etalonPayloads.append(packet);

        ui->textEdit->append("Еталон: Пакет №1 (ID 0x16) збережено");
    }

    // ===== GET_SPECTRE_REQ (0x8B): блок 09 — старт, блок 00 — запит спектра =====
    else if (command == 0x8B) {
        quint8 block = static_cast<quint8>(receiveBuff[5]);
        if (block == 0x09) {
            m_spectrumStartTime = QDateTime::currentSecsSinceEpoch();
            m_prevSpectrumRequestTime = m_spectrumStartTime;

            response.append(QByteArray::fromHex("55AA70"));
            response.append(static_cast<char>(address));
            response.append(QByteArray::fromHex("8B098C01"));
            response.append(QByteArray(2067, 0x00));

            int crc = calcCrc(response);
            response.append(static_cast<char>(crc & 0xFF));

            QString hexResponse = response.toHex().toUpper();
            ui->textEdit->append("БДБГ відправлено: " + hexResponse);

            writeBdbgData(bdbg_port, response);

            ui->textEdit->append("БДБГ: відповідь на старт набору спектра відправлено");
        } else if (block == 0x00) {
            if (m_etalonSpectrumReply.isEmpty()) {
                ui->textEdit->append("БДБГ: еталонний спектр не завантажено, відправляю нулі");
                response.fill(0, 2076);
            } else {
                response = m_etalonSpectrumReply;
            }

            response[0] = 0x55;
            response[1] = 0xAA;
            response[2] = 0x70;
            response[3] = static_cast<char>(address);
            response[4] = 0x8B;
            response[5] = 0x00;

            qint64 currentTime = QDateTime::currentSecsSinceEpoch();
            quint16 accTime = static_cast<quint16>(currentTime - m_prevSpectrumRequestTime);
            m_prevSpectrumRequestTime = currentTime;

            response[2054] = static_cast<char>(accTime & 0xFF);
            response[2055] = static_cast<char>((accTime >> 8) & 0xFF);

            float pedFloat = static_cast<float>(pedValue);
            const char* pedBytes = reinterpret_cast<const char*>(&pedFloat);
            for (int i = 0; i < 4; ++i) {
                response[2056 + i] = pedBytes[i];
            }

            QByteArray dataForCrc = response.left(2075);
            int crc = calcCrc(dataForCrc);
            response[2075] = static_cast<char>(crc & 0xFF);

            QString hexResponse = response.toHex().toUpper();
            ui->textEdit->append("БДБГ відправлено (спектр): " + hexResponse);

            writeBdbgData(bdbg_port, response);

            ui->textEdit->append(QString("БДБГ: відповідь на запит спектра відправлено (Acc Time=%1 сек, ПЕД=%2)")
                                     .arg(accTime).arg(pedValue, 0, 'f', 3));
        } else {
            ui->textEdit->append(QString("БДБГ: невідомий блок 0x%1 для команди 0x8B").arg(block, 2, 16, QChar('0')));
        }
    }

    else {
        ui->textEdit->append(QString("БДБГ: невідома команда 0x%1")
                                 .arg(command, 2, 16, QChar('0')));
    }
}



















// ============================================================
// ОТПРАВКА ДАННЫХ В ПОРТ БДБГ
// ============================================================
int MainWindow::writeBdbgData(QSerialPort *port, const QByteArray &data)
{
    if (!port || !port->isOpen()) {
        return -1;
    }

    QString hexResponse = data.toHex().toUpper();
    ui->textEdit->append("БДБГ-1 відправлено: " + hexResponse + '\n');

    int bytesWritten = port->write(data);
    if (bytesWritten > 0) {
        port->waitForBytesWritten(100);
    }
    return bytesWritten;
}

void MainWindow::on_pushButton_clear_clicked()
{
    ui->textEdit->clear();
}

int MainWindow::calcCrc(const QByteArray &data)
{
    int crc = 0;
    for (int i = 0; i < data.size(); ++i) {
        crc += static_cast<unsigned char>(data[i]);
        if ((crc & 0x0100) == 0x0100) {
            crc = (crc & 0xFF) + 1;
        }
    }
    return crc;
}

// ============================================================
// LIDAR
// ============================================================
void MainWindow::onLidarPortSelected(int index)
{
    Q_UNUSED(index)

    QString portName = ui->comboBox_LIDAR->currentText();
    if (portName == "NOT SELECT" || portName.isEmpty()) {
        return;
    }

    if (lidar_port->isOpen()) {
        lidar_port->close();
        if (lidar_timer->isActive()) {
            lidar_timer->stop();
            lidar_is_active = false;
        }
    }

    lidar_port->setPortName(portName);
    lidar_port->setBaudRate(QSerialPort::Baud115200);
    lidar_port->setDataBits(QSerialPort::Data8);
    lidar_port->setParity(QSerialPort::NoParity);
    lidar_port->setStopBits(QSerialPort::OneStop);
    lidar_port->setFlowControl(QSerialPort::NoFlowControl);

    if (lidar_port->open(QSerialPort::ReadWrite)) {
        ui->textEdit->append(QString("LIDAR: порт %1 відкрито").arg(portName));
    } else {
        ui->textEdit->append(QString("LIDAR: помилка відкриття порту %1").arg(portName));
    }
}

void MainWindow::onLidarFreqChanged(int index)
{
    if (!lidar_is_active) {
        return;
    }

    if (lidar_timer->isActive()) {
        lidar_timer->stop();
    }

    int intervalMs = 0;
    if (index == 0) {
        intervalMs = 1000;
    } else if (index == 1) {
        intervalMs = 10;
    } else {
        return;
    }

    lidar_timer->start(intervalMs);
    ui->textEdit->append(QString("LIDAR: частоту змінено на %1").arg(ui->comboBox_freq_lidar->currentText()));
}














void MainWindow::onLidarTimerTick()
{
    if (!lidar_is_active) {
        return;
    }

    if (!lidar_port || !lidar_port->isOpen()) {
        return;
    }

    if (ui->checkBox_not_ans_lidar->isChecked()) {
        return;
    }

    PointData currentPoint = m_surveyArea.getCurrentPoint();
    double heightMeters = currentPoint.getHeight();

    int distanceCm = static_cast<int>(heightMeters * 100.0);
    if (distanceCm > 0xFFFF) distanceCm = 0xFFFF;
    if (distanceCm < 0) distanceCm = 0;

    QString mode = ui->comboBox_imit_mode_lidar->currentText();
    quint16 distance = static_cast<quint16>(distanceCm);
    quint16 strength = 0x0FFF;

    if (mode == "Об'єкт не виявлено") {
        distance = 0;
        strength = 0x00D2;
    } else if (mode == "Слабке відбиття") {
        strength = 0x00D2;
    } else if (mode == "Сильне відбиття") {
        strength = 0xBFFF;
    } else if (mode == "Об'єкт дуже близько") {
        distance = 0x0010;
    } else if (mode == "Об'єкт занадто далеко") {
        distance = 0xFFFF;
    }

    QByteArray packet;
    packet.resize(9);
    packet[0] = 0x59;
    packet[1] = 0x59;
    packet[2] = static_cast<char>(distance & 0xFF);
    packet[3] = static_cast<char>((distance >> 8) & 0xFF);
    packet[4] = static_cast<char>(strength & 0xFF);
    packet[5] = static_cast<char>((strength >> 8) & 0xFF);
    packet[6] = 0x00;
    packet[7] = 0x00;

    quint8 checksum = 0;
    for (int i = 0; i < 8; ++i) {
        checksum += static_cast<quint8>(packet[i]);
    }
    checksum = checksum & 0xFF;

    if (ui->checkBox_uncorrect_lidar->isChecked()) {
        checksum = checksum ^ 0xFF;
    }

    packet[8] = static_cast<char>(checksum);

    m_lastLidarDistance = distance;
    m_lastLidarStrength = strength;

    QString hexPacket = packet.toHex().toUpper();
    ui->textEdit->append("LIDAR відправлено: " + hexPacket);
    lidar_port->write(packet);
}

















// ============================================================
// GPS
// ============================================================
void MainWindow::onGpsPortSelected(int index)
{
    Q_UNUSED(index)

    QString portName = ui->comboBox_GPS->currentText();
    if (portName == "NOT SELECT" || portName.isEmpty()) {
        return;
    }

    if (gps_port->isOpen()) {
        gps_port->close();
        if (gps_timer->isActive()) {
            gps_timer->stop();
            gps_is_active = false;
        }
    }

    gps_port->setPortName(portName);
    gps_port->setBaudRate(QSerialPort::Baud115200);
    gps_port->setDataBits(QSerialPort::Data8);
    gps_port->setParity(QSerialPort::NoParity);
    gps_port->setStopBits(QSerialPort::OneStop);
    gps_port->setFlowControl(QSerialPort::NoFlowControl);

    ui->label_gps_port->setText(portName);

    if (gps_port->open(QSerialPort::ReadWrite)) {
        ui->textEdit->append(QString("GPS: порт %1 відкрито").arg(portName));
    } else {
        ui->textEdit->append(QString("GPS: помилка відкриття порту %1").arg(portName));
    }
}

QString MainWindow::formatLatLon(double value, bool isLatitude)
{
    double absValue = qAbs(value);
    int degrees = static_cast<int>(absValue);
    double minutes = (absValue - degrees) * 60.0;

    QString minStr = QString("%1").arg(minutes, 0, 'f', 4);
    if (minutes < 10.0)
        minStr = "0" + minStr;

    QString degStr;
    if (isLatitude) {
        degStr = QString("%1").arg(degrees, 2, 10, QChar('0'));
    } else {
        degStr = QString("%1").arg(degrees, 3, 10, QChar('0'));
    }

    return degStr + minStr;
}

QString MainWindow::calculateNmeaChecksum(const QString &sentence)
{
    int start = sentence.indexOf('$');
    if (start == -1) {
        return "00";
    }

    quint8 checksum = 0;
    for (int i = start + 1; i < sentence.length(); ++i) {
        if (sentence[i] == '*')
            break;
        checksum ^= static_cast<quint8>(sentence[i].toLatin1());
    }

    return QString("%1").arg(checksum, 2, 16, QChar('0')).toUpper();
}


// ============================================================
// РАДИОЛИНИЯ
// ============================================================
void MainWindow::onRadiolinePortSelected(int index)
{
    Q_UNUSED(index)

    QString portName = ui->comboBox_radioline->currentText();
    if (portName == "NOT SELECT" || portName.isEmpty()) {
        return;
    }

    if (logger_rx_port->isOpen()) {
        logger_rx_port->close();
    }

    logger_rx_port->setPortName(portName);
    logger_rx_port->setBaudRate(QSerialPort::Baud115200);
    logger_rx_port->setDataBits(QSerialPort::Data8);
    logger_rx_port->setParity(QSerialPort::NoParity);
    logger_rx_port->setStopBits(QSerialPort::OneStop);
    logger_rx_port->setFlowControl(QSerialPort::NoFlowControl);

    ui->label_radioline_port->setText(portName);

    if (logger_rx_port->open(QSerialPort::ReadWrite)) {
        ui->textEdit->append(QString("Радіолінія: порт %1 відкрито").arg(portName));
    } else {
        ui->textEdit->append(QString("Радіолінія: помилка відкриття порту %1").arg(portName));
    }
}













void MainWindow::onGpsTimerTick()
{
    if (!gps_is_active) {
        return;
    }

    if (!gps_port || !gps_port->isOpen()) {
        return;
    }

    if (ui->checkBox_not_ans_gps->isChecked()) {
        return;
    }

    PointData currentPoint = m_surveyArea.getCurrentPoint();
    double latitude = currentPoint.getLatitude();
    double longitude = currentPoint.getLongitude();
    double altitude = currentPoint.getHeight();

    if (ui->checkBox_antenna_fault_gps->isChecked()) {
        latitude = 0.0;
        longitude = 0.0;
        altitude = 0.0;
    }

    if (ui->checkBox_spoofing_gps->isChecked()) {
        QRandomGenerator *rand = QRandomGenerator::global();
        double latOffset = (rand->generateDouble() * 20.0) - 10.0;
        double lonOffset = (rand->generateDouble() * 20.0) - 10.0;
        latitude += latOffset;
        longitude += lonOffset;
        if (latitude > 90.0) latitude = 90.0;
        if (latitude < -90.0) latitude = -90.0;
        if (longitude > 180.0) longitude = 180.0;
        if (longitude < -180.0) longitude = -180.0;
    }

    QDateTime utcNow = QDateTime::currentDateTimeUtc();
    QTime utcTime = utcNow.time();
    if (ui->checkBox_spoofing_gps->isChecked()) {
        QRandomGenerator *rand = QRandomGenerator::global();
        int hourOffset = (rand->generate() % 3) + 1;
        if (rand->generate() % 2 == 0) hourOffset = -hourOffset;
        utcTime = utcTime.addSecs(hourOffset * 3600);
    }
    QString utcTimeStr = utcTime.toString("hhmmss.00");

    QString latNmea = formatLatLon(latitude, true);
    QString lonNmea = formatLatLon(longitude, false);
    char latDir = (latitude >= 0) ? 'N' : 'S';
    char lonDir = (longitude >= 0) ? 'E' : 'W';

    int satellites = ui->spinBox_satellites_gps->value();
    double hdop = ui->doubleSpinBox_hdop_gps->value();
    QString status = ui->checkBox_status_valid_gps->isChecked() ? "A" : "V";
    QString mode = ui->checkBox_mode_valid_gps->isChecked() ? "A" : "N";

    QString gga = QString("$GNGGA,%1,%2,%3,%4,%5,1,%6,%7,%8,M,46.9,M,,*")
                      .arg(utcTimeStr)
                      .arg(latNmea).arg(latDir)
                      .arg(lonNmea).arg(lonDir)
                      .arg(satellites, 2, 10, QChar('0'))
                      .arg(hdop, 0, 'f', 1)
                      .arg(altitude, 0, 'f', 1);
    gga.replace("*", "*" + calculateNmeaChecksum(gga));

    QString gll = QString("$GNGLL,%1,%2,%3,%4,%5,%6,%7,*")
                      .arg(latNmea).arg(latDir)
                      .arg(lonNmea).arg(lonDir)
                      .arg(utcTimeStr)
                      .arg(status)
                      .arg(mode);
    gll.replace("*", "*" + calculateNmeaChecksum(gll));

    QString dateStr = utcNow.toString("ddMMyy");
    QString rmc = QString("$GNRMC,%1,%2,%3,%4,%5,%6,0.0,0.0,%7,,,A*")
                      .arg(utcTimeStr)
                      .arg(status)
                      .arg(latNmea).arg(latDir)
                      .arg(lonNmea).arg(lonDir)
                      .arg(dateStr);
    rmc.replace("*", "*" + calculateNmeaChecksum(rmc));

    m_lastGpsLatitude = latitude;
    m_lastGpsLongitude = longitude;
    m_lastGpsUtcTime = utcTime;

    GpsRecord rec;
    rec.utcTime = utcTime;
    rec.lat = latitude;
    rec.lon = longitude;
    m_gpsJournal.append(rec);

    static bool firstPacket = true;
    if (!firstPacket) {
        ui->textEdit->append("----------------------------------------");
    }
    firstPacket = false;

    if (ui->checkBox_realistic_gps->isChecked()) {
        QRandomGenerator *rand = QRandomGenerator::global();
        QList<int> prnList;
        for (int i = 0; i < qMin(satellites, 12); ++i) {
            prnList.append(rand->bounded(1, 33));
        }
        while (prnList.size() < 12) prnList.append(0);
        QString prnStr;
        for (int prn : prnList) {
            if (prn == 0) prnStr += ",";
            else prnStr += QString::number(prn) + ",";
        }
        if (prnStr.endsWith(",")) prnStr.chop(1);

        double pdop = hdop * 1.1;
        double vdop = hdop * 0.7;
        QString gsa = QString("$GNGSA,A,1,%1,%2,%3,%4*")
                          .arg(prnStr)
                          .arg(hdop, 0, 'f', 2)
                          .arg(pdop, 0, 'f', 2)
                          .arg(vdop, 0, 'f', 2);
        gsa.replace("*", "*" + calculateNmeaChecksum(gsa));

        int gpsSatellites = qMin(satellites, 4);
        QString gsvGps = QString("$GPGSV,1,1,%1").arg(gpsSatellites);
        for (int i = 0; i < gpsSatellites; ++i) {
            int prn = rand->bounded(1, 33);
            int elevation = rand->bounded(10, 90);
            int azimuth = rand->bounded(0, 360);
            int snr = rand->bounded(35, 50);
            gsvGps += QString(",%1,%2,%3,%4").arg(prn, 2, 10, QChar('0'))
                          .arg(elevation)
                          .arg(azimuth)
                          .arg(snr);
        }
        gsvGps += "*";
        gsvGps.replace("*", "*" + calculateNmeaChecksum(gsvGps));

        int glonassSatellites = qMin(satellites - gpsSatellites, 2);
        if (glonassSatellites < 0) glonassSatellites = 0;
        QString gsvGlonass = QString("$GLGSV,1,1,%1").arg(glonassSatellites);
        for (int i = 0; i < glonassSatellites; ++i) {
            int prn = rand->bounded(65, 97);
            int elevation = rand->bounded(10, 90);
            int azimuth = rand->bounded(0, 360);
            int snr = rand->bounded(35, 50);
            gsvGlonass += QString(",%1,%2,%3,%4").arg(prn, 2, 10, QChar('0'))
                              .arg(elevation)
                              .arg(azimuth)
                              .arg(snr);
        }
        gsvGlonass += "*";
        gsvGlonass.replace("*", "*" + calculateNmeaChecksum(gsvGlonass));

        QString vtg = QString("$GNVTG,0.0,T,0.0,M,0.0,N,0.0,K,N*");
        vtg += calculateNmeaChecksum(vtg);

        QString txt;
        if (rand->generateDouble() < 0.3) {
            txt = QString("$GNTXT,01,01,01,More than 100 frame errors, UART RX was disabled*");
            txt += calculateNmeaChecksum(txt);
        }

        gps_port->write((gga + "\r\n").toUtf8());
        gps_port->write((gll + "\r\n").toUtf8());
        gps_port->write((rmc + "\r\n").toUtf8());
        gps_port->write((gsa + "\r\n").toUtf8());
        gps_port->write((gsvGps + "\r\n").toUtf8());
        if (glonassSatellites > 0) {
            gps_port->write((gsvGlonass + "\r\n").toUtf8());
        }
        gps_port->write((vtg + "\r\n").toUtf8());
        if (!txt.isEmpty()) {
            gps_port->write((txt + "\r\n").toUtf8());
        }

        ui->textEdit->append("GPS (реалістичний): " + gga.trimmed());
        ui->textEdit->append("GPS (реалістичний): " + gll.trimmed());
        ui->textEdit->append("GPS (реалістичний): " + rmc.trimmed());
        ui->textEdit->append("GPS (реалістичний): " + gsa.trimmed());
        ui->textEdit->append("GPS (реалістичний): " + gsvGps.trimmed());
        if (glonassSatellites > 0) {
            ui->textEdit->append("GPS (реалістичний): " + gsvGlonass.trimmed());
        }
        ui->textEdit->append("GPS (реалістичний): " + vtg.trimmed());
        if (!txt.isEmpty()) {
            ui->textEdit->append("GPS (реалістичний): " + txt.trimmed());
        }

    } else {
        gps_port->write((gga + "\r\n").toUtf8());
        gps_port->write((gll + "\r\n").toUtf8());
        gps_port->write((rmc + "\r\n").toUtf8());

        ui->textEdit->append("GPS (спрощений): " + gga.trimmed());
        ui->textEdit->append("GPS (спрощений): " + gll.trimmed());
        ui->textEdit->append("GPS (спрощений): " + rmc.trimmed());
    }
}





// ============================================================
// ИНФОРМАЦИОННЫЕ КНОПКИ
// ============================================================
void MainWindow::on_pushButton_show_gps_packages_clicked()
{
    ui->textEdit->clear();
    ui->textEdit->append("\n========== GPS ПАКЕТИ (ОПИС) ==========\n");
    ui->textEdit->append("Частота: 1 Гц (один раз на секунду)");
    ui->textEdit->append("Протокол: NMEA 0183");
    ui->textEdit->append("Швидкість порту: 4800 бод");
    ui->textEdit->append("Формат: ASCII, кожен пакет закінчується CR+LF (\\r\\n)");
    ui->textEdit->append("Контрольна сума: XOR (всі байти між $ та *)\n");
    // ... остальное (сокращено для краткости, но в коде должно быть полное описание)
    ui->textEdit->append("\n========== КІНЕЦЬ ОПИСУ ==========\n");
}

void MainWindow::on_pushButton_show_lidar_packages_clicked()
{
    ui->textEdit->clear();
    ui->textEdit->append("\n========== LIDAR ПАКЕТИ (ОПИС) ==========\n");
    ui->textEdit->append("Протокол: власний (8 байт даних + 1 байт контрольна сума)");
    // ... и т.д.
    ui->textEdit->append("\n========== КІНЕЦЬ ОПИСУ ==========\n");
}

void MainWindow::on_pushButton_show_instruction_clicked()
{
    ui->textEdit->clear();
    ui->textEdit->append("====================================================");
    ui->textEdit->append("              ІНСТРУКЦІЯ З РОБОТИ З ІМІТАТОРОМ");
    ui->textEdit->append("====================================================");
    // ...
}
















// ============================================================
// ЗАГРУЗКА ДАННЫХ С SD-КАРТЫ (CSV-ФОРМАТ)
// ============================================================
// Формат каждой строки файла:
//   CommandID;Length;Payload\n
// Где:
//   CommandID - 2 HEX-символа (1 байт)
//   Length    - 4 HEX-символа (2 байта, uint16_t)
//   Payload   - N×2 HEX-символов (N байт)
//   Разделитель полей - точка с запятой ';'
//   Конец строки - '\n'
//
// Пример строки для Пакета №5 (ID 0x13, Length 11):
//   13;000B;A1B2C3D4E5F6...\n
//
// После чтения файла формируется m_sdPayloads — список бинарных
// пакетов в формате [Command ID (1 байт)] [Payload (N байт)]
// ============================================================
void MainWindow::onLoadSdCardClicked()
{
    // Открываем диалог выбора файла
    QString filePath = QFileDialog::getOpenFileName(
        this,
        "Виберіть CSV-файл з SD-карти",
        QDir::homePath(),
        "CSV-файли (*.csv);;Текстові файли (*.txt);;Усі файли (*.*)"
        );

    if (filePath.isEmpty()) {
        ui->textEdit->append("SD-карта: файл не вибрано");
        return;
    }

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        ui->textEdit->append("SD-карта: помилка відкриття файлу");
        return;
    }

    // Очищаем предыдущие данные
    m_sdPayloads.clear();

    QTextStream in(&file);
    int lineNumber = 0;
    int loadedCount = 0;
    int errorCount = 0;

    // Читаем файл построчно
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        lineNumber++;

        // Пропускаем пустые строки
        if (line.isEmpty()) {
            continue;
        }

        // Разбиваем строку по ',' на поля
        QStringList parts = line.split(',');
        if (parts.size() < 2) {
            ui->textEdit->append(QString("SD-карта: помилка формату рядка %1: %2")
                                     .arg(lineNumber).arg(line));
            errorCount++;
            continue;
        }

        // --- 1. Парсим Command ID (2 HEX-символа → 1 байт) ---
        bool ok;
        QString idStr = parts[0].trimmed();
        if (idStr.size() != 2) {
            ui->textEdit->append(QString("SD-карта: помилка ID у рядку %1").arg(lineNumber));
            errorCount++;
            continue;
        }
        uint8_t commandId = static_cast<uint8_t>(idStr.toUInt(&ok, 16));
        if (!ok) {
            ui->textEdit->append(QString("SD-карта: помилка ID у рядку %1").arg(lineNumber));
            errorCount++;
            continue;
        }

        // --- 2. Парсим Payload (N полей по 2 HEX-символа) ---
        QByteArray payload;
        bool parseError = false;
        for (int i = 1; i < parts.size(); ++i) {
            QString byteStr = parts[i].trimmed();
            if (byteStr.size() != 2) {
                parseError = true;
                break;
            }
            bool okByte;
            uint8_t byteVal = static_cast<uint8_t>(byteStr.toUInt(&okByte, 16));
            if (!okByte || byteVal > 0xFF) {
                parseError = true;
                break;
            }
            payload.append(static_cast<char>(byteVal));
        }

        if (parseError) {
            ui->textEdit->append(QString("SD-карта: помилка Payload у рядку %1").arg(lineNumber));
            errorCount++;
            continue;
        }

        // --- 3. Формируем бинарный пакет: Command ID (1 байт) + Payload (N байт) ---
        QByteArray packet;
        packet.append(static_cast<char>(commandId));
        packet.append(payload);

        // --- 4. Сохраняем в m_sdPayloads ---
        m_sdPayloads.append(packet);
        loadedCount++;
    }

    file.close();

    // Выводим информацию о загруженных данных
    ui->textEdit->append(QString("SD-карта: завантажено %1 пакетів з файлу %2 (помилок: %3)")
                             .arg(loadedCount)
                             .arg(QFileInfo(filePath).fileName())
                             .arg(errorCount));
}


















// ============================================================
// СРАВНЕНИЕ ТРЁХ НАБОРОВ ДАННЫХ: ЭТАЛОН, РАДИОЛИНИЯ, SD-КАРТА
// ============================================================
// Сравниваем поэлементно (побайтово) три списка:
//   m_etalonPayloads    — эталон (сформирован имитатором)
//   m_radiolinePayloads — принято по радиолинии
//   m_sdPayloads        — прочитано с SD-карты
//
// Все три набора хранятся в бинарном виде:
//   [Command ID (1 байт)] [Payload (N байт)]
// ============================================================
void MainWindow::onCompareDataClicked()
{
    ui->textEdit->append("\n========== ПОРІВНЯННЯ ДАНИХ ==========\n");

    // --- Проверка наличия данных ---
    if (m_etalonPayloads.isEmpty()) {
        ui->textEdit->append("Помилка: немає еталонних даних");
        return;
    }

    if (m_radiolinePayloads.isEmpty()) {
        ui->textEdit->append("Помилка: немає даних з радіолінії");
        return;
    }

    if (m_sdPayloads.isEmpty()) {
        ui->textEdit->append("Помилка: немає даних з SD-карти");
        return;
    }

    // --- Вывод количества пакетов в каждом наборе ---
    ui->textEdit->append(QString("Еталон:     %1 пакетів").arg(m_etalonPayloads.size()));
    ui->textEdit->append(QString("Радіолінія: %1 пакетів").arg(m_radiolinePayloads.size()));
    ui->textEdit->append(QString("SD-карта:   %1 пакетів").arg(m_sdPayloads.size()));
    ui->textEdit->append("----------------------------------------");

    // --- ШАГ 1: Сравнение SD-карты и радиолинии ---
    ui->textEdit->append("--- 1. ПОРІВНЯННЯ SD-КАРТИ ТА РАДІОЛІНІЇ ---");

    int minCount = qMin(m_sdPayloads.size(), m_radiolinePayloads.size());
    int errors = 0;

    for (int i = 0; i < minCount; ++i) {
        if (m_sdPayloads[i] != m_radiolinePayloads[i]) {
            errors++;
            ui->textEdit->append(QString("  Розбіжність у пакеті %1").arg(i + 1));
        }
    }

    if (m_sdPayloads.size() != m_radiolinePayloads.size()) {
        ui->textEdit->append(QString("  Кількість пакетів не співпадає: SD=%1, Радіо=%2")
                                 .arg(m_sdPayloads.size()).arg(m_radiolinePayloads.size()));
    }

    if (errors == 0 && m_sdPayloads.size() == m_radiolinePayloads.size()) {
        ui->textEdit->append("РЕЗУЛЬТАТ: SD ↔ Радіолінія — OK");
    } else {
        ui->textEdit->append(QString("РЕЗУЛЬТАТ: SD ↔ Радіолінія — %1 розбіжностей").arg(errors));
    }

    // --- ШАГ 2: Сравнение эталона и SD-карты ---
    ui->textEdit->append("\n--- 2. ПОРІВНЯННЯ ЕТАЛОНУ ТА SD-КАРТИ ---");

    minCount = qMin(m_etalonPayloads.size(), m_sdPayloads.size());
    errors = 0;

    for (int i = 0; i < minCount; ++i) {
        if (m_etalonPayloads[i] != m_sdPayloads[i]) {
            errors++;
            ui->textEdit->append(QString("  Розбіжність у пакеті %1").arg(i + 1));
        }
    }

    if (m_etalonPayloads.size() != m_sdPayloads.size()) {
        ui->textEdit->append(QString("  Кількість пакетів не співпадає: Еталон=%1, SD=%2")
                                 .arg(m_etalonPayloads.size()).arg(m_sdPayloads.size()));
    }

    if (errors == 0 && m_etalonPayloads.size() == m_sdPayloads.size()) {
        ui->textEdit->append("РЕЗУЛЬТАТ: Еталон ↔ SD — OK");
    } else {
        ui->textEdit->append(QString("РЕЗУЛЬТАТ: Еталон ↔ SD — %1 розбіжностей").arg(errors));
    }

    ui->textEdit->append("\n========== КІНЕЦЬ ПОРІВНЯННЯ ==========\n");
}












// ============================================================
// ОБРАБОТЧИК РАДИОЛИНИИ (ПРИЁМ ПАКЕТОВ)
// ============================================================
void MainWindow::onRadiolineDataReceived()
{
    if (!logger_rx_port || !logger_rx_port->isOpen()) {
        return;
    }

    m_radioBuffer.append(logger_rx_port->readAll());

    while (true) {
        // --- 1. Ищем маркер начала AA 55 AA 55 ---
        int startPos = -1;
        for (int i = 0; i <= m_radioBuffer.size() - 4; ++i) {
            if (static_cast<unsigned char>(m_radioBuffer[i])     == 0xAA &&
                static_cast<unsigned char>(m_radioBuffer[i + 1]) == 0x55 &&
                static_cast<unsigned char>(m_radioBuffer[i + 2]) == 0xAA &&
                static_cast<unsigned char>(m_radioBuffer[i + 3]) == 0x55) {
                startPos = i;
                break;
            }
        }

        // Маркер не найден — оставляем последние 3 байта (могут быть началом маркера)
        if (startPos == -1) {
            if (m_radioBuffer.size() > 3) {
                m_radioBuffer.remove(0, m_radioBuffer.size() - 3);
            }
            break;
        }

        // Всё до маркера — отбрасываем (в т.ч. 00 00)
        if (startPos > 0) {
            m_radioBuffer.remove(0, startPos);
        }

        // --- 2. Проверяем, хватает ли байт для заголовка (4 + 1 + 2 = 7) ---
        if (m_radioBuffer.size() < 7) {
            break; // ждём ещё данные
        }

        quint8 id = static_cast<quint8>(m_radioBuffer[4]);
        quint16 length = static_cast<quint8>(m_radioBuffer[5]) |
                         (static_cast<quint8>(m_radioBuffer[6]) << 8);

        // --- 2a. Проверка допустимости Length ---
        if (length == 0 || length > 2100) {
            m_radioBuffer.remove(0, 1);
            continue;
        }

        int fullSize = 4 + 1 + 2 + length + 2 + 4; // маркер + ID + Length + Payload + CRC + концевик

        // --- 3. Проверяем, что весь пакет уже в буфере ---
        if (m_radioBuffer.size() < fullSize) {
            break; // ждём ещё данные
        }

        // --- 4. Проверяем концевик на ожидаемом месте ---
        int endPos = 4 + 1 + 2 + length + 2;
        if (static_cast<quint8>(m_radioBuffer[endPos])     != 0x0D ||
            static_cast<quint8>(m_radioBuffer[endPos + 1]) != 0x0A ||
            static_cast<quint8>(m_radioBuffer[endPos + 2]) != 0x0D ||
            static_cast<quint8>(m_radioBuffer[endPos + 3]) != 0x0A) {
            // Концевик не на месте — пакет битый, сдвигаемся на 1 байт и ищем маркер заново
            m_radioBuffer.remove(0, 1);
            continue;
        }

        // --- 5. Вырезаем пакет ---
        QByteArray packet = m_radioBuffer.left(fullSize);

        // --- 6. Проверяем CRC ---
        QByteArray dataForCrc = packet.mid(4, 1 + 2 + length);
        uint16_t calculatedCrc = calculateCrc16(dataForCrc);

        int crcPos = 4 + 1 + 2 + length;
        uint16_t receivedCrc = static_cast<quint8>(packet[crcPos]) |
                               (static_cast<quint8>(packet[crcPos + 1]) << 8);

        if (receivedCrc != calculatedCrc) {
            // CRC не совпал — сдвигаемся на 1 байт и ищем маркер заново
            m_radioBuffer.remove(0, 1);
            continue;
        }

        // --- 7. Удаляем разобранный пакет из буфера ---
        m_radioBuffer.remove(0, fullSize);

        // --- 8. Проверка на копию пачки (п. 16) ---
        qint64 now = QDateTime::currentMSecsSinceEpoch();
        bool isDuplicate = (!m_lastRadioPacket.isEmpty() &&
                            packet == m_lastRadioPacket &&
                            (now - m_lastRadioPacketTime) <= 1000);

        if (isDuplicate) {
            // Это копия — обновляем время последней копии, но не обрабатываем
            m_lastRadioPacketTime = now;
            continue;
        }

        // Новая пачка: запоминаем пакет и время
        m_lastRadioPacket = packet;
        m_lastRadioPacketTime = now;

        // --- 9. Сохранение для сверки (п. 17) ---
        // Сохраняем ID19(0x13), ID20(0x14), ID22(0x16), ID23(0x17),
        //          ID25(0x19), ID27(0x1B), ID28(0x1C)
        if (id == 0x13 || id == 0x14 || id == 0x16 || id == 0x17 ||
            id == 0x19 || id == 0x1B || id == 0x1C) {
            QByteArray savedPacket;
            savedPacket.append(static_cast<char>(id));
            savedPacket.append(packet.mid(7, length));
            m_radiolinePayloads.append(savedPacket);
            ui->textEdit_radioline->append(QString("Радіолінія: збережено пакет ID 0x%1")
                                               .arg(id, 2, 16, QChar('0')));
        }

        // --- 10. Обработка пакета (вывод в textEdit_radioline) ---
        processDroneResponse(packet);
    }
}



// ============================================================
// ВСПОМОГАТЕЛЬНЫЕ МЕТОДЫ
// ============================================================
void MainWindow::generatePolygonPoints(double startLat, double startLon, double sideKm)
{
    m_polygonPoints.clear();

    double sideLat = m_surveyArea.kmToLatitudeDegrees(sideKm);
    double sideLon = m_surveyArea.kmToLongitudeDegrees(sideKm, startLat);

    double eps = 0.04;

    double latMax = startLat + eps;
    double latMin = startLat - sideLat - eps;
    double lonMin = startLon - eps;
    double lonMax = startLon + sideLon + eps;

    m_polygonPoints.append(PointData(latMax, lonMin, 0, 0, 0)); // верхняя левая
    m_polygonPoints.append(PointData(latMax, lonMax, 0, 0, 0)); // верхняя правая
    m_polygonPoints.append(PointData(latMin, lonMax, 0, 0, 0)); // нижняя правая
    m_polygonPoints.append(PointData(latMin, lonMin, 0, 0, 0)); // нижняя левая
}

bool MainWindow::isPointInPolygon(const PointData &point, const QList<PointData> &polygon)
{
    if (polygon.size() < 3) {
        return false;
    }

    auto crossProduct = [](const PointData &p1, const PointData &p2, const PointData &p3) -> double {
        double x1 = p2.getLatitude() - p1.getLatitude();
        double y1 = p2.getLongitude() - p1.getLongitude();
        double x2 = p3.getLatitude() - p1.getLatitude();
        double y2 = p3.getLongitude() - p1.getLongitude();
        return x1 * y2 - y1 * x2;
    };

    double firstSign = crossProduct(polygon[0], polygon[1], point);
    if (qAbs(firstSign) < 1e-10) {
        return true;
    }

    for (int i = 1; i < polygon.size(); ++i) {
        int next = (i + 1) % polygon.size();
        double sign = crossProduct(polygon[i], polygon[next], point);
        if (qAbs(sign) < 1e-10) {
            return true;
        }
        if (sign * firstSign < 0) {
            return false;
        }
    }

    return true;
}

void MainWindow::generatePathPoints(double startLat, double startLon,
                                    double endLat, double endLon,
                                    int count, double height, double bgPed,
                                    QList<PointData> &targetList)
{
    if (count < 2) {
        targetList.append(PointData(startLat, startLon, height, bgPed, bgPed));
        return;
    }

    for (int i = 0; i < count; ++i) {
        double t = static_cast<double>(i) / (count - 1);
        double lat = startLat + (endLat - startLat) * t;
        double lon = startLon + (endLon - startLon) * t;
        targetList.append(PointData(lat, lon, height, bgPed, bgPed));
    }
}

// ============================================================
// CRC16
// ============================================================
uint16_t MainWindow::calculateCrc16(const QByteArray &data)
{
    uint16_t crc = 0xFFFF;

    for (int i = 0; i < data.size(); ++i) {
        crc ^= static_cast<uint16_t>(static_cast<unsigned char>(data[i])) << 8;

        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

// ============================================================
// ОТПРАВКА КОМАНД НА ДРОН
// ============================================================
QByteArray MainWindow::buildCommandPacket(uint8_t commandId, const QByteArray &payload)
{
    QByteArray packet;

    packet.append(static_cast<char>(0xAA));
    packet.append(static_cast<char>(0x55));
    packet.append(static_cast<char>(0xAA));
    packet.append(static_cast<char>(0x55));

    packet.append(static_cast<char>(commandId));

    uint16_t length = static_cast<uint16_t>(payload.size());
    packet.append(static_cast<char>(length & 0xFF));
    packet.append(static_cast<char>((length >> 8) & 0xFF));

    packet.append(payload);

    QByteArray dataForCrc = packet.mid(4);
    uint16_t crc = calculateCrc16(dataForCrc);
    packet.append(static_cast<char>(crc & 0xFF));
    packet.append(static_cast<char>((crc >> 8) & 0xFF));

    packet.append(static_cast<char>(0x0D));
    packet.append(static_cast<char>(0x0A));
    packet.append(static_cast<char>(0x0D));
    packet.append(static_cast<char>(0x0A));

    return packet;
}





















// void MainWindow::sendCommandToDrone(uint8_t commandId, const QByteArray &payload)
// {
//     if (!logger_rx_port || !logger_rx_port->isOpen()) {
//         ui->textEdit_radioline->append("Помилка: порт радіолінії не відкрито");
//         return;
//     }

//     QByteArray singlePacket = buildCommandPacket(commandId, payload);

//     int repeatFactor = ui->spinBox_packet_dupl->value();
//     if (repeatFactor < 1) repeatFactor = 1;
//     if (repeatFactor > 10) repeatFactor = 10;

//     QByteArray sendData;
//     for (int i = 0; i < repeatFactor; ++i) {
//         sendData.append(singlePacket);
//     }

//     qint64 bytesWritten = logger_rx_port->write(sendData);
//     if (bytesWritten != sendData.size()) {
//         ui->textEdit_radioline->append("Помилка: не вдалося відправити пакет");
//         return;
//     }
//     logger_rx_port->waitForBytesWritten(100);

//     ui->textEdit_radioline->append(QString("Відправлено команду: %1 (ID %2), дублів: %3, розмір: %4 байт")
//                                        .arg(getCommandName(commandId))
//                                        .arg(commandId)
//                                        .arg(repeatFactor)
//                                        .arg(sendData.size()));
// }



void MainWindow::sendCommandToDrone(uint8_t commandId, const QByteArray &payload)
{
    if (!logger_rx_port || !logger_rx_port->isOpen()) {
        ui->textEdit_radioline->append("Помилка: порт радіолінії не відкрито");
        return;
    }

    QByteArray singlePacket = buildCommandPacket(commandId, payload);

    int repeatFactor = ui->spinBox_packet_dupl->value();
    if (repeatFactor < 1) repeatFactor = 1;
    if (repeatFactor > 10) repeatFactor = 10;

    QByteArray sendData;
    for (int i = 0; i < repeatFactor; ++i) {
        sendData.append(singlePacket);
    }

    qint64 bytesWritten = logger_rx_port->write(sendData);
    if (bytesWritten != sendData.size()) {
        ui->textEdit_radioline->append("Помилка: не вдалося відправити пакет");
        return;
    }
    logger_rx_port->waitForBytesWritten(100);

    ui->textEdit_radioline->append(QString("Відправлено команду: %1 (ID %2), дублів: %3, розмір: %4 байт")
                                       .arg(getCommandName(commandId))
                                       .arg(commandId)
                                       .arg(repeatFactor)
                                       .arg(sendData.size()));

    // === Вывод отправленного пакета (без дублей) в HEX-формате ===
    QString hexPacket;
    for (int i = 0; i < singlePacket.size(); ++i) {
        if (i > 0)
            hexPacket += ' ';
        hexPacket += QString("0x%1").arg(static_cast<quint8>(singlePacket.at(i)), 2, 16, QChar('0')).toUpper();
    }
    ui->textEdit_radioline->append(hexPacket);
}


QString MainWindow::getCommandName(uint8_t id)
{
    switch (id) {
    case 1:  return "Провести предполётное тестирование";
    case 2:  return "Продолжить разведку без SD-карты";
    case 3:  return "Не продолжать разведку без SD-карты";
    case 4:  return "Передать параметры разведки";
    case 5:  return "Изменить коэффициент дублирования пакетов";
    case 6:  return "Изменить время между измерениями";
    case 7:  return "Изменить дистанцию между измерениями";
    case 8:  return "Закончить разведку";
    case 9:  return "Изменить режим разведки";
    case 10: return "Произвести измерение (ручной режим)";
    case 11: return "Изменить время набора спектра";
    case 12: return "Прекратить набор спектра";
    case 13: return "Перейти в режим радиомолчания";
    case 14: return "Выйти из режима радиомолчания";
    default: return QString("Неизвестная команда (ID %1)").arg(id);
    }
}

QString MainWindow::getResponseText(uint8_t id)
{
    switch (id) {
    case 1:  return "Пакет принят";
    case 2:  return "Пакет не принят";
    case 3:  return "В текущем режиме команда не выполняется";
    case 4:  return "Аккумулятор разряжен";
    case 5:  return "Аккумулятор заряжен";
    case 6:  return "Аккумулятор неисправен, разведка невозможна";
    case 7:  return "БДБГ-15 ответил некорректно";
    case 8:  return "БДБГ-15 не ответил";
    case 9:  return "БДБГ-15 протестирован";
    case 10: return "БДБГ-15 вышел из строя, разведка невозможна";
    case 11: return "LIDAR протестирован";
    case 12: return "LIDAR неисправен (нет передачи данных)";
    case 13: return "LIDAR вышел из строя, разведка невозможна";
    case 14: return "GPS вышел из строя, разведка невозможна";
    case 15: return "GPS неисправен или не вышел в режим приёма";
    case 16: return "SD-карта неисправна";
    case 17: return "Система не готова к проведению разведки";
    case 18: return "Система радиационной разведки исправна";
    case 19: return "Координата, время, вне зоны";
    case 20: return "Координата, время, в области";
    case 21: return "Измерения для данной точки уже проведены";
    case 22: return "Данные (Пакет №1)";
    case 23: return "Спектр (Пакет №2)";
    case 24: return "Зафиксирован спуфинг, разведка невозможна";
    case 25: return "Дальнейшее проведение разведки невозможно, прибор ...";
    case 26: return "Не удалось синхронизировать RTC";
    case 27: return "Дані початку набору спектру";
    case 28: return "Не вдалось почати набір спектра, БДБГ вийшов з ладу";
    case 29: return "Прошу выбрати режим розвідки";
    case 70: return "Перешёл в состояние COMMAND_INFO";
    case 71: return "Перешёл в состояние REQ_BATTERY";
    case 72: return "Перешёл в состояние BATTERY_PROC";
    case 73: return "Перешёл в состояние BATTERY_ERROR_PROC";
    case 74: return "Перешёл в состояние REQ_BDBG";
    case 75: return "Перешёл в состояние BDBG_PROC";
    case 76: return "Перешёл в состояние BDBG_ERROR_PROC";
    case 77: return "Перешёл в состояние REQ_LIDAR";
    case 78: return "Перешёл в состояние LIDAR_PROC";
    case 79: return "Перешёл в состояние LIDAR_ERROR_PROC";
    case 80: return "Перешёл в состояние REQ_GPS";
    case 81: return "Перешёл в состояние GPS_PROC";
    case 82: return "Перешёл в состояние GPS_ERROR_PROC";
    case 83: return "Перешёл в состояние WORK_SD";
    case 84: return "Перешёл в состояние FLIGHT_CHANGE";
    default: return QString("Неизвестный ответ (ID %1)").arg(id);
    }
}

void MainWindow::on_pushButton_send_comm_clicked()
{
    int commandIndex = ui->comboBox_select_Command->currentIndex();
    uint8_t commandId = static_cast<uint8_t>(commandIndex + 1);

    QByteArray payload;

    switch (commandId) {
    case 1:
    case 2:
    case 3:
    case 8:
    case 10:
    case 12:
    case 13:
    case 14:
        payload.append(static_cast<char>(0x01));
        break;

    case 4: {
        payload.append(static_cast<char>(4));

        for (const PointData &p : m_polygonPoints) {
            float lat = static_cast<float>(p.getLatitude());
            const char* latBytes = reinterpret_cast<const char*>(&lat);
            payload.append(latBytes, sizeof(float));

            float lon = static_cast<float>(p.getLongitude());
            const char* lonBytes = reinterpret_cast<const char*>(&lon);
            payload.append(lonBytes, sizeof(float));
        }

        uint8_t mode = static_cast<uint8_t>(ui->comboBox_rec_mode->currentIndex());
        payload.append(static_cast<char>(mode));

        uint16_t saveTime = static_cast<uint16_t>(ui->spinBox_meas_time->value());
        payload.append(static_cast<char>(saveTime & 0xFF));
        payload.append(static_cast<char>((saveTime >> 8) & 0xFF));

        uint16_t saveDistance = static_cast<uint16_t>(ui->spinBox_meas_dist->value());
        payload.append(static_cast<char>(saveDistance & 0xFF));
        payload.append(static_cast<char>((saveDistance >> 8) & 0xFF));

        uint8_t duplCoeff = static_cast<uint8_t>(ui->spinBox_packet_dupl->value());
        payload.append(static_cast<char>(duplCoeff));

        uint16_t saveSpectre = static_cast<uint16_t>(ui->spinBox_spectre_time->value());
        payload.append(static_cast<char>(saveSpectre & 0xFF));
        payload.append(static_cast<char>((saveSpectre >> 8) & 0xFF));
        break;
    }

    case 5: {
        uint8_t value = static_cast<uint8_t>(ui->spinBox_packet_dupl->value());
        payload.append(static_cast<char>(value));
        break;
    }

    case 6: {
        uint16_t value = static_cast<uint16_t>(ui->spinBox_meas_time->value());
        payload.append(static_cast<char>(value & 0xFF));
        payload.append(static_cast<char>((value >> 8) & 0xFF));
        break;
    }

    case 7: {
        uint16_t value = static_cast<uint16_t>(ui->spinBox_meas_dist->value());
        payload.append(static_cast<char>(value & 0xFF));
        payload.append(static_cast<char>((value >> 8) & 0xFF));
        break;
    }

    case 9: {
        uint8_t mode = static_cast<uint8_t>(ui->comboBox_rec_mode->currentIndex());
        payload.append(static_cast<char>(mode));
        break;
    }

    case 11: {
        uint16_t value = static_cast<uint16_t>(ui->spinBox_spectre_time->value());
        payload.append(static_cast<char>(value & 0xFF));
        payload.append(static_cast<char>((value >> 8) & 0xFF));
        break;
    }

    default:
        ui->textEdit_radioline->append(QString("Невідома команда ID %1").arg(commandId));
        return;
    }

    sendCommandToDrone(commandId, payload);
}

void MainWindow::processDroneResponse(const QByteArray &packet)
{
    if (packet.size() < 13) {
        ui->textEdit_radioline->append("Помилка: пакет занадто короткий");
        return;
    }


    if (static_cast<uint8_t>(packet[0]) != 0xAAU ||
        static_cast<uint8_t>(packet[1]) != 0x55U ||
        static_cast<uint8_t>(packet[2]) != 0xAAU ||
        static_cast<uint8_t>(packet[3]) != 0x55U)
    {
        ui->textEdit_radioline->append("Помилка: невірні стартові байти");
        return;
    }
    uint8_t id = static_cast<uint8_t>(packet[4]);
    uint16_t length = static_cast<uint8_t>(packet[5]) |
                      (static_cast<uint8_t>(packet[6]) << 8);

    if (packet.size() < 7 + length + 6) {
        ui->textEdit_radioline->append("Помилка: пакет занадто короткий для Length");
        return;
    }

    QByteArray payload = packet.mid(7, length);

    int crcPos = 7 + length;
    uint16_t receivedCrc = static_cast<uint8_t>(packet[crcPos]) |
                           (static_cast<uint8_t>(packet[crcPos + 1]) << 8);

    int endPos = crcPos + 2;
    if (packet[endPos] != 0x0D || packet[endPos + 1] != 0x0A ||
        packet[endPos + 2] != 0x0D || packet[endPos + 3] != 0x0A) {
        ui->textEdit_radioline->append("Помилка: невірні кінцеві байти");
        return;
    }

    QByteArray dataForCrc = packet.mid(4, 3 + length);
    uint16_t calculatedCrc = calculateCrc16(dataForCrc);

    if (receivedCrc != calculatedCrc) {
        ui->textEdit_radioline->append("Неспівпадіння CRC");
        return;
    }

    QString responseText = getResponseText(id);
    QString payloadText;
    if (!payload.isEmpty()) {
        payloadText = formatPayload(payload, id);
    }

    if (payloadText.isEmpty()) {
        ui->textEdit_radioline->append(QString("Відповідь: %1 (ID %2)")
                                           .arg(responseText)
                                           .arg(id));
    } else {
        ui->textEdit_radioline->append(QString("Відповідь: %1 (ID %2) -> %3")
                                           .arg(responseText)
                                           .arg(id)
                                           .arg(payloadText));
    }
}








QString MainWindow::formatPayload(const QByteArray &payload, uint8_t id)
{
    if (payload.isEmpty()) {
        return QString();
    }

    QString result;
    int offset = 0;

    switch (id) {
    case 4:
    case 5: {
        if (payload.size() >= sizeof(float)) {
            float voltage;
            memcpy(&voltage, payload.data(), sizeof(float));
            result = QString::number(voltage, 'f', 2) + " V";
        }
        break;
    }

    case 19:
    case 20: {
        if (payload.size() >= 4 + 4 + 3) {
            float lat, lon;
            uint8_t hours, minutes, seconds;
            memcpy(&lat, payload.data(), sizeof(float));
            memcpy(&lon, payload.data() + 4, sizeof(float));
            hours = static_cast<uint8_t>(payload[8]);
            minutes = static_cast<uint8_t>(payload[9]);
            seconds = static_cast<uint8_t>(payload[10]);
            result = QString("lat=%1, lon=%2, час=%3:%4:%5")
                         .arg(lat, 0, 'f', 6)
                         .arg(lon, 0, 'f', 6)
                         .arg(hours, 2, 10, QChar('0'))
                         .arg(minutes, 2, 10, QChar('0'))
                         .arg(seconds, 2, 10, QChar('0'));
        }
        break;
    }

    case 22: {
        // Payload 31 байт:
        //  [0..3]   ПАЕД float
        //  [4..5]   температура uint16 LE
        //  [6]      состояние
        //  [7..8]   высота LIDAR uint16 LE
        //  [9..10]  мощность LIDAR uint16 LE
        //  [11..14] широта float
        //  [15..18] долгота float
        //  [19..22] высота GPS float
        //  [23]     спутники
        //  [24..27] батарея float
        //  [28..30] время HH MM SS
        if (payload.size() >= 31) {
            float ped, lat, lon, gpsAlt, batteryVoltage;
            uint16_t tempCode, lidarDist, lidarStrength;
            uint8_t status, satellites;
            uint8_t hours, minutes, seconds;

            offset = 0;
            memcpy(&ped, payload.data() + offset, sizeof(float)); offset += 4;
            memcpy(&tempCode, payload.data() + offset, sizeof(uint16_t)); offset += 2;
            status = static_cast<uint8_t>(payload[offset]); offset += 1;
            memcpy(&lidarDist, payload.data() + offset, sizeof(uint16_t)); offset += 2;
            memcpy(&lidarStrength, payload.data() + offset, sizeof(uint16_t)); offset += 2;
            memcpy(&lat, payload.data() + offset, sizeof(float)); offset += 4;
            memcpy(&lon, payload.data() + offset, sizeof(float)); offset += 4;
            memcpy(&gpsAlt, payload.data() + offset, sizeof(float)); offset += 4;
            satellites = static_cast<uint8_t>(payload[offset]); offset += 1;
            memcpy(&batteryVoltage, payload.data() + offset, sizeof(float)); offset += 4;
            hours = static_cast<uint8_t>(payload[offset]);
            minutes = static_cast<uint8_t>(payload[offset + 1]);
            seconds = static_cast<uint8_t>(payload[offset + 2]);

            QString statusBinary = QString("%1").arg(status, 8, 2, QChar('0'));

            result = QString("ПЕД=%1 мкЗв/ч, t(код)=%2, статус=%3, "
                             "LIDAR=%4 см, сила=%5, lat=%6, lon=%7, "
                             "висота GPS=%8 м, супутників=%9, "
                             "батарея=%10 V, час=%11:%12:%13")
                         .arg(ped, 0, 'f', 3)
                         .arg(tempCode)
                         .arg(statusBinary)
                         .arg(lidarDist)
                         .arg(lidarStrength)
                         .arg(lat, 0, 'f', 6)
                         .arg(lon, 0, 'f', 6)
                         .arg(gpsAlt, 0, 'f', 1)
                         .arg(satellites)
                         .arg(batteryVoltage, 0, 'f', 2)
                         .arg(hours, 2, 10, QChar('0'))
                         .arg(minutes, 2, 10, QChar('0'))
                         .arg(seconds, 2, 10, QChar('0'));
        }
        break;
    }

    case 23: {
        // Payload 2055 байт:
        //  [0..2047]  каналы (1024 × uint16)
        //  [2048..2049] AccTime uint16 LE
        //  [2050..2053] ПАЕД float
        //  [2054]     состояние
        if (payload.size() >= 2055) {
            QVector<double> receivedSpectrum(1024);
            for (int i = 0; i < 1024; ++i) {
                uint16_t value;
                memcpy(&value, payload.data() + i * 2, sizeof(uint16_t));
                receivedSpectrum[i] = static_cast<double>(value);
            }

            if (m_spectrumData.isEmpty()) {
                m_spectrumData.resize(1024);
                m_spectrumData.fill(0.0);
            }
            for (int i = 0; i < 1024; ++i) {
                m_spectrumData[i] += receivedSpectrum[i];
            }

            updateSpectrumPlot(m_spectrumData);

            uint16_t spectreTime;
            memcpy(&spectreTime, payload.data() + 2048, sizeof(uint16_t));

            float ped;
            memcpy(&ped, payload.data() + 2050, sizeof(float));

            uint8_t status = static_cast<uint8_t>(payload[2054]);

            double firstChannel = m_spectrumData[0];
            double lastChannel = m_spectrumData[1023];

            result = QString("Спектр: 1024 канали, час набору=%1 с, ПАЕД=%2 мкЗв/ч, "
                             "статус=0x%3, канал0=%4, канал1023=%5")
                         .arg(spectreTime)
                         .arg(ped, 0, 'f', 3)
                         .arg(status, 2, 16, QChar('0'))
                         .arg(firstChannel, 0, 'f', 0)
                         .arg(lastChannel, 0, 'f', 0);
        }
        break;
    }

    case 25: {
        if (payload.size() >= 1) {
            uint8_t deviceType = static_cast<uint8_t>(payload[0]);
            QString deviceName;
            switch (deviceType) {
            case 0: deviceName = "БДБГ"; break;
            case 1: deviceName = "LIDAR"; break;
            case 2: deviceName = "GPS"; break;
            case 3: deviceName = "Аккумулятор"; break;
            default: deviceName = "Невідомий пристрій"; break;
            }
            result = QString("Пристрій: %1").arg(deviceName);
        }
        break;
    }

    case 27: { // Пакет №8 (ID 0x1B) — начальные данные набора спектра
        // Payload 22 байта:
        //   [0..1]   год (2 ASCII)
        //   [2..3]   месяц (2 ASCII)
        //   [4..5]   день (2 ASCII)
        //   [6..7]   часы (2 ASCII)
        //   [8..9]   минуты (2 ASCII)
        //   [10..11] номер спектра uint16 LE
        //   [12..15] широта float
        //   [16..19] долгота float
        //   [20..21] высота uint16 см LE
        if (payload.size() >= 22) {
            // Сброс накопленного спектра для нового набора
            m_spectrumData.fill(0.0, 1024);
            updateSpectrumPlot(m_spectrumData);

            QByteArray yearStr    = payload.mid(0, 2);
            QByteArray monthStr   = payload.mid(2, 2);
            QByteArray dayStr     = payload.mid(4, 2);
            QByteArray hourStr    = payload.mid(6, 2);
            QByteArray minuteStr  = payload.mid(8, 2);

            uint16_t spectreNum;
            memcpy(&spectreNum, payload.data() + 10, sizeof(uint16_t));

            float lat;
            memcpy(&lat, payload.data() + 12, sizeof(float));

            float lon;
            memcpy(&lon, payload.data() + 16, sizeof(float));

            uint16_t heightCm;
            memcpy(&heightCm, payload.data() + 20, sizeof(uint16_t));

            double heightM = heightCm / 100.0;

            QString spectreId = QString::fromUtf8(yearStr + monthStr + dayStr + hourStr + minuteStr);

            ui->lineEdit_ID_spectre->setText(spectreId);
            ui->lineEdit_num_spectre->setText(QString::number(spectreNum));
            ui->lineEdit_coords_spectre->setText(
                QString("%1, %2").arg(lat, 0, 'f', 6).arg(lon, 0, 'f', 6));
            ui->lineEdit_high_spectre->setText(QString::number(heightM, 'f', 2));

            m_spectreNumber = spectreNum;

            result = QString("ID=%1, Номер=%2, lat=%3, lon=%4, Висота=%5 м")
                         .arg(spectreId)
                         .arg(spectreNum)
                         .arg(lat, 0, 'f', 6)
                         .arg(lon, 0, 'f', 6)
                         .arg(heightM, 0, 'f', 2);
        }
        break;
    }

    default:
        result = payload.toHex().toUpper();
        break;
    }

    return result;
}














// ============================================================
// СПЕКТР: НАСТРОЙКА ГРАФИКА И ЗАГРУЗКА ЭТАЛОНА
// ============================================================
void MainWindow::setupSpectrumPlot()
{
    ui->spectrum_widget->xAxis->setLabel("Канал");
    ui->spectrum_widget->yAxis->setLabel("Амплітуда");
    ui->spectrum_widget->xAxis->setRange(0, 1040);
    ui->spectrum_widget->yAxis->setRange(0, 5000);

    ui->spectrum_widget->setBackground(Qt::white);
    ui->spectrum_widget->xAxis->setBasePen(QPen(Qt::black));
    ui->spectrum_widget->yAxis->setBasePen(QPen(Qt::black));
    ui->spectrum_widget->xAxis->setTickLabelColor(Qt::black);
    ui->spectrum_widget->yAxis->setTickLabelColor(Qt::black);
    ui->spectrum_widget->xAxis->setLabelColor(Qt::black);
    ui->spectrum_widget->yAxis->setLabelColor(Qt::black);

    ui->spectrum_widget->setInteraction(QCP::iRangeDrag, true);
    ui->spectrum_widget->setInteraction(QCP::iRangeZoom, true);

    ui->spectrum_widget->addGraph();
    ui->spectrum_widget->graph(0)->setPen(QPen(Qt::green, 1));
    ui->spectrum_widget->graph(0)->setBrush(QBrush(QColor(0, 255, 0, 50)));

    ui->spectrum_widget->replot();
}

void MainWindow::updateSpectrumPlot(const QVector<double> &data)
{
    if (data.size() != 1024) {
        qDebug() << "Некорректный размер данных для спектра:" << data.size();
        return;
    }

    double maxVal = *std::max_element(data.constBegin(), data.constEnd());
    if (maxVal < 1.0) maxVal = 1.0;

    double yMax = maxVal * 1.1;
    ui->spectrum_widget->yAxis->setRange(0, yMax);

    QVector<double> x(1024);
    for (int i = 0; i < 1024; ++i) {
        x[i] = i;
    }

    ui->spectrum_widget->graph(0)->setData(x, data);
    ui->spectrum_widget->replot();
}

void MainWindow::on_pushButton_clicked()
{
    QVector<double> data(1024);
    for (int i = 0; i < 1024; ++i) {
        double value = 3500 * sin(i * 0.05) + 500 * ((rand() % 100) / 100.0);
        data[i] = qBound(0.0, value, 65535.0);
    }
    updateSpectrumPlot(data);
    ui->textEdit->append("Тестовий спектр виведено на графік");
}

void MainWindow::on_pushButton_send_lidar_once_clicked()
{
    if (!lidar_port || !lidar_port->isOpen()) {
        ui->textEdit->append("LIDAR: порт не відкрито");
        return;
    }
    sendSingleLidarPacket();
    ui->textEdit->append("LIDAR: одноразовий пакет відправлено");
}

void MainWindow::onCheckBoxLidarContinuousToggled(bool checked)
{
    if (checked) {
        if (m_isReconRunning && lidar_port && lidar_port->isOpen()) {
            int freqIndex = ui->comboBox_freq_lidar->currentIndex();
            int intervalMs = (freqIndex == 0) ? 1000 : 10;
            lidar_timer->start(intervalMs);
            lidar_is_active = true;
            ui->textEdit->append("LIDAR: безперервну передачу включено");
        }
    } else {
        if (lidar_timer->isActive()) {
            lidar_timer->stop();
            lidar_is_active = false;
            ui->textEdit->append("LIDAR: безперервну передачу вимкнено");
        }
    }
}















void MainWindow::sendSingleLidarPacket()
{
    if (!lidar_port || !lidar_port->isOpen()) return;

    PointData currentPoint = m_surveyArea.getCurrentPoint();
    double heightMeters = currentPoint.getHeight();

    int distanceCm = static_cast<int>(heightMeters * 100.0);
    if (distanceCm > 0xFFFF) distanceCm = 0xFFFF;
    if (distanceCm < 0) distanceCm = 0;

    QString mode = ui->comboBox_imit_mode_lidar->currentText();
    quint16 distance = static_cast<quint16>(distanceCm);
    quint16 strength = 0x0FFF;

    if (mode == "Об'єкт не виявлено") {
        distance = 0;
        strength = 0x00D2;
    } else if (mode == "Слабке відбиття") {
        strength = 0x00D2;
    } else if (mode == "Сильне відбиття") {
        strength = 0xBFFF;
    } else if (mode == "Об'єкт дуже близько") {
        distance = 0x0010;
    } else if (mode == "Об'єкт занадто далеко") {
        distance = 0xFFFF;
    }

    QByteArray packet;
    packet.resize(9);
    packet[0] = 0x59;
    packet[1] = 0x59;
    packet[2] = static_cast<char>(distance & 0xFF);
    packet[3] = static_cast<char>((distance >> 8) & 0xFF);
    packet[4] = static_cast<char>(strength & 0xFF);
    packet[5] = static_cast<char>((strength >> 8) & 0xFF);
    packet[6] = 0x00;
    packet[7] = 0x00;

    quint8 checksum = 0;
    for (int i = 0; i < 8; ++i) {
        checksum += static_cast<quint8>(packet[i]);
    }
    checksum = checksum & 0xFF;

    if (ui->checkBox_uncorrect_lidar->isChecked()) {
        checksum = checksum ^ 0xFF;
    }

    packet[8] = static_cast<char>(checksum);

    m_lastLidarDistance = distance;
    m_lastLidarStrength = strength;

    QString hexPacket = packet.toHex().toUpper();
    ui->textEdit->append("LIDAR (одноразово) відправлено: " + hexPacket);
    lidar_port->write(packet);
}



















void MainWindow::onCheckBoxGpsContinuousToggled(bool checked)
{
    if (checked) {
        if (m_isReconRunning && gps_port && gps_port->isOpen()) {
            gps_timer->start(1000);
            gps_is_active = true;
            ui->textEdit->append("GPS: безперервну передачу включено");
        }
    } else {
        if (gps_timer->isActive()) {
            gps_timer->stop();
            gps_is_active = false;
            ui->textEdit->append("GPS: безперервну передачу вимкнено");
        }
    }
}

void MainWindow::on_pushButton_send_gps_once_clicked()
{
    if (!gps_port || !gps_port->isOpen()) {
        ui->textEdit->append("GPS: порт не відкрито");
        return;
    }
    sendSingleGpsPacket();
    ui->textEdit->append("GPS: одноразовий пакет відправлено");
}

















void MainWindow::sendSingleGpsPacket()
{
    if (!gps_port || !gps_port->isOpen()) return;

    PointData currentPoint = m_surveyArea.getCurrentPoint();
    double latitude = currentPoint.getLatitude();
    double longitude = currentPoint.getLongitude();
    double altitude = currentPoint.getHeight();

    if (ui->checkBox_antenna_fault_gps->isChecked()) {
        latitude = 0.0;
        longitude = 0.0;
        altitude = 0.0;
    }
    if (ui->checkBox_spoofing_gps->isChecked()) {
        QRandomGenerator *rand = QRandomGenerator::global();
        double latOffset = (rand->generateDouble() * 20.0) - 10.0;
        double lonOffset = (rand->generateDouble() * 20.0) - 10.0;
        latitude += latOffset;
        longitude += lonOffset;
        if (latitude > 90.0) latitude = 90.0;
        if (latitude < -90.0) latitude = -90.0;
        if (longitude > 180.0) longitude = 180.0;
        if (longitude < -180.0) longitude = -180.0;
    }

    QDateTime utcNow = QDateTime::currentDateTimeUtc();
    QTime utcTime = utcNow.time();
    if (ui->checkBox_spoofing_gps->isChecked()) {
        QRandomGenerator *rand = QRandomGenerator::global();
        int hourOffset = (rand->generate() % 3) + 1;
        if (rand->generate() % 2 == 0) hourOffset = -hourOffset;
        utcTime = utcTime.addSecs(hourOffset * 3600);
    }
    QString utcTimeStr = utcTime.toString("hhmmss.00");

    QString latNmea = formatLatLon(latitude, true);
    QString lonNmea = formatLatLon(longitude, false);
    char latDir = (latitude >= 0) ? 'N' : 'S';
    char lonDir = (longitude >= 0) ? 'E' : 'W';

    int satellites = ui->spinBox_satellites_gps->value();
    double hdop = ui->doubleSpinBox_hdop_gps->value();
    QString status = ui->checkBox_status_valid_gps->isChecked() ? "A" : "V";
    QString mode = ui->checkBox_mode_valid_gps->isChecked() ? "A" : "N";

    QString gga = QString("$GNGGA,%1,%2,%3,%4,%5,1,%6,%7,%8,M,46.9,M,,*")
                      .arg(utcTimeStr)
                      .arg(latNmea).arg(latDir)
                      .arg(lonNmea).arg(lonDir)
                      .arg(satellites, 2, 10, QChar('0'))
                      .arg(hdop, 0, 'f', 1)
                      .arg(altitude, 0, 'f', 1);
    gga.replace("*", "*" + calculateNmeaChecksum(gga));

    QString gll = QString("$GNGLL,%1,%2,%3,%4,%5,%6,%7,*")
                      .arg(latNmea).arg(latDir)
                      .arg(lonNmea).arg(lonDir)
                      .arg(utcTimeStr)
                      .arg(status)
                      .arg(mode);
    gll.replace("*", "*" + calculateNmeaChecksum(gll));

    QString dateStr = utcNow.toString("ddMMyy");
    QString rmc = QString("$GNRMC,%1,%2,%3,%4,%5,%6,0.0,0.0,%7,,,A*")
                      .arg(utcTimeStr)
                      .arg(status)
                      .arg(latNmea).arg(latDir)
                      .arg(lonNmea).arg(lonDir)
                      .arg(dateStr);
    rmc.replace("*", "*" + calculateNmeaChecksum(rmc));

    m_lastGpsLatitude = latitude;
    m_lastGpsLongitude = longitude;
    m_lastGpsUtcTime = utcTime;

    GpsRecord rec;
    rec.utcTime = utcTime;
    rec.lat = latitude;
    rec.lon = longitude;
    m_gpsJournal.append(rec);

    if (ui->checkBox_realistic_gps->isChecked()) {
        QRandomGenerator *rand = QRandomGenerator::global();
        QList<int> prnList;
        for (int i = 0; i < qMin(satellites, 12); ++i) {
            prnList.append(rand->bounded(1, 33));
        }
        while (prnList.size() < 12) prnList.append(0);
        QString prnStr;
        for (int prn : prnList) {
            if (prn == 0) prnStr += ",";
            else prnStr += QString::number(prn) + ",";
        }
        if (prnStr.endsWith(",")) prnStr.chop(1);

        double pdop = hdop * 1.1;
        double vdop = hdop * 0.7;
        QString gsa = QString("$GNGSA,A,1,%1,%2,%3,%4*")
                          .arg(prnStr)
                          .arg(hdop, 0, 'f', 2)
                          .arg(pdop, 0, 'f', 2)
                          .arg(vdop, 0, 'f', 2);
        gsa.replace("*", "*" + calculateNmeaChecksum(gsa));

        int gpsSatellites = qMin(satellites, 4);
        QString gsvGps = QString("$GPGSV,1,1,%1").arg(gpsSatellites);
        for (int i = 0; i < gpsSatellites; ++i) {
            int prn = rand->bounded(1, 33);
            int elevation = rand->bounded(10, 90);
            int azimuth = rand->bounded(0, 360);
            int snr = rand->bounded(35, 50);
            gsvGps += QString(",%1,%2,%3,%4").arg(prn, 2, 10, QChar('0'))
                          .arg(elevation)
                          .arg(azimuth)
                          .arg(snr);
        }
        gsvGps += "*";
        gsvGps.replace("*", "*" + calculateNmeaChecksum(gsvGps));

        int glonassSatellites = qMin(satellites - gpsSatellites, 2);
        if (glonassSatellites < 0) glonassSatellites = 0;
        QString gsvGlonass = QString("$GLGSV,1,1,%1").arg(glonassSatellites);
        for (int i = 0; i < glonassSatellites; ++i) {
            int prn = rand->bounded(65, 97);
            int elevation = rand->bounded(10, 90);
            int azimuth = rand->bounded(0, 360);
            int snr = rand->bounded(35, 50);
            gsvGlonass += QString(",%1,%2,%3,%4").arg(prn, 2, 10, QChar('0'))
                              .arg(elevation)
                              .arg(azimuth)
                              .arg(snr);
        }
        gsvGlonass += "*";
        gsvGlonass.replace("*", "*" + calculateNmeaChecksum(gsvGlonass));

        QString vtg = QString("$GNVTG,0.0,T,0.0,M,0.0,N,0.0,K,N*");
        vtg += calculateNmeaChecksum(vtg);

        QString txt;
        if (rand->generateDouble() < 0.3) {
            txt = QString("$GNTXT,01,01,01,More than 100 frame errors, UART RX was disabled*");
            txt += calculateNmeaChecksum(txt);
        }

        gps_port->write((gga + "\r\n").toUtf8());
        gps_port->write((gll + "\r\n").toUtf8());
        gps_port->write((rmc + "\r\n").toUtf8());
        gps_port->write((gsa + "\r\n").toUtf8());
        gps_port->write((gsvGps + "\r\n").toUtf8());
        if (glonassSatellites > 0) {
            gps_port->write((gsvGlonass + "\r\n").toUtf8());
        }
        gps_port->write((vtg + "\r\n").toUtf8());
        if (!txt.isEmpty()) {
            gps_port->write((txt + "\r\n").toUtf8());
        }

        ui->textEdit->append("GPS (одноразово, реалістичний): " + gga.trimmed());
        ui->textEdit->append("GPS (одноразово, реалістичний): " + gll.trimmed());
        ui->textEdit->append("GPS (одноразово, реалістичний): " + rmc.trimmed());
        ui->textEdit->append("GPS (одноразово, реалістичний): " + gsa.trimmed());
        ui->textEdit->append("GPS (одноразово, реалістичний): " + gsvGps.trimmed());
        if (glonassSatellites > 0) {
            ui->textEdit->append("GPS (одноразово, реалістичний): " + gsvGlonass.trimmed());
        }
        ui->textEdit->append("GPS (одноразово, реалістичний): " + vtg.trimmed());
        if (!txt.isEmpty()) {
            ui->textEdit->append("GPS (одноразово, реалістичний): " + txt.trimmed());
        }
    } else {
        gps_port->write((gga + "\r\n").toUtf8());
        gps_port->write((gll + "\r\n").toUtf8());
        gps_port->write((rmc + "\r\n").toUtf8());

        ui->textEdit->append("GPS (одноразово): " + gga.trimmed());
        ui->textEdit->append("GPS (одноразово): " + gll.trimmed());
        ui->textEdit->append("GPS (одноразово): " + rmc.trimmed());
    }
}











void MainWindow::loadEtalonSpectrum(const QString &fileName)
{
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        ui->textEdit->append(QString("Не удалось открыть файл эталонного спектра: %1").arg(fileName));
        m_etalonSpectrumReply.fill(0, 2076);
        return;
    }

    QTextStream in(&file);
    QStringList hexStrings;
    QByteArray rawData;

    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty()) continue;
        hexStrings.append(line);
    }
    file.close();

    for (const QString &hexStr : hexStrings) {
        bool ok;
        uint8_t byte = static_cast<uint8_t>(hexStr.toUInt(&ok, 16));
        if (ok) {
            rawData.append(static_cast<char>(byte));
        } else {
            qDebug() << "Ошибка парсинга HEX:" << hexStr;
        }
    }

    if (rawData.size() < 2076) {
        qDebug() << "Файл спектра меньше 2076 байт, используется" << rawData.size() << "байт";
        rawData.append(2076 - rawData.size(), 0x00);
    } else if (rawData.size() > 2076) {
        qDebug() << "Файл спектра больше 2076 байт, обрезаем до 2076";
        rawData.truncate(2076);
    }

    m_etalonSpectrumReply = rawData;

    ui->textEdit->append(QString("Эталонный ответ спектра загружен, размер: %1 байт")
                             .arg(m_etalonSpectrumReply.size()));
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MainWindow::on_pushButton_clear_radioline_clicked()
{
    ui->textEdit_radioline->clear();
}

