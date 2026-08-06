#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "mapcore.h"

#include <QSerialPortInfo>
#include <QRandomGenerator>
#include <QDateTime>
#include <QFile>
#include <QDir>
#include <QFileDialog>

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

    // Начальное состояние кнопок (Ручной режим по умолчанию)
    onMoveModeChanged(0);

    connect(ui->req_interval_box, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MainWindow::onIntervalChanged);


    connect(ui->start_height_box, QOverload<int>::of(&QSpinBox::valueChanged),
            ui->cloud__lower_level_box, &QSpinBox::setValue);



    // Заполнение комбобокса портами от COM1 до COM40 - БДБГ-верхний
    for (int i = 1; i <= 40; ++i)
        ui->comboBox_BDBG_1->addItem(QString("COM%1").arg(i));

    for (int i = 1; i <= 40; ++i)
        ui->comboBox_LIDAR->addItem(QString("COM%1").arg(i));

    for (int i = 1; i <= 40; ++i)
        ui->comboBox_GPS->addItem(QString("COM%1").arg(i));


    // Инициализация таймера мониторинга портов
    m_portMonitorTimer = new QTimer(this);
    connect(m_portMonitorTimer, &QTimer::timeout, this, &MainWindow::onPortMonitorTick);

    // Запоминаем все порты, которые уже есть при старте программы
    QList<QSerialPortInfo> initialPorts = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &port : initialPorts) {
        m_knownPorts.append(port.portName());
    }

    // Запускаем мониторинг с интервалом 1 секунда
    m_portMonitorTimer->start(500);


    // Инициализация портов БДБГ
    bdbg_port = new QSerialPort(this);


    // Подключение сигналов readyRead (каждый к своему обработчику)
    connect(bdbg_port, &QSerialPort::readyRead, this, &MainWindow::onBdbgDataReceived);


    // Подключение сигналов выбора порта из comboBox
    connect(ui->comboBox_BDBG_1, QOverload<int>::of(&QComboBox::activated),
            this, &MainWindow::onBdbgPortSelected);



    // ============================================================
    // ИНИЦИАЛИЗАЦИЯ LIDAR
    // ============================================================

    // Создаём объект порта
    lidar_port = new QSerialPort(this);

    // Создаём таймер LIDAR (пока не запущен)
    lidar_timer = new QTimer(this);
    lidar_timer->setSingleShot(false);  // Повторяющийся таймер
    connect(lidar_timer, &QTimer::timeout, this, &MainWindow::onLidarTimerTick);

    // Флаг активности LIDAR
    lidar_is_active = false;

    // --- Заполняем comboBox для LIDAR ---

    // 1. Частота (1 Гц и 100 Гц)
    ui->comboBox_freq_lidar->addItem("1 Гц");
    ui->comboBox_freq_lidar->addItem("100 Гц");
    ui->comboBox_freq_lidar->setCurrentIndex(0);  // По умолчанию 1 Гц

    // 2. Режимы имитации
    ui->comboBox_imit_mode_lidar->addItem("Нормальне вимірювання");
    ui->comboBox_imit_mode_lidar->addItem("Об'єкт не виявлено");
    ui->comboBox_imit_mode_lidar->addItem("Слабке відбиття");
    ui->comboBox_imit_mode_lidar->addItem("Сильне відбиття");
    ui->comboBox_imit_mode_lidar->addItem("Об'єкт дуже близько");
    ui->comboBox_imit_mode_lidar->addItem("Об'єкт занадто далеко");
    ui->comboBox_imit_mode_lidar->setCurrentIndex(0);  // По умолчанию нормальное измерение

    // --- Подключаем сигналы ---

    // Выбор порта LIDAR
    connect(ui->comboBox_LIDAR, QOverload<int>::of(&QComboBox::activated), this, &MainWindow::onLidarPortSelected);

    // Смена частоты LIDAR
    connect(ui->comboBox_freq_lidar, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onLidarFreqChanged);


    // ============================================================
    // ИНИЦИАЛИЗАЦИЯ GPS
    // ============================================================

    // Создаём объект порта
    gps_port = new QSerialPort(this);

    // Создаём таймер GPS (1 Гц)
    gps_timer = new QTimer(this);
    gps_timer->setSingleShot(false);
    connect(gps_timer, &QTimer::timeout, this, &MainWindow::onGpsTimerTick);

    // Флаг активности GPS
    gps_is_active = false;

    // --- Заполняем comboBox для GPS (частота фиксированная 1 Гц) ---
    ui->comboBox_freq_gps->addItem("1 Гц");
    ui->comboBox_freq_gps->setCurrentIndex(0);

    // --- Подключаем сигналы ---
    connect(ui->comboBox_GPS, QOverload<int>::of(&QComboBox::activated),
            this, &MainWindow::onGpsPortSelected);


    // --- Заполнение comboBox для радиолинии ---
    for (int i = 1; i <= 40; ++i) {
        ui->comboBox_radioline->addItem(QString("COM%1").arg(i));
    }

    // --- Инициализация порта радиолинии ---
    logger_rx_port = new QSerialPort(this);
    connect(logger_rx_port, &QSerialPort::readyRead, this, &MainWindow::onRadiolineDataReceived);

    // Подключение выбора порта
    connect(ui->comboBox_radioline, QOverload<int>::of(&QComboBox::activated),
            this, &MainWindow::onRadiolinePortSelected);

    // Подключение кнопок загрузки SD и сравнения
    connect(ui->pushButton_load_SD_data, &QPushButton::clicked,
            this, &MainWindow::onLoadSdCardClicked);
    connect(ui->pushButton_compare_data, &QPushButton::clicked,
            this, &MainWindow::onCompareDataClicked);


}

MainWindow::~MainWindow()
{
    if (m_autoMoveTimer) {
        m_autoMoveTimer->stop();
        m_autoMoveTimer->deleteLater();
    }

    if (bdbg_port->isOpen()) {
        bdbg_port->close();       
    }
    bdbg_port->deleteLater();


    if (lidar_timer && lidar_timer->isActive()) {
        lidar_timer->stop();
    }
    if (lidar_port && lidar_port->isOpen()) {
        lidar_port->close();
    }
    lidar_timer->deleteLater();
    lidar_port->deleteLater();

    if (gps_timer && gps_timer->isActive()) {
        gps_timer->stop();
    }
    if (gps_port && gps_port->isOpen()) {
        gps_port->close();
    }
    gps_timer->deleteLater();
    gps_port->deleteLater();

    if (logger_rx_port && logger_rx_port->isOpen()) {
        logger_rx_port->close();
    }
    logger_rx_port->deleteLater();

    delete ui;
}

// void MainWindow::displayCurrentPoint()
// {
//     PointData currentPoint = m_surveyArea.getCurrentPoint();

//     // Обновляем поля текущих координат и высоты
//     ui->curr_lat_box->setValue(currentPoint.getLatitude());
//     ui->curr_lon_box->setValue(currentPoint.getLongitude());
//     ui->curr_height_box->setValue(currentPoint.getHeight());

//     // ========== ДОБАВЛЯЕМАЯ ЧАСТЬ ==========
//     // Определяем тип угрозы для отображения на карте
//     int dangerType = 0;
//     double bgPed = ui->paed_phone_box->value();  // фоновое значение ПЕД

//     // Если ПЕД нижнего или верхнего детектора превышает фон - радиационная угроза
//     if (currentPoint.getPedLower() > bgPed + 0.0001 ||
//         currentPoint.getPedUpper() > bgPed + 0.0001) {
//         dangerType = 1;  // 0x1 - радиация
//     }

//     // Передаём координаты и тип угрозы на карту
//     // Координаты: долгота (longitude), широта (latitude)
//     ui->p_map->setdronepos({currentPoint.getLongitude(), currentPoint.getLatitude()}, dangerType);
//     // ======================================

//     // Форматируем строку для вывода в textEdit (оставляем без изменений)
//     QString info = QString("Поточна точка:\n"
//                            "  Широта: %1\n"
//                            "  Довгота: %2\n"
//                            "  Висота: %3 м\n"
//                            "  ПЕД нижній (вниз): %4 мкЗв/ч\n"
//                            "  ПЕД верхній (вгору): %5 мкЗв/ч\n"
//                            "  Індекс вузла: %6, Індекс рівня: %7\n"
//                            "----------------------------------------")
//                        .arg(currentPoint.getLatitude(), 0, 'f', 6)
//                        .arg(currentPoint.getLongitude(), 0, 'f', 6)
//                        .arg(currentPoint.getHeight())
//                        .arg(currentPoint.getPedLower(), 0, 'f', 3)
//                        .arg(currentPoint.getPedUpper(), 0, 'f', 3)
//                        .arg(m_surveyArea.getCurrentGridIndex())
//                        .arg(m_surveyArea.getCurrentLevelIndex());

//     ui->textEdit->append(info);

//     /*
//         ТОЧКА ВХОДА ПРИСВОЕНИЯ ДАННЫХ ПРИБОРАМ
//         m_gps.setCoordinates(currentPoint.getLatitude(), currentPoint.getLongitude())
//         m_lidar.setHeight(currentPoint.getHeight())
//         m_dosimeterDown.setPedValue(currentPoint.getPedLower())
//         m_dosimeterUp.setPedValue(currentPoint.getPedUpper())
//     */

//     // ============================================================
//     // ДОБАВЛЕНИЕ ЭТАЛОННОЙ ЗАПИСИ
//     // ============================================================
//     // Каждый раз, когда отображается новая точка (при переходе,
//     // подъёме, спуске, авторежиме), добавляем её в эталонный список.
//     // Это позволяет сохранить все данные, которые были "измерены"
//     // в процессе разведки.
//     // ============================================================
//     addEtalonRecord();

// }


void MainWindow::displayCurrentPoint()
{
    PointData currentPoint = m_surveyArea.getCurrentPoint();

    // Обновляем поля текущих координат и высоты
    ui->curr_lat_box->setValue(currentPoint.getLatitude());
    ui->curr_lon_box->setValue(currentPoint.getLongitude());
    ui->curr_height_box->setValue(currentPoint.getHeight());

    // ========== ДОБАВЛЯЕМАЯ ЧАСТЬ ==========
    // Определяем тип угрозы для отображения на карте
    int dangerType = 0;
    double bgPed = ui->paed_phone_box->value();  // фоновое значение ПЕД

    // Если ПЕД нижнего или верхнего детектора превышает фон - радиационная угроза
    if (currentPoint.getPedLower() > bgPed + 0.0001 ||
        currentPoint.getPedUpper() > bgPed + 0.0001) {
        dangerType = 1;  // 0x1 - радиация
    }

    // Передаём координаты и тип угрозы на карту
    ui->p_map->setdronepos({currentPoint.getLongitude(), currentPoint.getLatitude()}, dangerType);
    // ======================================

    // ============================================================
    // ДОБАВЛЕНИЕ ЭТАЛОННОЙ ЗАПИСИ
    // ============================================================
    // Каждый раз, когда отображается новая точка (при переходе,
    // подъёме, спуске, авторежиме), добавляем её в эталонный список.
    // Это позволяет сохранить все данные, которые были "измерены"
    // в процессе разведки.
    // ============================================================
    addEtalonRecord();

    // ============================================================
    // ВЫВОД ЭТАЛОННОЙ ЗАПИСИ В textEdit
    // ============================================================
    // После добавления эталонной записи выводим её в textEdit
    // в том же формате, что и на SD-карте/радиолинии.
    // Это позволяет видеть, какие данные записываются в эталон,
    // и сравнивать их с тем, что приходит от логгера.
    // ============================================================
    if (!m_etalonRecords.isEmpty()) {
        const SdRecord &record = m_etalonRecords.last();  // берём последнюю добавленную запись

        // Формируем строку в формате SD-карты:
        // номер_строки,temp0,temp1,ped0,ped1,longitude,latitude,lidar_distance,lidar_strength
        QString etalonStr = QString("Еталон: %1,%2,%3,%4,%5,%6,%7,%8,%9")
                                .arg(record.lineNumber)                           // номер строки
                                .arg(record.temp0, 0, 'f', 2)                    // temp0
                                .arg(record.temp1, 0, 'f', 2)                    // temp1
                                .arg(record.ped0, 0, 'f', 3)                     // ped0
                                .arg(record.ped1, 0, 'f', 3)                     // ped1
                                .arg(record.longitude, 0, 'f', 6)                // longitude
                                .arg(record.latitude, 0, 'f', 6)                 // latitude
                                .arg(record.lidarDistance)                       // lidar_distance
                                .arg(record.lidarStrength);                      // lidar_strength

        ui->textEdit->append(etalonStr);
    }

    // ============================================================
    // ВЫВОД ИНФОРМАЦИИ О ТЕКУЩЕЙ ТОЧКЕ В textEdit
    // ============================================================
    // Форматируем строку для вывода в textEdit
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

    // Проверка вхождения в полигон
    if (isPointInPolygon(currentPoint, m_polygonPoints)) {
        info += "\n  Статус: В зоні";
    } else {
        info += "\n  Статус: Поза зоною";
    }

    info += "\n----------------------------------------";

    ui->textEdit->append(info);

    /*
        ТОЧКА ВХОДА ПРИСВОЕНИЯ ДАННЫХ ПРИБОРАМ
        m_gps.setCoordinates(currentPoint.getLatitude(), currentPoint.getLongitude())
        m_lidar.setHeight(currentPoint.getHeight())
        m_dosimeterDown.setPedValue(currentPoint.getPedLower())
        m_dosimeterUp.setPedValue(currentPoint.getPedUpper())
    */
}


void MainWindow::updateButtonsState()
{
    // Обновляем состояние кнопок на основе возможностей SurveyArea
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

    // Кнопка "Зробити крок" активна в ручном режиме, если есть точки
    ui->take_step_but->setEnabled(isManual && m_surveyArea.canMoveForward());

    // Кнопки вертикального перемещения активны только в ручном режиме И в режиме "Хмара"
    ui->up_1_level_but->setEnabled(isManual && isCloud && m_surveyArea.canMoveUp());
    ui->put_1_level_bu->setEnabled(isManual && isCloud && m_surveyArea.canMoveDown());

    // Кнопка "Спуститись на початкову висоту" активна только в ручном режиме и в режиме "Хмара"
    ui->descend_start_height_but->setEnabled(isManual && isCloud);

    // Если переключились на ручной режим во время авто-движения - останавливаем авторежим
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
        m_autoMoveTimer->start(newValue * 1000);  // перезапуск с новым интервалом
        ui->textEdit->append(QString("Інтервал опиту змінено на %1 сек").arg(newValue));
    }
}

void MainWindow::onPortMonitorTick()
{
    // ======================================================================
    // ОБРАБОТЧИК ТАЙМЕРА МОНИТОРИНГА COM-ПОРТОВ
    // ======================================================================
    // Задача: отслеживать появление новых COM-портов и выводить их в lineEdit_COM
    // При подключении нового преобразователя RS485-USB ОС создаёт новый порт,
    // этот обработчик его обнаруживает и показывает пользователю.
    // При отключении порт удаляется из списка известных.
    // При повторном подключении порт снова будет обнаружен как новый.
    // ======================================================================

    // Получаем текущий список доступных портов
    QList<QSerialPortInfo> currentPorts = QSerialPortInfo::availablePorts();
    QStringList currentPortNames;

    // Сохраняем имена текущих портов в список
    for (const QSerialPortInfo &port : currentPorts) {
        currentPortNames.append(port.portName());
    }

    // ---- 1. Проверяем, появились ли новые порты ----
    for (const QString &portName : currentPortNames) {
        if (!m_knownPorts.contains(portName)) {
            // Новый порт обнаружен!
            // Выводим его имя в lineEdit_COM (только последний подключенный)
            ui->lineEdit_COM->setText(portName);

            // Добавляем новый порт в список известных
            m_knownPorts.append(portName);
        }
    }

    // ---- 2. Проверяем, не отключились ли какие-то порты ----
    for (int i = m_knownPorts.size() - 1; i >= 0; --i) {
        if (!currentPortNames.contains(m_knownPorts[i])) {
            // Порт был отключен - удаляем его из списка известных
            m_knownPorts.removeAt(i);
        }
    }
}

void MainWindow::onAutoMoveTick()
{
    QString mode = ui->recogn_mode_combo->currentText();

    if (mode == "Поверхня") {
        // Для поверхности: просто переход к следующему узлу
        if (m_surveyArea.moveForward()) {
            displayCurrentPoint();
            updateButtonsState();
        } else {
            // Достигли последней точки
            m_autoMoveTimer->stop();
            m_isAutoMoving = false;
            ui->start_reconn_but->setText("Почати розвідку");
            ui->move_mode_combo->setCurrentIndex(0); // Переключаем на ручной режим
            ui->textEdit->append("Авторежим завершено! Пройдено всі точки.");
        }
    }
    else if (mode == "Хмара") {
        // Для облака: полный обход всех уровней в текущем узле, затем переход к следующему узлу
        int currentLevel = m_surveyArea.getCurrentLevelIndex();
        int numLevels = m_surveyArea.getNumLevels();

        if (currentLevel + 1 < numLevels) {
            // Есть следующий уровень - поднимаемся
            m_surveyArea.moveUp();
            displayCurrentPoint();
            updateButtonsState();
        } else {
            // Достигли верхнего уровня - переходим к следующему узлу (сбрасываем высоту)
            if (m_surveyArea.moveForward()) {
                // Сбрасываем высоту на начальную в новом узле
                m_surveyArea.resetToStartHeight();
                displayCurrentPoint();
                updateButtonsState();
            } else {
                // Достигли последнего узла и последнего уровня
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
    // Полигон строится по крайним точкам сетки с увеличением на 0.005 градуса
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

    // --- 4. Создаём внешнюю стартовую точку (гарантированно вне полигона) ---
    double offsetLat = 0.01;   // увеличенное смещение для надёжности
    double offsetLon = 0.01;
    double startLatOutside = startLat + offsetLat;
    double startLonOutside = startLon - offsetLon;

    // --- 5. Читаем количество точек пути из spinBox ---
    int pathCount = ui->spinBox_path_to_zone->value();

    // --- 6. Генерируем путь к зоне ---
    QList<PointData> pathToZone;
    PointData firstMeasPoint = measurementPoints.first();

    generatePathPoints(startLatOutside, startLonOutside,
                       firstMeasPoint.getLatitude(), firstMeasPoint.getLongitude(),
                       pathCount, startHeight, bgPed, pathToZone);


    // // --- 7. Генерируем путь обратно ---
    // QList<PointData> pathFromZone;
    // PointData lastMeasPoint = measurementPoints.last();

    // generatePathPoints(lastMeasPoint.getLatitude(), lastMeasPoint.getLongitude(),
    //                    startLatOutside, startLonOutside,
    //                    pathCount, startHeight, bgPed, pathFromZone);

    // --- Генерируем путь обратно с удвоенным количеством точек ---
    int pathBackCount = pathCount * 2;  // в 2 раза больше

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

    displayCurrentPoint();
    updateButtonsState();

    m_etalonRecords.clear();
    m_sdRecords.clear();
    m_radiolineRecords.clear();
    ui->textEdit_radioline->clear();
}











// ============================================================
// ОБРАБОТЧИК КНОПКИ "ПОЧАТИ РОЗВІДКУ" / "ЗУПИНИТИ РОЗВІДКУ"
// ============================================================
// Запускает/останавливает:
//   1. Автоматическое перемещение дрона (если выбран режим "Авто")
//   2. Непрерывную отправку данных LIDAR
//   3. Непрерывную отправку данных GPS
// ============================================================
void MainWindow::onStartReconClicked()
{
    if (!m_isReconRunning) {
        // ===== ЗАПУСК РОЗВІДКИ =====

        if (m_surveyArea.getTotalPointsCount() == 0) {
            ui->textEdit->append("Помилка: спочатку створіть область розвідки");
            return;
        }

        // --- Запуск LIDAR ---
        if (lidar_port && lidar_port->isOpen()) {
            int freqIndex = ui->comboBox_freq_lidar->currentIndex();
            int intervalMs = (freqIndex == 0) ? 1000 : 10;
            lidar_timer->start(intervalMs);
            lidar_is_active = true;
            ui->textEdit->append(QString("LIDAR: запущено (частота %1)")
                                     .arg(ui->comboBox_freq_lidar->currentText()));
        } else {
            ui->textEdit->append("LIDAR: порт не відкрито, запуск неможливий");
        }

        // --- Запуск GPS ---
        if (gps_port && gps_port->isOpen()) {
            gps_timer->start(1000);
            gps_is_active = true;
            ui->textEdit->append("GPS: запущено (частота 1 Гц)");
        } else {
            ui->textEdit->append("GPS: порт не відкрито, запуск неможливий");
        }

        // --- Авторежим (если выбран) ---
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
        // ===== ЗУПИНКА РОЗВІДКИ =====

        // --- Остановка LIDAR ---
        if (lidar_timer->isActive()) {
            lidar_timer->stop();
        }
        lidar_is_active = false;
        ui->textEdit->append("LIDAR: зупинено");

        // --- Остановка GPS ---
        if (gps_timer->isActive()) {
            gps_timer->stop();
        }
        gps_is_active = false;
        ui->textEdit->append("GPS: зупинено");

        // --- Остановка авторежима ---
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
    // Если авторежим остановлен и мониторинг портов не активен - перезапускаем его
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
// Вызывается, когда в порту появляются данные.
// 1. Читает запрос
// 2. Проверяет CRC
// 3. Извлекает адрес (байт с индексом 3)
// 4. Для адреса 0x01 берёт pedLower, для 0x02 — pedUpper
// 5. Использует соответствующие чекбоксы неисправностей (_1 или _2)
// 6. Формирует и отправляет ответ
// ============================================================
void MainWindow::onBdbgDataReceived()
{
    // --- 1. ЧИТАЕМ ДАННЫЕ ИЗ ПОРТА ---

    QByteArray receiveBuff;

    // Даём время накопить данные
    QEventLoop loop;
    QTimer::singleShot(100, &loop, [&loop]() {
        loop.quit();
    });
    loop.exec();

    // Читаем всё, что есть в буфере
    receiveBuff.append(bdbg_port->readAll());

    // Если после первого чтения ещё что-то пришло — добираем
    while (bdbg_port->waitForReadyRead(50)) {
        receiveBuff.append(bdbg_port->readAll());
    }

    // Очищаем буфер порта
    bdbg_port->clear();

    // Если ничего не пришло — выходим
    if (receiveBuff.isEmpty()) {
        return;
    }

    // Вывод принятого массива в HEX
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

    // Структура пакета: 0x55 0xAA 0x70 0xFF [Adrs] [команда] [данные] [CRC]
    // Адрес находится в байте с индексом 3 (четвёртый байт)
    if (receiveBuff.size() <= 3) {
        ui->textEdit->append("БДБГ: пакет занадто короткий для визначення адреси");
        return;
    }

    quint8 address = static_cast<quint8>(receiveBuff[3]);

    // --- 4. ПРОВЕРКА ЧЕКБОКСА "ВІДСУТНІСТЬ ЗВ'ЯЗКУ" (в зависимости от адреса) ---

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

    // Выбираем ПЕД в зависимости от адреса
    double pedValue = 0.0;
    if (address == 0x01) {
        pedValue = currentPoint.getPedLower();   // нижний дозиметр
    } else if (address == 0x02) {
        pedValue = currentPoint.getPedUpper();   // верхний дозиметр
    }

    // --- 6. ОПРЕДЕЛЯЕМ КОМАНДУ И ФОРМИРУЕМ ОТВЕТ ---

    // Команда находится в байте с индексом 4 (пятый байт)
    if (receiveBuff.size() <= 4) {
        ui->textEdit->append("БДБГ: пакет занадто короткий для визначення команди");
        return;
    }

    quint8 command = static_cast<quint8>(receiveBuff[4]);

    QByteArray response;

    if (command == 0x08) {
        // ===== ЗАПИТ ТЕМПЕРАТУРЫ =====
        // Ответ: 55 AA 70 FF [Adrs] 08 5A 01 [CRC]
        response.append(QByteArray::fromHex("55AA70FF"));
        response.append(static_cast<char>(address));
        response.append(QByteArray::fromHex("085A01"));

        // Вычисляем CRC
        int crc = calcCrc(response);

        // Проверяем чекбокс "Некоректна відповідь" в зависимости от адреса
        if (address == 0x01 && ui->checkBox_uncorrect_1->isChecked()) {
            crc += 0x02;
        } else if (address == 0x02 && ui->checkBox_uncorrect_2->isChecked()) {
            crc += 0x02;
        }

        response.append(static_cast<char>(crc & 0xFF));

        // Вывод ответа в HEX
        QString hexResponse = response.toHex().toUpper();
        ui->textEdit->append("БДБГ відправлено: " + hexResponse);

        // Отправляем ответ
        writeBdbgData(bdbg_port, response);

        ui->textEdit->append("БДБГ: відповідь на запит температури відправлено (адр. " +
                             QString::number(address) + ")");
    }

    else if (command == 0x00) {
        // ===== ЗАПИТ ПЕД =====

        // Ответ: 55 AA 70 FF [Adrs] 01 [4 байта float ПЕД] [статусный байт] 00 [CRC]
        response.append(QByteArray::fromHex("55AA70FF"));
        response.append(static_cast<char>(address));
        response.append(QByteArray::fromHex("01"));

        // ---- Вставляем значение ПЕД (4 байта float, little-endian) ----
        float pedFloat = static_cast<float>(pedValue);
        const char* pedBytes = reinterpret_cast<const char*>(&pedFloat);
        response.append(pedBytes, sizeof(float));

        // ---- Статусный байт (битовые флаги) ----
        quint8 statusByte = 0x11;  // базовое значение

        // Устанавливаем флаги в зависимости от адреса
        if (address == 0x01) {
            if (ui->checkBox_high_sens_1->isChecked()) {
                statusByte |= 0x01;
            }
            if (ui->checkBox_low_sens_1->isChecked()) {
                statusByte |= 0x02;
            }
        } else if (address == 0x02) {
            if (ui->checkBox_high_sens_2->isChecked()) {
                statusByte |= 0x01;
            }
            if (ui->checkBox_low_sens_2->isChecked()) {
                statusByte |= 0x02;
            }
        }

        response.append(static_cast<char>(statusByte));

        // ---- Добавляем завершающий байт ----
        response.append(static_cast<char>(0x00));

        // ---- Вычисляем CRC ----
        int crc = calcCrc(response);

        // Проверяем чекбокс "Некоректна відповідь"
        if (address == 0x01 && ui->checkBox_uncorrect_1->isChecked()) {
            crc += 0x02;
        } else if (address == 0x02 && ui->checkBox_uncorrect_2->isChecked()) {
            crc += 0x02;
        }

        response.append(static_cast<char>(crc & 0xFF));

        // Отправляем ответ
        writeBdbgData(bdbg_port, response);

        ui->textEdit->append(QString("БДБГ: відповідь на запит ПЕД відправлено (адр. %1, ПЕД = %2)")
                                 .arg(address).arg(pedValue, 0, 'f', 3));
    }

    else {
        ui->textEdit->append(QString("БДБГ: невідома команда 0x%1")
                                 .arg(command, 2, 16, QChar('0')));
    }
}


// ============================================================
// ОТПРАВКА ДАННЫХ В ПОРТ БДБГ
// ============================================================
// Записывает данные в указанный порт и ждёт завершения отправки.
// Возвращает количество отправленных байт.
// ============================================================
int MainWindow::writeBdbgData(QSerialPort *port, const QByteArray &data)
{
    if (!port || !port->isOpen()) {
        return -1;  // порт не открыт
    }


    // 2. Вывод массива байт для передачи в UPPER HEX
    QString hexResponse = data.toHex().toUpper();
    ui->textEdit->append("БДБГ-1 відправлено: " + hexResponse + '\n');

    int bytesWritten = port->write(data);
    if (bytesWritten > 0) {
        port->waitForBytesWritten(100);  // ждём завершения отправки (до 100 мс)
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
// ВЫБОР ПОРТА ДЛЯ LIDAR
// ============================================================
// Вызывается при выборе порта в comboBox_LIDAR.
// Открывает порт с настройками: 115200, 8N1 (типично для LIDAR).
// ============================================================
void MainWindow::onLidarPortSelected(int index)
{
    Q_UNUSED(index)

    // Получаем имя порта из comboBox
    QString portName = ui->comboBox_LIDAR->currentText();

    // Если выбран "NOT SELECT" или пустая строка — ничего не делаем
    if (portName == "NOT SELECT" || portName.isEmpty()) {
        return;
    }

    // Если порт уже открыт — закрываем его (для переподключения)
    if (lidar_port->isOpen()) {
        lidar_port->close();
        // Останавливаем таймер, если он был запущен
        if (lidar_timer->isActive()) {
            lidar_timer->stop();
            lidar_is_active = false;
        }
    }

    // Настраиваем порт (скорость для LIDAR обычно 115200)
    lidar_port->setPortName(portName);
    lidar_port->setBaudRate(QSerialPort::Baud115200);
    lidar_port->setDataBits(QSerialPort::Data8);
    lidar_port->setParity(QSerialPort::NoParity);
    lidar_port->setStopBits(QSerialPort::OneStop);
    lidar_port->setFlowControl(QSerialPort::NoFlowControl);

    // Пытаемся открыть порт в режиме чтения/записи
    if (lidar_port->open(QSerialPort::ReadWrite)) {
        ui->textEdit->append(QString("LIDAR: порт %1 відкрито").arg(portName));
    } else {
        ui->textEdit->append(QString("LIDAR: помилка відкриття порту %1").arg(portName));
    }
}


// ============================================================
// СМЕНА ЧАСТОТЫ LIDAR
// ============================================================
// Вызывается при изменении выбранной частоты в comboBox_freq_lidar.
// Если LIDAR активен — перезапускает таймер с новым интервалом.
// ============================================================
void MainWindow::onLidarFreqChanged(int index)
{
    // Если LIDAR не активен — ничего не делаем
    if (!lidar_is_active) {
        return;
    }

    // Останавливаем текущий таймер
    if (lidar_timer->isActive()) {
        lidar_timer->stop();
    }

    // Определяем интервал в миллисекундах
    int intervalMs = 0;
    if (index == 0) {
        // 1 Гц = 1000 мс
        intervalMs = 1000;
    } else if (index == 1) {
        // 100 Гц = 10 мс
        intervalMs = 10;
    } else {
        return;
    }

    // Запускаем таймер с новым интервалом
    lidar_timer->start(intervalMs);
    ui->textEdit->append(QString("LIDAR: частоту змінено на %1").arg(ui->comboBox_freq_lidar->currentText()));
}


// ============================================================
// ОБРАБОТЧИК ТАЙМЕРА LIDAR (ОТПРАВКА ДАННЫХ)
// ============================================================
// Вызывается периодически с частотой 1 Гц или 100 Гц.
// Формирует пакет LIDAR и отправляет его в порт.
// ============================================================
// void MainWindow::onLidarTimerTick()
// {
//     qDebug() << "LIDAR tick, lidar_is_active =" << lidar_is_active;

//     // --- 0. ПРОВЕРКА: активен ли LIDAR ---
//     if (!lidar_is_active) {
//         return;  // Если LIDAR остановлен — ничего не отправляем
//     }

//     // --- 1. ПРОВЕРКА: порт открыт? ---
//     if (!lidar_port || !lidar_port->isOpen()) {
//         return;
//     }

//     // --- 2. ПРОВЕРКА: чекбокс "Відсутність зв'язку" ---
//     if (ui->checkBox_not_ans_lidar->isChecked()) {
//         // Имитация обрыва связи — просто молчим, ничего не отправляем
//         return;
//     }

//     // --- 3. ПОЛУЧАЕМ ТЕКУЩУЮ ВЫСОТУ ---
//     PointData currentPoint = m_surveyArea.getCurrentPoint();
//     double heightMeters = currentPoint.getHeight();  // Высота в метрах

//     // Переводим высоту из метров в сантиметры (целое число)
//     int distanceCm = static_cast<int>(heightMeters * 100.0);

//     // Ограничиваем максимальное значение 0xFFFF (65535 см = 655.35 м)
//     if (distanceCm > 0xFFFF) {
//         distanceCm = 0xFFFF;
//     }
//     if (distanceCm < 0) {
//         distanceCm = 0;
//     }

//     // --- 4. ОПРЕДЕЛЯЕМ РЕЖИМ ИМИАЦИИ ---
//     QString mode = ui->comboBox_imit_mode_lidar->currentText();

//     // Значения по умолчанию
//     quint16 distance = static_cast<quint16>(distanceCm);
//     quint16 strength = 0x0FFF;  // Нормальный Strength (середина диапазона 255 < x < 0xAFFF)

//     // --- 5. ПРИМЕНЯЕМ РЕЖИМ ИМИАЦИИ ---
//     if (mode == "Нормальне вимірювання") {
//         // Всё оставляем как есть (реальная высота, нормальный Strength)
//         // distance уже установлен, strength = 0x0FFF
//     }
//     else if (mode == "Об'єкт не виявлено") {
//         distance = 0;           // Объект не обнаружен
//         strength = 0x00D2;      // Слабое отражение (< 255)
//     }
//     else if (mode == "Слабке відбиття") {
//         // distance — реальная высота
//         strength = 0x00D2;      // Слабое отражение (0xD2 = 210)
//     }
//     else if (mode == "Сильне відбиття") {
//         // distance — реальная высота
//         strength = 0xBFFF;      // Сильное отражение (> 0xAFFF)
//     }
//     else if (mode == "Об'єкт дуже близько") {
//         distance = 0x0010;      // Очень близко (< 0x20)
//         // strength — нормальный
//     }
//     else if (mode == "Об'єкт занадто далеко") {
//         distance = 0xFFFF;      // Максимальное значение (слишком далеко)
//         // strength — нормальный
//     }
//     else {
//         // Неизвестный режим — используем нормальный
//     }

//     // --- 6. ФОРМИРУЕМ ПАКЕТ LIDAR ---

//     QByteArray packet;
//     packet.resize(9);  // 8 байт данных + 1 байт Checksum

//     // Заголовок: 0x59 0x59
//     packet[0] = 0x59;
//     packet[1] = 0x59;

//     // Distance (2 байта, Little-Endian: младший байт сначала)
//     packet[2] = static_cast<char>(distance & 0xFF);         // Low byte
//     packet[3] = static_cast<char>((distance >> 8) & 0xFF);  // High byte

//     // Strength (2 байта, Little-Endian)
//     packet[4] = static_cast<char>(strength & 0xFF);         // Low byte
//     packet[5] = static_cast<char>((strength >> 8) & 0xFF);  // High byte

//     // Reserved (2 байта, всегда 0x00)
//     packet[6] = 0x00;
//     packet[7] = 0x00;

//     // --- 7. ВЫЧИСЛЯЕМ CHECKSUM ---

//     // Checksum = сумма всех байтов с 0 по 7, младший байт (0xFF)
//     quint8 checksum = 0;
//     for (int i = 0; i < 8; ++i) {
//         checksum += static_cast<quint8>(packet[i]);
//     }
//     // Берём только младший байт
//     checksum = checksum & 0xFF;

//     // --- 8. ПРОВЕРКА ЧЕКБОКСА "НЕКОРЕКТНА ВІДПОВІДЬ" ---
//     if (ui->checkBox_uncorrect_lidar->isChecked()) {
//         // Имитация ошибки: портим Checksum (инвертируем или добавляем 1)
//         checksum = checksum ^ 0xFF;  // Инверсия байта
//         // или можно: checksum = checksum + 1;
//     }

//     // Записываем Checksum в пакет (байт 8)
//     packet[8] = static_cast<char>(checksum);

//     // --- 9. ВЫВОД HEX-СТРОКИ В textEdit (опционально, для отладки) ---
//     QString hexPacket = packet.toHex().toUpper();
//     ui->textEdit->append("LIDAR відправлено: " + hexPacket);

//     // --- 10. ОТПРАВЛЯЕМ ПАКЕТ В ПОРТ ---
//     qint64 bytesWritten = lidar_port->write(packet);
//     if (bytesWritten == 9) {
//         // Успешно отправлено
//     } else {
//         ui->textEdit->append("LIDAR: помилка відправки пакета");
//     }
// }


void MainWindow::onLidarTimerTick()
{
    // --- 0. ПРОВЕРКА: активен ли LIDAR ---
    if (!lidar_is_active) {
        return;  // Если LIDAR остановлен — ничего не отправляем
    }

    // --- 1. ПРОВЕРКА: порт открыт? ---
    if (!lidar_port || !lidar_port->isOpen()) {
        return;
    }

    // --- 2. ПРОВЕРКА: чекбокс "Відсутність зв'язку" ---
    if (ui->checkBox_not_ans_lidar->isChecked()) {
        return;
    }

    // --- 3. ПОЛУЧАЕМ ТЕКУЩУЮ ВЫСОТУ ---
    PointData currentPoint = m_surveyArea.getCurrentPoint();
    double heightMeters = currentPoint.getHeight();

    int distanceCm = static_cast<int>(heightMeters * 100.0);
    if (distanceCm > 0xFFFF) distanceCm = 0xFFFF;
    if (distanceCm < 0) distanceCm = 0;

    // --- 4. ОПРЕДЕЛЯЕМ РЕЖИМ ИМИАЦИИ ---
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

    // --- 5. ФОРМИРУЕМ ПАКЕТ LIDAR ---
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

    // --- 6. ОТПРАВКА ---
    QString hexPacket = packet.toHex().toUpper();
    ui->textEdit->append("LIDAR відправлено: " + hexPacket);
    lidar_port->write(packet);
}













// ============================================================
// ВЫБОР ПОРТА ДЛЯ GPS
// ============================================================
// Открывает порт с настройками: 4800, 8N1 (стандарт NMEA)
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
    gps_port->setBaudRate(QSerialPort::Baud115200);   // NMEA стандарт — 4800
    gps_port->setDataBits(QSerialPort::Data8);
    gps_port->setParity(QSerialPort::NoParity);
    gps_port->setStopBits(QSerialPort::OneStop);
    gps_port->setFlowControl(QSerialPort::NoFlowControl);

    if (gps_port->open(QSerialPort::ReadWrite)) {
        ui->textEdit->append(QString("GPS: порт %1 відкрито").arg(portName));
    } else {
        ui->textEdit->append(QString("GPS: помилка відкриття порту %1").arg(portName));
    }
}


// ============================================================
// ПРЕОБРАЗОВАНИЕ ДЕСЯТИЧНЫХ ГРАДУСОВ В ФОРМАТ NMEA (ddmm.mmmm)
// ============================================================
// Вход: значение в десятичных градусах, флаг isLatitude (true - широта, false - долгота)
// Выход: строка в формате:
//   - широта: ddmm.mmmm (2 цифры градусов + минуты с 4 знаками)
//   - долгота: dddmm.mmmm (3 цифры градусов + минуты с 4 знаками)
// ============================================================
QString MainWindow::formatLatLon(double value, bool isLatitude)
{
    // Берём абсолютное значение
    double absValue = qAbs(value);

    // Отделяем градусы от минут
    int degrees = static_cast<int>(absValue);
    double minutes = (absValue - degrees) * 60.0;

    // Формируем строку: градусы (2 или 3 цифры) + минуты (2 цифры + 4 знака после запятой)
    QString result;
    if (isLatitude) {
        // Широта: 2 цифры градусов (00-90)
        result = QString("%1%2").arg(degrees, 2, 10, QChar('0'))
                     .arg(minutes, 2, 'f', 4, QChar('0'));
    } else {
        // Долгота: 3 цифры градусов (000-180)
        result = QString("%1%2").arg(degrees, 3, 10, QChar('0'))
                     .arg(minutes, 2, 'f', 4, QChar('0'));
    }

    return result;
}

// ============================================================
// РАСЧЁТ КОНТРОЛЬНОЙ СУММЫ NMEA (XOR)
// ============================================================
// Вход: строка пакета без '*' и контрольной суммы
// Выход: контрольная сумма в виде двух HEX-символов (например, "5C")
// ============================================================
QString MainWindow::calculateNmeaChecksum(const QString &sentence)
{
    // Ищем символ '$' — начало пакета
    int start = sentence.indexOf('$');
    if (start == -1) {
        return "00";
    }

    // Начинаем XOR со следующего символа после '$'
    quint8 checksum = 0;
    for (int i = start + 1; i < sentence.length(); ++i) {
        checksum ^= static_cast<quint8>(sentence[i].toLatin1());
    }

    // Возвращаем в виде двух HEX-символов (верхний регистр)
    return QString("%1").arg(checksum, 2, 16, QChar('0')).toUpper();
}



// ============================================================
// ВЫБОР ПОРТА ДЛЯ РАДІОЛІНІЇ (ПРИЙОМ ДАНИХ ВІД ЛОГГЕРА)
// ============================================================
// Вызывается при выборе порта в comboBox_radioline.
// Открывает порт с настройками, соответствующими протоколу логгера.
// Порт работает в режиме ReadWrite для возможности отправки команд на логгер.
// ============================================================
void MainWindow::onRadiolinePortSelected(int index)
{
    Q_UNUSED(index)  // индекс не нужен, берём текст из comboBox

    // Получаем имя порта из comboBox
    QString portName = ui->comboBox_radioline->currentText();

    // Если выбран "NOT SELECT" или пустая строка — ничего не делаем
    if (portName == "NOT SELECT" || portName.isEmpty()) {
        return;
    }

    // Если порт уже открыт — закрываем его (для переподключения)
    if (logger_rx_port->isOpen()) {
        logger_rx_port->close();
    }

    // Настраиваем порт
    // Скорость и параметры должны соответствовать настройкам логгера
    logger_rx_port->setPortName(portName);
    logger_rx_port->setBaudRate(QSerialPort::Baud115200);   // скорость, как у логгера
    logger_rx_port->setDataBits(QSerialPort::Data8);        // 8 бит данных
    logger_rx_port->setParity(QSerialPort::NoParity);       // без контроля чётности
    logger_rx_port->setStopBits(QSerialPort::OneStop);      // 1 стоп-бит
    logger_rx_port->setFlowControl(QSerialPort::NoFlowControl); // без аппаратного контроля

    // Открываем порт в режиме чтения/записи
    // ReadWrite нужен для возможной отправки команд на логгер (управление с земли)
    if (logger_rx_port->open(QSerialPort::ReadWrite)) {
        ui->textEdit->append(QString("Радіолінія: порт %1 відкрито").arg(portName));
    }

    else {
        ui->textEdit->append(QString("Радіолінія: помилка відкриття порту %1").arg(portName));
    }
}


// ============================================================
// ОБРАБОТЧИК ТАЙМЕРА GPS (ОТПРАВКА ДАННЫХ)
// ============================================================
// Вызывается каждую секунду (1 Гц).
// Формирует NMEA-пакеты в зависимости от режима:
//   - Упрощённый: только GGA и GLL
//   - Реалистичный: GGA, GLL, RMC, GSA, GSV (GPS+GLONASS), VTG, TXT (случайно)
// ============================================================
void MainWindow::onGpsTimerTick()
{
    // --- 1. ПРОВЕРКА: активен ли GPS ---
    if (!gps_is_active) {
        return;
    }

    // --- 2. ПРОВЕРКА: порт открыт? ---
    if (!gps_port || !gps_port->isOpen()) {
        return;
    }

    // --- 3. ПРОВЕРКА: чекбокс "Відсутність зв'язку" ---
    if (ui->checkBox_not_ans_gps->isChecked()) {
        return;
    }

    // --- 4. ПОЛУЧАЕМ ДАННЫЕ ИЗ ТЕКУЩЕЙ ТОЧКИ ---
    PointData currentPoint = m_surveyArea.getCurrentPoint();
    double latitude = currentPoint.getLatitude();
    double longitude = currentPoint.getLongitude();
    double altitude = currentPoint.getHeight();

    // --- 5. ОБРАБОТКА РЕЖИМОВ ИМИТАЦИИ ---
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

    // --- 6. ФОРМИРУЕМ ВРЕМЯ UTC ---
    QDateTime utcNow = QDateTime::currentDateTimeUtc();
    QTime utcTime = utcNow.time();
    if (ui->checkBox_spoofing_gps->isChecked()) {
        QRandomGenerator *rand = QRandomGenerator::global();
        int hourOffset = (rand->generate() % 3) + 1;
        if (rand->generate() % 2 == 0) hourOffset = -hourOffset;
        utcTime = utcTime.addSecs(hourOffset * 3600);
    }
    QString utcTimeStr = utcTime.toString("hhmmss.00");

    // --- 7. ФОРМАТИРУЕМ КООРДИНАТЫ ---
    QString latNmea = formatLatLon(latitude, true);
    QString lonNmea = formatLatLon(longitude, false);
    char latDir = (latitude >= 0) ? 'N' : 'S';
    char lonDir = (longitude >= 0) ? 'E' : 'W';

    // --- 8. ПАРАМЕТРЫ ИЗ ИНТЕРФЕЙСА ---
    int satellites = ui->spinBox_satellites_gps->value();
    double hdop = ui->doubleSpinBox_hdop_gps->value();
    QString status = ui->checkBox_status_valid_gps->isChecked() ? "A" : "V";
    QString mode = ui->checkBox_mode_valid_gps->isChecked() ? "A" : "N";

    // --- 9. ФОРМИРУЕМ БАЗОВЫЕ ПАКЕТЫ (GGA И GLL) ---
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

    // --- 10. ОТПРАВКА И ВЫВОД В ЗАВИСИМОСТИ ОТ РЕЖИМА ---

    // Сначала выводим разделитель перед новым набором пакетов (кроме первого)
    static bool firstPacket = true;
    if (!firstPacket) {
        ui->textEdit->append("----------------------------------------");
    }
    firstPacket = false;

    if (ui->checkBox_realistic_gps->isChecked()) {
        // ------ РЕАЛИСТИЧНЫЙ РЕЖИМ (все пакеты) ------

        // RMC
        QString dateStr = utcNow.toString("ddMMyy");
        QString rmc = QString("$GNRMC,%1,%2,%3,%4,%5,%6,0.0,0.0,%7,,,A*")
                          .arg(utcTimeStr)
                          .arg(status)
                          .arg(latNmea).arg(latDir)
                          .arg(lonNmea).arg(lonDir)
                          .arg(dateStr);
        rmc.replace("*", "*" + calculateNmeaChecksum(rmc));

        // GSA
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

        // GPGSV
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

        // GLGSV
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

        // VTG
        QString vtg = QString("$GNVTG,0.0,T,0.0,M,0.0,N,0.0,K,N*");
        vtg += calculateNmeaChecksum(vtg);

        // TXT (30% вероятность)
        QString txt;
        if (rand->generateDouble() < 0.3) {
            txt = QString("$GNTXT,01,01,01,More than 100 frame errors, UART RX was disabled*");
            txt += calculateNmeaChecksum(txt);
        }

        // --- ОТПРАВКА ---
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

        // --- ВЫВОД ВСЕХ ПАКЕТОВ В textEdit ---
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
        // ------ УПРОЩЁННЫЙ РЕЖИМ (только GGA и GLL) ------
        gps_port->write((gga + "\r\n").toUtf8());
        gps_port->write((gll + "\r\n").toUtf8());

        ui->textEdit->append("GPS (спрощений): " + gga.trimmed());
        ui->textEdit->append("GPS (спрощений): " + gll.trimmed());
    }
}









// ============================================================
// ОБРАБОТЧИК КНОПКИ "ВИВЕСТИ ПАКЕТИ GPS"
// ============================================================
// Выводит в textEdit описание всех пакетов, которые будут
// отправляться в упрощённом и реалистичном режимах.
// Полезно для разработчика логгера.
// ============================================================
void MainWindow::on_pushButton_show_gps_packages_clicked()
{
    ui->textEdit->clear();

    ui->textEdit->append("\n========== GPS ПАКЕТИ (ОПИС) ==========\n");

    // --- 1. ОБЩАЯ ИНФОРМАЦИЯ ---
    ui->textEdit->append("Частота: 1 Гц (один раз на секунду)");
    ui->textEdit->append("Протокол: NMEA 0183");
    ui->textEdit->append("Швидкість порту: 4800 бод");
    ui->textEdit->append("Формат: ASCII, кожен пакет закінчується CR+LF (\\r\\n)");
    ui->textEdit->append("Контрольна сума: XOR (всі байти між $ та *)\n");

    // --- 2. УПРОЩЁННЫЙ РЕЖИМ ---
    ui->textEdit->append("--- CПРОЩЕНИЙ РЕЖИМ (2 пакети) ---\n");

    ui->textEdit->append("1. $GNGGA - основні дані GPS");
    ui->textEdit->append("   Формат: $GNGGA,UTC,lat,N,lon,E,quality,satellites,HDOP,altitude,M,geoid,M,,*CS");
    ui->textEdit->append("   Поля:");
    ui->textEdit->append("     - UTC час (hhmmss.ss)");
    ui->textEdit->append("     - Широта (ddmm.mmmm) та напрямок (N/S)");
    ui->textEdit->append("     - Довгота (dddmm.mmmm) та напрямок (E/W)");
    ui->textEdit->append("     - Якість фіксації (1 - GPS fix)");
    ui->textEdit->append("     - Кількість супутників (з інтерфейсу)");
    ui->textEdit->append("     - HDOP (з інтерфейсу)");
    ui->textEdit->append("     - Висота над рівнем моря (з поточної точки)");
    ui->textEdit->append("     - Висота геоїда (фіксовано: 46.9)");
    ui->textEdit->append("     - Контрольна сума (XOR)\n");

    ui->textEdit->append("2. $GNGLL - географічна позиція");
    ui->textEdit->append("   Формат: $GNGLL,lat,N,lon,E,UTC,status,mode,*CS");
    ui->textEdit->append("   Поля:");
    ui->textEdit->append("     - Широта (ddmm.mmmm) та напрямок (N/S)");
    ui->textEdit->append("     - Довгота (dddmm.mmmm) та напрямок (E/W)");
    ui->textEdit->append("     - UTC час (hhmmss.ss)");
    ui->textEdit->append("     - Статус: A (валідний) / V (невалідний) — з чекбоксу");
    ui->textEdit->append("     - Режим: A (автономний) / N (невалідний) — з чекбоксу");
    ui->textEdit->append("     - Контрольна сума (XOR)\n");

    // --- 3. РЕАЛИСТИЧНЫЙ РЕЖИМ ---
    ui->textEdit->append("--- РЕАЛІСТИЧНИЙ РЕЖИМ (7-8 пакетів) ---\n");

    ui->textEdit->append("3. $GNRMC - рекомендовані мінімальні дані");
    ui->textEdit->append("   Формат: $GNRMC,UTC,status,lat,N,lon,E,speed,course,date,,,A,*CS");
    ui->textEdit->append("   Поля:");
    ui->textEdit->append("     - UTC час (hhmmss.ss)");
    ui->textEdit->append("     - Статус: A (валідний) / V (невалідний)");
    ui->textEdit->append("     - Широта (ddmm.mmmm) та напрямок (N/S)");
    ui->textEdit->append("     - Довгота (dddmm.mmmm) та напрямок (E/W)");
    ui->textEdit->append("     - Швидкість (вузли) — 0.0");
    ui->textEdit->append("     - Курс (градуси) — 0.0");
    ui->textEdit->append("     - Дата (ddmmyy)");
    ui->textEdit->append("     - Контрольна сума (XOR)\n");

    ui->textEdit->append("4. $GNGSA - активні супутники");
    ui->textEdit->append("   Формат: $GNGSA,A,1,PRN1,PRN2,...,PRN12,HDOP,PDOP,VDOP,*CS");
    ui->textEdit->append("   Поля:");
    ui->textEdit->append("     - Режим: A (автономний)");
    ui->textEdit->append("     - Тип фіксації: 1 (без фіксації)");
    ui->textEdit->append("     - PRN-номера супутників (12 полів, з інтерфейсу)");
    ui->textEdit->append("     - HDOP (з інтерфейсу)");
    ui->textEdit->append("     - PDOP (≈ HDOP * 1.1)");
    ui->textEdit->append("     - VDOP (≈ HDOP * 0.7)");
    ui->textEdit->append("     - Контрольна сума (XOR)\n");

    ui->textEdit->append("5. $GPGSV - видимі супутники GPS");
    ui->textEdit->append("   Формат: $GPGSV,1,1,count,PRN,elev,azim,SNR,...*CS");
    ui->textEdit->append("   Поля:");
    ui->textEdit->append("     - Кількість супутників (до 4)");
    ui->textEdit->append("     - Для кожного: PRN, кут місця, азимут, SNR (випадкові)");
    ui->textEdit->append("     - Контрольна сума (XOR)\n");

    ui->textEdit->append("6. $GLGSV - видимі супутники GLONASS");
    ui->textEdit->append("   Формат: $GLGSV,1,1,count,PRN,elev,azim,SNR,...*CS");
    ui->textEdit->append("   Поля:");
    ui->textEdit->append("     - Кількість супутників (0-2)");
    ui->textEdit->append("     - Для кожного: PRN, кут місця, азимут, SNR (випадкові)");
    ui->textEdit->append("     - Контрольна сума (XOR)\n");

    ui->textEdit->append("7. $GNVTG - курс та швидкість");
    ui->textEdit->append("   Формат: $GNVTG,course,T,course,M,speed,N,speed,K,mode,*CS");
    ui->textEdit->append("   Поля:");
    ui->textEdit->append("     - Курс (градуси) — 0.0");
    ui->textEdit->append("     - Швидкість (вузли) — 0.0");
    ui->textEdit->append("     - Швидкість (км/год) — 0.0");
    ui->textEdit->append("     - Режим: N (невалідний)");
    ui->textEdit->append("     - Контрольна сума (XOR)\n");

    ui->textEdit->append("8. $GNTXT - текстове повідомлення (30% випадків)");
    ui->textEdit->append("   Формат: $GNTXT,01,01,01,Message,*CS");
    ui->textEdit->append("   Поля:");
    ui->textEdit->append("     - Повідомлення: 'More than 100 frame errors, UART RX was disabled'");
    ui->textEdit->append("     - Контрольна сума (XOR)\n");

    // --- 4. ПРИМЕРЫ ---
    ui->textEdit->append("--- ПРИКЛАДИ ПАКЕТІВ ---\n");

    // Формируем примеры на основе текущих данных (или фиктивных)
    QString exampleLat = formatLatLon(49.990000, true);
    QString exampleLon = formatLatLon(24.023239, false);

    ui->textEdit->append("Упрощений режим (GGA + GLL):");
    ui->textEdit->append(QString("  $GNGGA,123519.00,%1,N,%2,E,1,08,0.9,545.4,M,46.9,M,,*59")
                             .arg(exampleLat).arg(exampleLon));
    ui->textEdit->append(QString("  $GNGLL,%1,N,%2,E,123519.00,A,A*5C")
                             .arg(exampleLat).arg(exampleLon));
    ui->textEdit->append("\n-------------------------------\n");

    ui->textEdit->append("Реалістичний режим (повний набір):");
    ui->textEdit->append(QString("  $GNGGA,123519.00,%1,N,%2,E,1,08,0.9,545.4,M,46.9,M,,*59")
                             .arg(exampleLat).arg(exampleLon));
    ui->textEdit->append(QString("  $GNGLL,%1,N,%2,E,123519.00,A,A*5C")
                             .arg(exampleLat).arg(exampleLon));
    ui->textEdit->append("  $GNRMC,123519.00,A,4807.038,N,01131.000,E,0.0,0.0,160726,,,A*6D");
    ui->textEdit->append("  $GNGSA,A,1,06,07,30,81,76,,,,,,,,5.51,4.27,3.48*19");
    ui->textEdit->append("  $GPGSV,1,1,04,06,37,236,44,07,59,190,46,30,20,213,35,36,32,170,43*7E");
    ui->textEdit->append("  $GLGSV,1,1,02,76,42,134,26,81,23,242,39*64");
    ui->textEdit->append("  $GNVTG,0.0,T,0.0,M,0.0,N,0.0,K,N*2E");
    ui->textEdit->append("  $GNTXT,01,01,01,More than 100 frame errors, UART RX was disabled*70");

    ui->textEdit->append("\n========== КІНЕЦЬ ОПИСУ ==========\n");
}

// ============================================================
// ОБРАБОТЧИК КНОПКИ "ВИВЕСТИ ПАКЕТИ LIDAR"
// ============================================================
// Выводит в textEdit описание пакетов LIDAR.
// Полезно для разработчика логгера.
// ============================================================
void MainWindow::on_pushButton_show_lidar_packages_clicked()
{
    ui->textEdit->clear();

    ui->textEdit->append("\n========== LIDAR ПАКЕТИ (ОПИС) ==========\n");

    // --- 1. ОБЩАЯ ИНФОРМАЦИЯ ---
    ui->textEdit->append("Протокол: власний (8 байт даних + 1 байт контрольна сума)");
    ui->textEdit->append("Частота: 1 Гц або 100 Гц (вибирається в інтерфейсі)");
    ui->textEdit->append("Швидкість порту: 115200 бод");
    ui->textEdit->append("Формат: бінарний, 9 байт на пакет");
    ui->textEdit->append("Контрольна сума: сума всіх байтів (0-7) & 0xFF\n");

    // --- 2. СТРУКТУРА ПАКЕТА ---
    ui->textEdit->append("--- СТРУКТУРА ПАКЕТА (9 байт) ---\n");

    ui->textEdit->append("Байт 0: 0x59 (заголовок)");
    ui->textEdit->append("Байт 1: 0x59 (заголовок)");
    ui->textEdit->append("Байт 2: Distance Low (молодший байт відстані)");
    ui->textEdit->append("Байт 3: Distance High (старший байт відстані)");
    ui->textEdit->append("Байт 4: Strength Low (молодший байт інтенсивності)");
    ui->textEdit->append("Байт 5: Strength High (старший байт інтенсивності)");
    ui->textEdit->append("Байт 6: Reserved (зарезервовано, 0x00)");
    ui->textEdit->append("Байт 7: Reserved (зарезервовано, 0x00)");
    ui->textEdit->append("Байт 8: Checksum (сума байтів 0-7, & 0xFF)\n");

    // --- 3. РЕЖИМЫ РАБОТЫ ---
    ui->textEdit->append("--- РЕЖИМИ РОБОТИ (вибираються в comboBox) ---\n");

    ui->textEdit->append("1. Нормальне вимірювання");
    ui->textEdit->append("   - Distance: реальна висота з поточної точки (см)");
    ui->textEdit->append("   - Strength: 0x0FFF (нормальна інтенсивність)\n");

    ui->textEdit->append("2. Об'єкт не виявлено");
    ui->textEdit->append("   - Distance: 0x0000 (об'єкт відсутній)");
    ui->textEdit->append("   - Strength: 0x00D2 (слабкий сигнал < 255)\n");

    ui->textEdit->append("3. Слабке відбиття");
    ui->textEdit->append("   - Distance: реальна висота з поточної точки (см)");
    ui->textEdit->append("   - Strength: 0x00D2 (слабкий сигнал < 255)\n");

    ui->textEdit->append("4. Сильне відбиття");
    ui->textEdit->append("   - Distance: реальна висота з поточної точки (см)");
    ui->textEdit->append("   - Strength: 0xBFFF (сильний сигнал > 0xAFFF)\n");

    ui->textEdit->append("5. Об'єкт дуже близько");
    ui->textEdit->append("   - Distance: 0x0010 (менше 0x20)");
    ui->textEdit->append("   - Strength: 0x0FFF (нормальна інтенсивність)\n");

    ui->textEdit->append("6. Об'єкт занадто далеко");
    ui->textEdit->append("   - Distance: 0xFFFF (максимальне значення)");
    ui->textEdit->append("   - Strength: 0x0FFF (нормальна інтенсивність)\n");

    // --- 4. НЕИСПРАВНОСТИ ---
    ui->textEdit->append("--- НЕІСПРАВНОСТІ (чекбокси) ---\n");

    ui->textEdit->append("1. Відсутність зв'язку");
    ui->textEdit->append("   - Дані не передаються (імітація обриву зв'язку)\n");

    ui->textEdit->append("2. Некоректна відповідь");
    ui->textEdit->append("   - Пакет передається з неправильною контрольною сумою (Checksum інвертована)\n");

    // --- 5. ПРИМЕРЫ ПАКЕТОВ ---
    ui->textEdit->append("--- ПРИКЛАДИ ПАКЕТІВ (HEX) ---\n");

    // Нормальне вимірювання (висота 545.4 м = 54540 см)
    QByteArray normalPacket;
    normalPacket.append(static_cast<char>(0x59));
    normalPacket.append(static_cast<char>(0x59));
    normalPacket.append(static_cast<char>(0x4C)); // 54540 & 0xFF
    normalPacket.append(static_cast<char>(0xD5)); // (54540 >> 8) & 0xFF
    normalPacket.append(static_cast<char>(0xFF)); // Strength Low (0x0FFF)
    normalPacket.append(static_cast<char>(0x0F)); // Strength High (0x0FFF)
    normalPacket.append(static_cast<char>(0x00));
    normalPacket.append(static_cast<char>(0x00));
    quint8 checksum = 0;
    for (int i = 0; i < 8; ++i) {
        checksum += static_cast<quint8>(normalPacket[i]);
    }
    normalPacket.append(static_cast<char>(checksum));
    ui->textEdit->append("Нормальне вимірювання:");
    ui->textEdit->append("  " + QString(normalPacket.toHex().toUpper()));

    // Об'єкт не виявлено
    QByteArray noObjectPacket;
    noObjectPacket.append(static_cast<char>(0x59));
    noObjectPacket.append(static_cast<char>(0x59));
    noObjectPacket.append(static_cast<char>(0x00));
    noObjectPacket.append(static_cast<char>(0x00));
    noObjectPacket.append(static_cast<char>(0xD2));
    noObjectPacket.append(static_cast<char>(0x00));
    noObjectPacket.append(static_cast<char>(0x00));
    noObjectPacket.append(static_cast<char>(0x00));
    checksum = 0;
    for (int i = 0; i < 8; ++i) {
        checksum += static_cast<quint8>(noObjectPacket[i]);
    }
    noObjectPacket.append(static_cast<char>(checksum));
    ui->textEdit->append("Об'єкт не виявлено:");
    ui->textEdit->append("  " + QString(noObjectPacket.toHex().toUpper()));

    // Слабке відбиття
    QByteArray weakPacket;
    weakPacket.append(static_cast<char>(0x59));
    weakPacket.append(static_cast<char>(0x59));
    weakPacket.append(static_cast<char>(0x4C));
    weakPacket.append(static_cast<char>(0xD5));
    weakPacket.append(static_cast<char>(0xD2));
    weakPacket.append(static_cast<char>(0x00));
    weakPacket.append(static_cast<char>(0x00));
    weakPacket.append(static_cast<char>(0x00));
    checksum = 0;
    for (int i = 0; i < 8; ++i) {
        checksum += static_cast<quint8>(weakPacket[i]);
    }
    weakPacket.append(static_cast<char>(checksum));
    ui->textEdit->append("Слабке відбиття:");
    ui->textEdit->append("  " + QString(weakPacket.toHex().toUpper()));

    // Сильне відбиття
    QByteArray strongPacket;
    strongPacket.append(static_cast<char>(0x59));
    strongPacket.append(static_cast<char>(0x59));
    strongPacket.append(static_cast<char>(0x4C));
    strongPacket.append(static_cast<char>(0xD5));
    strongPacket.append(static_cast<char>(0xFF));
    strongPacket.append(static_cast<char>(0xBF));
    strongPacket.append(static_cast<char>(0x00));
    strongPacket.append(static_cast<char>(0x00));
    checksum = 0;
    for (int i = 0; i < 8; ++i) {
        checksum += static_cast<quint8>(strongPacket[i]);
    }
    strongPacket.append(static_cast<char>(checksum));
    ui->textEdit->append("Сильне відбиття:");
    ui->textEdit->append("  " + QString(strongPacket.toHex().toUpper()));

    // Об'єкт дуже близько
    QByteArray closePacket;
    closePacket.append(static_cast<char>(0x59));
    closePacket.append(static_cast<char>(0x59));
    closePacket.append(static_cast<char>(0x10));
    closePacket.append(static_cast<char>(0x00));
    closePacket.append(static_cast<char>(0xFF));
    closePacket.append(static_cast<char>(0x0F));
    closePacket.append(static_cast<char>(0x00));
    closePacket.append(static_cast<char>(0x00));
    checksum = 0;
    for (int i = 0; i < 8; ++i) {
        checksum += static_cast<quint8>(closePacket[i]);
    }
    closePacket.append(static_cast<char>(checksum));
    ui->textEdit->append("Об'єкт дуже близько:");
    ui->textEdit->append("  " + QString(closePacket.toHex().toUpper()));

    // Об'єкт занадто далеко
    QByteArray farPacket;
    farPacket.append(static_cast<char>(0x59));
    farPacket.append(static_cast<char>(0x59));
    farPacket.append(static_cast<char>(0xFF));
    farPacket.append(static_cast<char>(0xFF));
    farPacket.append(static_cast<char>(0xFF));
    farPacket.append(static_cast<char>(0x0F));
    farPacket.append(static_cast<char>(0x00));
    farPacket.append(static_cast<char>(0x00));
    checksum = 0;
    for (int i = 0; i < 8; ++i) {
        checksum += static_cast<quint8>(farPacket[i]);
    }
    farPacket.append(static_cast<char>(checksum));
    ui->textEdit->append("Об'єкт занадто далеко:");
    ui->textEdit->append("  " + QString(farPacket.toHex().toUpper()));

    // --- 6. ПОЯСНЕННЯ КОНТРОЛЬНОЇ СУМИ ---
    ui->textEdit->append("\n--- КОНТРОЛЬНА СУМА (Checksum) ---");
    ui->textEdit->append("Checksum = (байт0 + байт1 + ... + байт7) & 0xFF");
    ui->textEdit->append("Приклад для нормального вимірювання:");
    ui->textEdit->append("  0x59 + 0x59 + 0x4C + 0xD5 + 0xFF + 0x0F + 0x00 + 0x00 = ...");
    ui->textEdit->append("  Результат & 0xFF = " + QString::number(checksum, 16).toUpper());

    ui->textEdit->append("\n========== КІНЕЦЬ ОПИСУ ==========\n");
}


// ============================================================
// ОБРАБОТЧИК КНОПКИ "ПОКАЗАТИ ІНСТРУКЦІЮ"
// ============================================================
// Выводит в textEdit подробную инструкцию по работе с программой.
// ============================================================
void MainWindow::on_pushButton_show_instruction_clicked()
{
    ui->textEdit->clear();

    ui->textEdit->append("====================================================");
    ui->textEdit->append("              ІНСТРУКЦІЯ З РОБОТИ З ІМІТАТОРОМ");
    ui->textEdit->append("====================================================");

    ui->textEdit->append("\n--- 1. ПІДКЛЮЧЕННЯ ПЕРЕХІДНИКІВ ---");
    ui->textEdit->append("  - Підключіть перехідник RS485-USB до комп'ютера.");
    ui->textEdit->append("  - Ім'я порту (наприклад, COM5) автоматично з'явиться у полі \"lineEdit_COM\".");
    ui->textEdit->append("  - Виберіть цей порт у комбобоксі потрібного приладу:");
    ui->textEdit->append("      * comboBox_BDBG_1 — для БДБГ (нижній/верхній на одній шині)");
    ui->textEdit->append("      * comboBox_LIDAR — для LIDAR");
    ui->textEdit->append("      * comboBox_GPS — для GPS");
    ui->textEdit->append("  - Якщо потрібно перевірити інший прилад — від'єднайте перехідник,");
    ui->textEdit->append("    підключіть його до іншого приладу та повторіть процедуру.");

    ui->textEdit->append("\n--- 2. СТВОРЕННЯ ОБЛАСТІ РОЗВІДКИ ---");
    ui->textEdit->append("  - Заповніть параметри в блоці \"Параметри розвідки\":");
    ui->textEdit->append("      * Широта старту (наприклад, 49.990000)");
    ui->textEdit->append("      * Довгота старту (наприклад, 24.023239)");
    ui->textEdit->append("      * Ребро квадрата (км) (наприклад, 10)");
    ui->textEdit->append("      * Кількість точок замірів (наприклад, 50)");
    ui->textEdit->append("      * ПАЕД макс. (наприклад, 10.0)");
    ui->textEdit->append("      * ПАЕД фон (наприклад, 0.1)");
    ui->textEdit->append("      * Висота старту (м) (наприклад, 100)");
    ui->textEdit->append("  - Виберіть режим розвідки: \"Поверхня\" або \"Хмара\".");
    ui->textEdit->append("  - Для режиму \"Хмара\" додатково заповніть:");
    ui->textEdit->append("      * Кількість кроків підйому-спуску (наприклад, 5)");
    ui->textEdit->append("      * Верхній поріг хмари (м) (наприклад, 2000)");
    ui->textEdit->append("      * Коефіціент збільшення хмари (наприклад, 5.0)");
    ui->textEdit->append("  - Натисніть кнопку \"Створити область розвідки\".");

    ui->textEdit->append("\n--- 3. НАЛАШТУВАННЯ ПРИЛАДІВ ---");
    ui->textEdit->append("  - Виберіть COM-порт для кожного приладу:");
    ui->textEdit->append("      * БДБГ-1: comboBox_BDBG_1");
    ui->textEdit->append("      * БДБГ-2: comboBox_BDBG_2 (одна шина, два адреси)");
    ui->textEdit->append("      * LIDAR: comboBox_LIDAR");
    ui->textEdit->append("      * GPS: comboBox_GPS");

    ui->textEdit->append("\n--- 4. НАЛАШТУВАННЯ РЕЖИМІВ ІМІТАЦІЇ ---");
    ui->textEdit->append("  LIDAR:");
    ui->textEdit->append("      * Виберіть режим роботи (нормальне вимірювання, об'єкт не виявлено, тощо)");
    ui->textEdit->append("      * Виберіть частоту (1 Гц або 100 Гц)");
    ui->textEdit->append("  GPS:");
    ui->textEdit->append("      * Увімкніть/вимкніть \"Реалістичний потік\" для повного набору пакетів");
    ui->textEdit->append("      * Налаштуйте чекбокси: несправна антена, спуфінг, статус, режим, тощо");
    ui->textEdit->append("      * Встановіть кількість супутників та HDOP");
    ui->textEdit->append("  БДБГ:");
    ui->textEdit->append("      * Чекбокси для кожного адресу: відсутність зв'язку, некорентка відповідь,");
    ui->textEdit->append("        відмова високочутливого/низькочутливого детектора");

    ui->textEdit->append("\n--- 5. ЗАПУСК РОЗВІДКИ ---");
    ui->textEdit->append("  - Виберіть режим пересування дрона: \"Ручний\" або \"Авто\".");
    ui->textEdit->append("  - Якщо \"Авто\" — встановіть інтервал опиту (сек).");
    ui->textEdit->append("  - Натисніть кнопку \"Почати розвідку\".");
    ui->textEdit->append("  - У полі textEdit з'являться HEX-пакети LIDAR, NMEA-пакети GPS, та відповіді БДБГ.");

    ui->textEdit->append("\n--- 6. РУЧНЕ ПЕРЕМІЩЕННЯ ДРОНА ---");
    ui->textEdit->append("  - Якщо вибрано режим \"Ручний\":");
    ui->textEdit->append("      * \"Зробити крок\" — перехід до наступної точки (горизонталь)");
    ui->textEdit->append("      * \"Піднятись на 1 рівень\" — збільшення висоти (тільки для Хмара)");
    ui->textEdit->append("      * \"Спуститись на 1 рівень\" — зменшення висоти (тільки для Хмара)");
    ui->textEdit->append("      * \"Спуститись на початкову висоту\" — скидання висоти до мінімальної");

    ui->textEdit->append("\n--- 7. ЗУПИНКА РОЗВІДКИ ---");
    ui->textEdit->append("  - Натисніть кнопку \"Зупинити розвідку\" (вона замінює \"Почати розвідку\").");
    ui->textEdit->append("  - Зупиняються всі передачі даних (LIDAR, GPS, БДБГ) та авто-переміщення.");

    ui->textEdit->append("\n--- 8. ПЕРЕВІРКА РОБОТИ ЛОГГЕРА (SD-КАРТА ТА РАДІОЛІНІЯ) ---");
    ui->textEdit->append("  - Після завершення розвідки:");
    ui->textEdit->append("      * Вставте SD-карту з логгера в комп'ютер.");
    ui->textEdit->append("      * Натисніть кнопку \"Завантажити SD-карту\" та виберіть файл.");
    ui->textEdit->append("      * Натисніть кнопку \"Порівняти дані\" для порівняння еталона, SD та радіолінії.");

    ui->textEdit->append("\n--- 9. ДОДАТКОВІ КНОПКИ ---");
    ui->textEdit->append("  - \"Вивести пакети LIDAR\" — опис пакетів LIDAR");
    ui->textEdit->append("  - \"Вивести пакети GPS\" — опис пакетів GPS");
    ui->textEdit->append("  - \"Очистити вікно текстової інформації\" — очищення textEdit");

    ui->textEdit->append("\n====================================================");
    ui->textEdit->append("              Бажаємо успішної роботи!");
    ui->textEdit->append("====================================================");
}


// ============================================================
// ПАРСИНГ СТРОКИ ИЗ ФОРМАТА SD-КАРТЫ / РАДИОЛИНИИ
// ============================================================
// Формат строки (соответствует sprintf в логгере):
//   номер_строки,temp0,temp1,ped0,ped1,longitude,latitude,lidar_distance,lidar_strength
//
// Пример: 1,90.00,90.00,0.10,0.10,24.023239,49.990000,10000,4095
//
// Поля:
//   lineNumber    - сквозной номер записи (начинается с 1)
//   temp0         - температура БДБГ-1 (нижний дозиметр) в °C
//   temp1         - температура БДБГ-2 (верхний дозиметр) в °C
//   ped0          - ПЕД нижнего дозиметра (pedLower) в мкЗв/ч
//   ped1          - ПЕД верхнего дозиметра (pedUpper) в мкЗв/ч
//   longitude     - долгота в десятичных градусах
//   latitude      - широта в десятичных градусах
//   lidarDistance - расстояние от LIDAR в сантиметрах (целое число)
//   lidarStrength - интенсивность отражённого сигнала LIDAR (целое число)
//
// Вход:  QString line - строка для парсинга
// Выход: SdRecord &record - структура с распарсенными данными (передаётся по ссылке)
// Возврат: true если парсинг успешен, false если ошибка
// ============================================================
bool MainWindow::parseSdLine(const QString &line, SdRecord &record)
{
    // Разбиваем строку по запятым на отдельные поля
    // Пример: "1,90.00,90.00,0.10,0.10,24.023239,49.990000,10000,4095"
    // После split получим QStringList с 9 элементами
    QStringList fields = line.split(',');

    // Проверяем, что количество полей соответствует ожидаемому (9 полей)
    // Если полей меньше или больше — строка повреждена или не соответствует формату
    if (fields.size() != 9) {
        return false;
    }

    bool ok;  // Переменная для проверки успешности преобразования строки в число

    // --- Парсим каждое поле по порядку ---

    // Поле 0: номер строки (lineNumber) — целое число
    // toInt() преобразует строку в int, ok = true если преобразование успешно
    record.lineNumber = fields[0].toInt(&ok);
    if (!ok) return false;  // Если не удалось преобразовать — выходим с ошибкой

    // Поле 1: temp0 — температура БДБГ-1 (дробное число)
    // toDouble() преобразует строку в double
    record.temp0 = fields[1].toDouble(&ok);
    if (!ok) return false;

    // Поле 2: temp1 — температура БДБГ-2
    record.temp1 = fields[2].toDouble(&ok);
    if (!ok) return false;

    // Поле 3: ped0 — ПЕД нижнего дозиметра
    record.ped0 = fields[3].toDouble(&ok);
    if (!ok) return false;

    // Поле 4: ped1 — ПЕД верхнего дозиметра
    record.ped1 = fields[4].toDouble(&ok);
    if (!ok) return false;

    // Поле 5: longitude — долгота
    record.longitude = fields[5].toDouble(&ok);
    if (!ok) return false;

    // Поле 6: latitude — широта
    record.latitude = fields[6].toDouble(&ok);
    if (!ok) return false;

    // Поле 7: lidarDistance — расстояние LIDAR в сантиметрах (целое число)
    record.lidarDistance = fields[7].toInt(&ok);
    if (!ok) return false;

    // Поле 8: lidarStrength — интенсивность LIDAR (целое число)
    record.lidarStrength = fields[8].toInt(&ok);
    if (!ok) return false;

    // Все поля успешно распарсены — возвращаем true
    return true;
}


// ============================================================
// ЗАГРУЗКА ДАННЫХ С SD-КАРТЫ
// ============================================================
// Вызывается при нажатии кнопки "Завантажити SD-карту".
// Открывает диалог выбора файла, читает файл построчно,
// парсит каждую строку и сохраняет в m_sdRecords.
// ============================================================
void MainWindow::onLoadSdCardClicked()
{
    // Открываем диалог выбора файла с фильтром текстовых файлов
    QString filePath = QFileDialog::getOpenFileName(
        this,
        "Виберіть файл з SD-карти",           // Заголовок окна
        QDir::homePath(),                      // Начальная папка (домашняя)
        "Текстові файли (*.txt);;Усі файли (*.*)" // Фильтр типов файлов
        );

    // Если пользователь нажал "Отмена" или не выбрал файл — выходим
    if (filePath.isEmpty()) {
        ui->textEdit->append("SD-карта: файл не вибрано");
        return;
    }

    // Открываем файл для чтения в текстовом режиме
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        ui->textEdit->append("SD-карта: помилка відкриття файлу");
        return;
    }

    // Очищаем предыдущие данные с SD-карты
    m_sdRecords.clear();

    // Создаём поток для построчного чтения
    QTextStream in(&file);
    int lineNumber = 0;      // Счётчик строк (для вывода номеров при ошибках)

    // Читаем файл построчно до конца
    while (!in.atEnd()) {
        QString line = in.readLine();  // Читаем одну строку
        lineNumber++;

        // Пропускаем пустые строки
        if (line.trimmed().isEmpty()) {
            continue;
        }

        // Парсим строку в структуру SdRecord
        SdRecord record;
        if (parseSdLine(line, record)) {
            // Если парсинг успешен — добавляем запись в список
            m_sdRecords.append(record);
        } else {
            // Если парсинг не удался — выводим предупреждение в лог
            ui->textEdit->append(QString("SD-карта: помилка парсингу рядка %1: %2")
                                     .arg(lineNumber)
                                     .arg(line));
        }
    }

    // Закрываем файл
    file.close();

    // Выводим информацию о загруженных данных
    ui->textEdit->append(QString("SD-карта: завантажено %1 записів з файлу %2")
                             .arg(m_sdRecords.size())
                             .arg(QFileInfo(filePath).fileName()));
}


// ============================================================
// ОБРАБОТЧИК ПРИЁМА ДАННЫХ ПО РАДИОЛИНИИ
// ============================================================
// Вызывается, когда в порту logger_rx_port появляются данные.
// Читает данные из порта, парсит строки и сохраняет в m_radiolineRecords.
// Дополнительно выводит принятые данные в textEdit_radioline.
// ============================================================
// void MainWindow::onRadiolineDataReceived()
// {
//     // Проверяем, что порт существует и открыт
//     if (!logger_rx_port || !logger_rx_port->isOpen()) {
//         return;
//     }

//     // Читаем все доступные данные из порта
//     QByteArray data = logger_rx_port->readAll();

//     // Преобразуем байты в строку (предполагается текстовая кодировка UTF-8)
//     QString receivedText = QString::fromUtf8(data);

//     // Выводим принятый текст в отдельное окно textEdit_radioline
//     // trimmed() убирает лишние пробелы и переводы строк в конце
//     ui->textEdit_radioline->append(receivedText.trimmed());

//     // Разбиваем принятый текст на отдельные строки
//     // Qt::SkipEmptyParts пропускает пустые строки
//     QStringList lines = receivedText.split('\n', Qt::SkipEmptyParts);

//     // Обрабатываем каждую строку
//     for (const QString &line : lines) {
//         // Убираем лишние пробелы и символы возврата каретки (\r)
//         QString trimmedLine = line.trimmed();

//         SdRecord record;
//         if (parseSdLine(trimmedLine, record)) {
//             // Если парсинг успешен — добавляем запись в список
//             m_radiolineRecords.append(record);
//         } else {
//             // Если парсинг не удался — выводим предупреждение
//             ui->textEdit_radioline->append("Радіолінія: помилка парсингу: " + trimmedLine);
//         }
//     }
// }


void MainWindow::onRadiolineDataReceived()
{
    if (!logger_rx_port || !logger_rx_port->isOpen()) {
        return;
    }

    // Добавляем новые данные в буфер
    m_radioBuffer.append(logger_rx_port->readAll());

    // Цикл обработки пакетов в буфере
    while (true) {
        // --- 1. Ищем первый заголовок 0xAA 0x55 0xAA 0x55 ---
        int startPos = -1;
        for (int i = 0; i <= m_radioBuffer.size() - 4; ++i) {
            if (static_cast<unsigned char>(m_radioBuffer[i]) == 0xAA &&
                static_cast<unsigned char>(m_radioBuffer[i+1]) == 0x55 &&
                static_cast<unsigned char>(m_radioBuffer[i+2]) == 0xAA &&
                static_cast<unsigned char>(m_radioBuffer[i+3]) == 0x55) {
                startPos = i;
                break;
            }
        }

        // Если заголовок не найден — выходим (ждём новые данные)
        if (startPos == -1) {
            break;
        }

        // --- 2. Ищем окончание 0x0D 0x0A 0x0D 0x0A, начиная с позиции после заголовка ---
        int endPos = -1;
        int searchPos = startPos + 4;

        while (searchPos <= m_radioBuffer.size() - 4) {
            // Проверяем, не встретился ли новый заголовок до окончания
            if (static_cast<unsigned char>(m_radioBuffer[searchPos]) == 0xAA &&
                static_cast<unsigned char>(m_radioBuffer[searchPos+1]) == 0x55 &&
                static_cast<unsigned char>(m_radioBuffer[searchPos+2]) == 0xAA &&
                static_cast<unsigned char>(m_radioBuffer[searchPos+3]) == 0x55) {
                // Обнаружен новый заголовок — отбрасываем всё до него и начинаем заново
                m_radioBuffer.remove(0, searchPos);
                startPos = -1;  // Сброс для повторного поиска
                break;
            }

            // Проверяем окончание
            if (static_cast<unsigned char>(m_radioBuffer[searchPos]) == 0x0D &&
                static_cast<unsigned char>(m_radioBuffer[searchPos+1]) == 0x0A &&
                static_cast<unsigned char>(m_radioBuffer[searchPos+2]) == 0x0D &&
                static_cast<unsigned char>(m_radioBuffer[searchPos+3]) == 0x0A) {
                endPos = searchPos;
                break;
            }

            searchPos++;
        }

        // Если нашли новый заголовок и сбросили startPos — начинаем цикл заново
        if (startPos == -1) {
            continue;
        }

        // Если окончание не найдено — выходим (ждём новые данные)
        if (endPos == -1) {
            break;
        }

        // --- 3. Вырезаем пакет (от startPos до endPos + 4) ---
        int packetLength = endPos + 4 - startPos;
        QByteArray packet = m_radioBuffer.mid(startPos, packetLength);

        // // --- 4. Удаляем пакет из буфера ---
        // m_radioBuffer.remove(startPos, packetLength);

        // Удаляем всё от начала буфера до конца пакета (включая мусор перед заголовком)
        m_radioBuffer.remove(0, startPos + packetLength);

        // --- 5. Проверяем CRC ---
        // CRC рассчитывается по данным: ID + Length + Payload (без Start и End)
        if (packet.size() < 13) {
            // Слишком короткий пакет — удаляем и продолжаем поиск
            continue;
        }

        // Извлекаем ID, Length, Payload и CRC
        uint8_t id = static_cast<uint8_t>(packet[4]);
        uint16_t length = static_cast<unsigned char>(packet[5]) |
                          (static_cast<unsigned char>(packet[6]) << 8);

        // Проверяем, что длина соответствует размеру пакета
        if (packet.size() != 4 + 1 + 2 + length + 2 + 4) {
            // Неправильная длина — удаляем и продолжаем поиск
            continue;
        }

        // Данные для CRC: ID + Length + Payload (байты 4..4+1+2+length-1)
        QByteArray dataForCrc = packet.mid(4, 1 + 2 + length);
        uint16_t calculatedCrc = calculateCrc16(dataForCrc);

        // Извлекаем полученный CRC
        int crcPos = 4 + 1 + 2 + length;
        uint16_t receivedCrc = static_cast<unsigned char>(packet[crcPos]) |
                               (static_cast<unsigned char>(packet[crcPos + 1]) << 8);

        // --- 6. Если CRC совпало — обрабатываем пакет ---
        if (receivedCrc == calculatedCrc) {
            processDroneResponse(packet);
            // Выходим, так как пакет обработан успешно


            break;
        } else {
            // CRC не совпало — удаляем пакет (уже удалён) и продолжаем поиск
            continue;
        }
    }
}






// ============================================================
// СРАВНЕНИЕ ДАННЫХ (ЭТАЛОН, SD-КАРТА, РАДИОЛИНИЯ)
// ============================================================
// Вызывается при нажатии кнопки "Порівняти дані".
// Выполняет двухэтапное сравнение:
//   1. SD-карта vs Радиолиния (проверка работы логгера)
//   2. Эталон vs SD-карта (проверка работы имитатора)
// Результат выводится в textEdit.
// ============================================================
void MainWindow::onCompareDataClicked()
{
    // Очищаем предыдущие сообщения о сравнении
    ui->textEdit->append("\n========== ПОРІВНЯННЯ ДАНИХ ==========\n");

    // ============================================================
    // ПРОВЕРКА: есть ли эталонные данные
    // ============================================================
    if (m_etalonRecords.isEmpty()) {
        ui->textEdit->append("Помилка: немає еталонних даних (розвідку не проведено)");
        return;
    }

    // ============================================================
    // ШАГ 1: Сравниваем SD-карту и радиолинию
    // ============================================================
    ui->textEdit->append("--- 1. ПОРІВНЯННЯ SD-КАРТИ ТА РАДІОЛІНІЇ ---");

    // Проверяем наличие данных для сравнения
    if (m_sdRecords.isEmpty() && m_radiolineRecords.isEmpty()) {
        ui->textEdit->append("Немає даних для порівняння (SD і радіолінія порожні)");
    } else if (m_sdRecords.isEmpty()) {
        ui->textEdit->append("SD-карта: немає даних");
    } else if (m_radiolineRecords.isEmpty()) {
        ui->textEdit->append("Радіолінія: немає даних");
    } else {
        // Определяем минимальное количество записей для сравнения
        int count = qMin(m_sdRecords.size(), m_radiolineRecords.size());
        int errors = 0;  // Счётчик расхождений

        // Сравниваем записи по одной
        for (int i = 0; i < count; ++i) {
            const SdRecord &sd = m_sdRecords[i];
            const SdRecord &radio = m_radiolineRecords[i];

            // Сравниваем все поля (кроме номера строки)
            // Используем допустимые погрешности:
            //   - температура: 0.01°C
            //   - ПЕД: 0.001 мкЗв/ч
            //   - координаты: 0.000001 градуса
            //   - lidarDistance и lidarStrength: точное сравнение (целые числа)
            bool match = true;

            if (qAbs(sd.temp0 - radio.temp0) > 0.01) match = false;
            if (qAbs(sd.temp1 - radio.temp1) > 0.01) match = false;
            if (qAbs(sd.ped0 - radio.ped0) > 0.001) match = false;
            if (qAbs(sd.ped1 - radio.ped1) > 0.001) match = false;
            if (qAbs(sd.longitude - radio.longitude) > 0.000001) match = false;
            if (qAbs(sd.latitude - radio.latitude) > 0.000001) match = false;
            if (sd.lidarDistance != radio.lidarDistance) match = false;
            if (sd.lidarStrength != radio.lidarStrength) match = false;

            if (!match) {
                errors++;
                ui->textEdit->append(QString("  Розбіжність у рядку %1 (SD vs Радіо)")
                                         .arg(i + 1));
            }
        }

        // Проверяем количество записей
        if (m_sdRecords.size() != m_radiolineRecords.size()) {
            ui->textEdit->append(QString("  Кількість записів: SD=%1, Радіо=%2 (не співпадає)")
                                     .arg(m_sdRecords.size())
                                     .arg(m_radiolineRecords.size()));
        }

        // Выводим итоговый результат
        if (errors == 0 && m_sdRecords.size() == m_radiolineRecords.size()) {
            ui->textEdit->append("РЕЗУЛЬТАТ: SD ↔ Радіолінія: OK (всі дані співпадають)");
        } else {
            ui->textEdit->append(QString("РЕЗУЛЬТАТ: SD ↔ Радіолінія: %1 розбіжностей").arg(errors));
        }
    }

    // ============================================================
    // ШАГ 2: Сравниваем эталон с SD-картой
    // ============================================================
    ui->textEdit->append("\n--- 2. ПОРІВНЯННЯ ЕТАЛОНУ ТА SD-КАРТИ ---");

    // Проверяем наличие данных
    if (m_etalonRecords.isEmpty() || m_sdRecords.isEmpty()) {
        ui->textEdit->append("Немає даних для порівняння (еталон або SD порожні)");
    } else {
        int count = qMin(m_etalonRecords.size(), m_sdRecords.size());
        int errors = 0;

        for (int i = 0; i < count; ++i) {
            const SdRecord &etalon = m_etalonRecords[i];
            const SdRecord &sd = m_sdRecords[i];

            // Сравниваем поля (без температуры, так как она фиктивная)
            bool match = true;

            if (qAbs(etalon.ped0 - sd.ped0) > 0.001) match = false;
            if (qAbs(etalon.ped1 - sd.ped1) > 0.001) match = false;
            if (qAbs(etalon.longitude - sd.longitude) > 0.000001) match = false;
            if (qAbs(etalon.latitude - sd.latitude) > 0.000001) match = false;
            if (etalon.lidarDistance != sd.lidarDistance) match = false;
            if (etalon.lidarStrength != sd.lidarStrength) match = false;

            if (!match) {
                errors++;
                ui->textEdit->append(QString("  Розбіжність у рядку %1 (Еталон vs SD)")
                                         .arg(i + 1));
            }
        }

        if (m_etalonRecords.size() != m_sdRecords.size()) {
            ui->textEdit->append(QString("  Кількість записів: Еталон=%1, SD=%2 (не співпадає)")
                                     .arg(m_etalonRecords.size())
                                     .arg(m_sdRecords.size()));
        }

        if (errors == 0 && m_etalonRecords.size() == m_sdRecords.size()) {
            ui->textEdit->append("РЕЗУЛЬТАТ: Еталон ↔ SD: OK (всі дані співпадають)");
        } else {
            ui->textEdit->append(QString("РЕЗУЛЬТАТ: Еталон ↔ SD: %1 розбіжностей").arg(errors));
        }
    }

    ui->textEdit->append("\n========== КІНЕЦЬ ПОРІВНЯННЯ ==========\n");
}


// ============================================================
// ФОРМИРОВАНИЕ ЭТАЛОННОЙ ЗАПИСИ ДЛЯ ТЕКУЩЕЙ ТОЧКИ
// ============================================================
// Создаёт запись в формате SdRecord на основе:
//   - текущей точки из SurveyArea (координаты, высота, ПЕД)
//   - текущего режима LIDAR (intensity)
//   - фиктивных температур БДБГ (90°C)
// Добавляет запись в список m_etalonRecords.
// ============================================================
void MainWindow::addEtalonRecord()
{
    // Получаем текущую точку из SurveyArea
    PointData currentPoint = m_surveyArea.getCurrentPoint();

    // Создаём новую запись
    SdRecord record;

    // Номер строки = текущий размер списка + 1 (сквозная нумерация)
    record.lineNumber = m_etalonRecords.size() + 1;

    // --- Температуры БДБГ (фиктивные, пока 90°C) ---
    // В реальном протоколе БДБГ при запросе температуры возвращает 90°C
    // (значение 0x5A01 в протоколе)
    record.temp0 = 90.0;   // нижний дозиметр
    record.temp1 = 90.0;   // верхний дозиметр

    // --- Данные из текущей точки ---
    record.ped0 = currentPoint.getPedLower();       // ПЕД нижнего дозиметра
    record.ped1 = currentPoint.getPedUpper();       // ПЕД верхнего дозиметра
    record.longitude = currentPoint.getLongitude(); // долгота
    record.latitude = currentPoint.getLatitude();   // широта

    // --- LIDAR расстояние (переводим метры в сантиметры) ---
    // В протоколе LIDAR расстояние передаётся в сантиметрах
    record.lidarDistance = static_cast<int>(currentPoint.getHeight() * 100.0);

    // --- LIDAR интенсивность (зависит от режима имитации) ---
    // Берём значение из comboBox_imit_mode_lidar
    QString mode = ui->comboBox_imit_mode_lidar->currentText();

    if (mode == "Нормальне вимірювання") {
        record.lidarStrength = 0x0FFF;      // нормальная интенсивность
    } else if (mode == "Об'єкт не виявлено") {
        record.lidarStrength = 0x00D2;      // слабое отражение (< 255)
    } else if (mode == "Слабке відбиття") {
        record.lidarStrength = 0x00D2;      // слабое отражение (0xD2)
    } else if (mode == "Сильне відбиття") {
        record.lidarStrength = 0xBFFF;      // сильное отражение (> 0xAFFF)
    } else if (mode == "Об'єкт дуже близько") {
        record.lidarStrength = 0x0FFF;      // нормальная интенсивность
    } else if (mode == "Об'єкт занадто далеко") {
        record.lidarStrength = 0x0FFF;      // нормальная интенсивность
    } else {
        record.lidarStrength = 0x0FFF;      // по умолчанию
    }

    // Добавляем запись в список эталонных данных
    m_etalonRecords.append(record);
}




// ============================================================
// ГЕНЕРАЦИЯ ТОЧЕК ПОЛИГОНА (12 ТОЧЕК)
// ============================================================
// Создаёт 12 точек, описывающих границу квадрата:
//   - 4 вершины квадрата
//   - по 2 точки на каждом ребре (равномерно распределены)
//
// Вход: startLat, startLon - левый верхний угол квадрата (градусы)
//       sideKm - ребро квадрата (км)
// Выход: заполняет m_polygonPoints
// ============================================================












// ============================================================
// ПРОВЕРКА ВХОЖДЕНИЯ ТОЧКИ В ВЫПУКЛЫЙ ПОЛИГОН
// ============================================================
// Использует алгоритм проверки знаков векторных произведений.
// Для выпуклого полигона: точка внутри, если все векторные
// произведения (ребро → точка) имеют один знак.
//
// Вход: point - проверяемая точка
//       polygon - список точек полигона (в порядке обхода)
// Выход: true - точка внутри полигона, false - вне
// ============================================================
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
        return true;   // На границе = внутри
    }

    for (int i = 1; i < polygon.size(); ++i) {
        int next = (i + 1) % polygon.size();
        double sign = crossProduct(polygon[i], polygon[next], point);
        if (qAbs(sign) < 1e-10) {
            return true;   // На границе = внутри
        }
        if (sign * firstSign < 0) {
            return false;
        }
    }

    return true;
}









// ============================================================
// ГЕНЕРАЦИЯ ТОЧЕК ПУТИ (ЛИНЕЙНАЯ ИНТЕРПОЛЯЦИЯ)
// ============================================================
// Создаёт count точек, равномерно распределённых между
// начальной и конечной точками (включая обе конечные).
//
// Вход: startLat, startLon - начальная точка (градусы)
//       endLat, endLon - конечная точка (градусы)
//       count - количество точек (должно быть >= 2)
//       height - высота для всех точек пути (м)
//       bgPed - фоновое значение ПЕД (мкЗв/ч)
//       targetList - список, в который добавляются точки
// ============================================================
void MainWindow::generatePathPoints(double startLat, double startLon,
                                    double endLat, double endLon,
                                    int count, double height, double bgPed,
                                    QList<PointData> &targetList)
{
    if (count < 2) {
        // Если count < 2, добавляем только стартовую точку
        targetList.append(PointData(startLat, startLon, height, bgPed, bgPed));
        return;
    }

    for (int i = 0; i < count; ++i) {
        double t = static_cast<double>(i) / (count - 1);  // от 0 до 1

        double lat = startLat + (endLat - startLat) * t;
        double lon = startLon + (endLon - startLon) * t;

        // Точки пути имеют фоновое ПЕД (нет заражения)
        targetList.append(PointData(lat, lon, height, bgPed, bgPed));
    }
}



// ============================================================
// ГЕНЕРАЦИЯ ТОЧЕК ПОЛИГОНА (12 ТОЧЕК)
// ============================================================
// Создаёт 12 точек, описывающих границу квадрата:
//   - 4 вершины квадрата
//   - по 2 точки на каждом ребре (равномерно распределены)
//
// Вход: startLat, startLon - левый верхний угол квадрата (градусы)
//       sideKm - ребро квадрата (км)
// Выход: заполняет m_polygonPoints
// ============================================================
void MainWindow::generatePolygonPoints(double startLat, double startLon, double sideKm)
{
    m_polygonPoints.clear();

    // Рассчитываем размер ребра в градусах
    double sideLat = m_surveyArea.kmToLatitudeDegrees(sideKm);
    double sideLon = m_surveyArea.kmToLongitudeDegrees(sideKm, startLat);

    // Увеличиваем полигон на 0.005 градуса (примерно 500 м) для гарантии
    double eps = 0.04;

    // Вершины увеличенного квадрата
    double latMax = startLat + eps;
    double latMin = startLat - sideLat - eps;
    double lonMin = startLon - eps;
    double lonMax = startLon + sideLon + eps;

    // 4 вершины
    QList<PointData> vertices;
    vertices.append(PointData(latMax, lonMin, 0, 0, 0)); // левый верхний
    vertices.append(PointData(latMax, lonMax, 0, 0, 0)); // правый верхний
    vertices.append(PointData(latMin, lonMax, 0, 0, 0)); // правый нижний
    vertices.append(PointData(latMin, lonMin, 0, 0, 0)); // левый нижний

    // Добавляем вершины
    for (const auto &v : vertices) {
        m_polygonPoints.append(v);
    }

    // Добавляем по 2 точки на каждом ребре
    for (int i = 0; i < 4; ++i) {
        PointData p1 = vertices[i];
        PointData p2 = vertices[(i + 1) % 4];
        for (int j = 1; j <= 2; ++j) {
            double t = j / 3.0;
            double lat = p1.getLatitude() + (p2.getLatitude() - p1.getLatitude()) * t;
            double lon = p1.getLongitude() + (p2.getLongitude() - p1.getLongitude()) * t;
            m_polygonPoints.append(PointData(lat, lon, 0, 0, 0));
        }
    }
}

// ============================================================
// РАСЧЁТ CRC16 (CCITT) ДЛЯ ПАКЕТОВ
// ============================================================
// Алгоритм: CRC-16/CCITT-FALSE
// Полином: 0x1021
// Начальное значение: 0xFFFF
// Без финального XOR
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
// ФОРМИРОВАНИЕ ПАКЕТА ПО ПРОТОКОЛУ
// ============================================================
// Структура пакета:
//   Start bytes: 0xAA 0x55 0xAA 0x55 (4 байта)
//   Command ID: 1 байт
//   Length: 2 байта (uint16_t, Little-Endian)
//   Payload: N байт
//   CRC16: 2 байта (uint16_t, Little-Endian)
//   End bytes: 0x0D 0x0A 0x0D 0x0A (4 байта)
// ============================================================
QByteArray MainWindow::buildCommandPacket(uint8_t commandId, const QByteArray &payload)
{
    QByteArray packet;

    // --- Start bytes ---
    packet.append(static_cast<char>(0xAA));
    packet.append(static_cast<char>(0x55));
    packet.append(static_cast<char>(0xAA));
    packet.append(static_cast<char>(0x55));

    // --- Command ID ---
    packet.append(static_cast<char>(commandId));

    // --- Length (2 байта, Little-Endian) ---
    uint16_t length = static_cast<uint16_t>(payload.size());
    packet.append(static_cast<char>(length & 0xFF));        // Low byte
    packet.append(static_cast<char>((length >> 8) & 0xFF)); // High byte

    // --- Payload ---
    packet.append(payload);

    // --- CRC16 (расчёт по данным: ID + Length + Payload) ---
    QByteArray dataForCrc = packet.mid(4); // Отбрасываем Start bytes (первые 4 байта)
    uint16_t crc = calculateCrc16(dataForCrc);
    packet.append(static_cast<char>(crc & 0xFF));        // Low byte
    packet.append(static_cast<char>((crc >> 8) & 0xFF)); // High byte

    // --- End bytes ---
    packet.append(static_cast<char>(0x0D));
    packet.append(static_cast<char>(0x0A));
    packet.append(static_cast<char>(0x0D));
    packet.append(static_cast<char>(0x0A));

    return packet;
}

// ============================================================
// ОТПРАВКА КОМАНДЫ НА ДРОН
// ============================================================
// Формирует пакет, дублирует его repeat_factor раз и отправляет
// через порт logger_rx_port.
// ============================================================
void MainWindow::sendCommandToDrone(uint8_t commandId, const QByteArray &payload)
{
    if (!logger_rx_port || !logger_rx_port->isOpen()) {
        ui->textEdit_radioline->append("Помилка: порт радіолінії не відкрито");
        return;
    }

    // Формируем один пакет
    QByteArray singlePacket = buildCommandPacket(commandId, payload);

    // Получаем коэффициент дублирования
    int repeatFactor = ui->spinBox_packet_dupl->value();
    if (repeatFactor < 1) repeatFactor = 1;
    if (repeatFactor > 10) repeatFactor = 10;

    // Формируем общий массив для передачи (повторяем пакет repeatFactor раз)
    QByteArray sendData;
    for (int i = 0; i < repeatFactor; ++i) {
        sendData.append(singlePacket);
    }

    // Отправляем одной операцией
    qint64 bytesWritten = logger_rx_port->write(sendData);
    if (bytesWritten != sendData.size()) {
        ui->textEdit_radioline->append("Помилка: не вдалося відправити пакет");
        return;
    }
    logger_rx_port->waitForBytesWritten(100);

    // Вывод информации
    ui->textEdit_radioline->append(QString("Відправлено команду: %1 (ID %2), дублів: %3, розмір: %4 байт")
                                       .arg(getCommandName(commandId))
                                       .arg(commandId)
                                       .arg(repeatFactor)
                                       .arg(sendData.size()));
}

// ============================================================
// ПОЛУЧЕНИЕ ТЕКСТА КОМАНДЫ ПО ID (ЗЕМЛЯ → ДРОН)
// ============================================================
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
    case 12: return "Перейти в режим набора спектра";
    case 13: return "Прекратить набор спектра";
    case 14: return "Перейти в режим радиомолчания";
    case 15: return "Выйти из режима радиомолчания";
    default: return QString("Неизвестная команда (ID %1)").arg(id);
    }
}



// ============================================================
// ПОЛУЧЕНИЕ ТЕКСТА ОТВЕТА ПО ID (ДРОН → ЗЕМЛЯ)
// ============================================================
QString MainWindow::getResponseText(uint8_t id)
{
    switch (id) {
    case 1:  return "Пакет принят";
    case 2:  return "Пакет не принят";
    case 3:  return "Команда не выполняется (режим теста)";
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
    default: return QString("Неизвестный ответ (ID %1)").arg(id);
    }
}

// ============================================================
// ОБРАБОТЧИК КНОПКИ "ПЕРЕДАЧА КОМАНДИ НА ДРОН"
// ============================================================
// Считывает выбранную команду из comboBox_select_Command,
// формирует Payload в зависимости от ID,
// отправляет команду через sendCommandToDrone().
// ============================================================
void MainWindow::on_pushButton_send_comm_clicked()
{
    // Получаем ID команды (индекс в comboBox + 1, так как ID начинаются с 1)
    int commandIndex = ui->comboBox_select_Command->currentIndex();
    uint8_t commandId = static_cast<uint8_t>(commandIndex + 1);

    QByteArray payload;

    // --- Формируем Payload в зависимости от ID ---
    switch (commandId) {
    case 1:  // Провести предполётное тестирование
    case 2:  // Продолжить разведку без SD-карты
    case 3:  // Не продолжать разведку без SD-карты
    case 8:  // Закончить разведку
    case 10: // Произвести измерение (ручной режим)
    case 12: // Перейти в режим набора спектра
    case 13: // Прекратить набор спектра
    case 14: // Перейти в режим радиомолчания
    case 15: // Выйти из режима радиомолчания
        // Команды без данных: Length = 0x0001, Payload = 0x01
        payload.append(static_cast<char>(0x01));
        break;

    case 4: { // Передать параметры разведки (FlightConfig)
        // pointCount (всегда 12)
        payload.append(static_cast<char>(12));

        // Массив координат (12 точек из m_polygonPoints)
        for (const PointData &p : m_polygonPoints) {
            // float lat (Little-Endian)
            float lat = static_cast<float>(p.getLatitude());
            const char* latBytes = reinterpret_cast<const char*>(&lat);
            payload.append(latBytes, sizeof(float));

            // float lon (Little-Endian)
            float lon = static_cast<float>(p.getLongitude());
            const char* lonBytes = reinterpret_cast<const char*>(&lon);
            payload.append(lonBytes, sizeof(float));
        }

        // mode (индекс из comboBox_rec_mode)
        uint8_t mode = static_cast<uint8_t>(ui->comboBox_rec_mode->currentIndex());
        payload.append(static_cast<char>(mode));

        // saveTime (uint16_t, Little-Endian)
        uint16_t saveTime = static_cast<uint16_t>(ui->spinBox_meas_time->value());
        payload.append(static_cast<char>(saveTime & 0xFF));
        payload.append(static_cast<char>((saveTime >> 8) & 0xFF));

        // saveDistance (uint16_t, Little-Endian)
        uint16_t saveDistance = static_cast<uint16_t>(ui->spinBox_meas_dist->value());
        payload.append(static_cast<char>(saveDistance & 0xFF));
        payload.append(static_cast<char>((saveDistance >> 8) & 0xFF));

        // duplCoeff (uint8_t)
        uint8_t duplCoeff = static_cast<uint8_t>(ui->spinBox_packet_dupl->value());
        payload.append(static_cast<char>(duplCoeff));

        // saveSpectre (uint16_t, Little-Endian)
        uint16_t saveSpectre = static_cast<uint16_t>(ui->spinBox_spectre_time->value());
        payload.append(static_cast<char>(saveSpectre & 0xFF));
        payload.append(static_cast<char>((saveSpectre >> 8) & 0xFF));
        break;
    }

    case 5: { // Изменить коэффициент дублирования (uint8_t)
        uint8_t value = static_cast<uint8_t>(ui->spinBox_packet_dupl->value());
        payload.append(static_cast<char>(value));
        break;
    }

    case 6: { // Изменить время между измерениями (uint16_t)
        uint16_t value = static_cast<uint16_t>(ui->spinBox_meas_time->value());
        payload.append(static_cast<char>(value & 0xFF));
        payload.append(static_cast<char>((value >> 8) & 0xFF));
        break;
    }

    case 7: { // Изменить дистанцию между измерениями (uint16_t)
        uint16_t value = static_cast<uint16_t>(ui->spinBox_meas_dist->value());
        payload.append(static_cast<char>(value & 0xFF));
        payload.append(static_cast<char>((value >> 8) & 0xFF));
        break;
    }

    case 9: { // Изменить режим разведки (uint8_t)
        uint8_t mode = static_cast<uint8_t>(ui->comboBox_rec_mode->currentIndex());
        payload.append(static_cast<char>(mode));
        break;
    }

    case 11: { // Изменить время набора спектра (uint16_t)
        uint16_t value = static_cast<uint16_t>(ui->spinBox_spectre_time->value());
        payload.append(static_cast<char>(value & 0xFF));
        payload.append(static_cast<char>((value >> 8) & 0xFF));
        break;
    }

    default:
        ui->textEdit_radioline->append(QString("Невідома команда ID %1").arg(commandId));
        return;
    }

    // --- Отправляем команду ---
    sendCommandToDrone(commandId, payload);
}

// ============================================================
// ОБРАБОТКА ПРИНЯТОГО ОТВЕТА ОТ ДРОНА
// ============================================================
// Парсит пакет, извлекает ID, Length, Payload,
// проверяет CRC, выводит читабельное сообщение.
// ============================================================
void MainWindow::processDroneResponse(const QByteArray &packet)
{
    // Минимальный размер пакета: Start(4) + ID(1) + Length(2) + CRC(2) + End(4) = 13 байт
    if (packet.size() < 13) {
        ui->textEdit_radioline->append("Помилка: пакет занадто короткий");
        return;
    }

    // --- Проверяем Start bytes ---
    if (packet[0] != 0xAA || packet[1] != 0x55 || packet[2] != 0xAA || packet[3] != 0x55) {
        ui->textEdit_radioline->append("Помилка: невірні стартові байти");
        return;
    }

    // --- Извлекаем ID ---
    uint8_t id = static_cast<uint8_t>(packet[4]);

    // --- Извлекаем Length (Little-Endian) ---
    uint16_t length = static_cast<uint8_t>(packet[5]) |
                      (static_cast<uint8_t>(packet[6]) << 8);

    // --- Проверяем размер ---
    if (packet.size() < 7 + length + 6) { // 7 = Start(4) + ID(1) + Length(2)
        ui->textEdit_radioline->append("Помилка: пакет занадто короткий для Length");
        return;
    }

    // --- Извлекаем Payload ---
    QByteArray payload = packet.mid(7, length);

    // --- Извлекаем CRC (Little-Endian) ---
    int crcPos = 7 + length;
    uint16_t receivedCrc = static_cast<uint8_t>(packet[crcPos]) |
                           (static_cast<uint8_t>(packet[crcPos + 1]) << 8);

    // --- Проверяем End bytes ---
    int endPos = crcPos + 2;
    if (packet[endPos] != 0x0D || packet[endPos + 1] != 0x0A ||
        packet[endPos + 2] != 0x0D || packet[endPos + 3] != 0x0A) {
        ui->textEdit_radioline->append("Помилка: невірні кінцеві байти");
        return;
    }

    // --- Проверяем CRC ---
    QByteArray dataForCrc = packet.mid(4, 3 + length); // ID + Length + Payload
    uint16_t calculatedCrc = calculateCrc16(dataForCrc);

    if (receivedCrc != calculatedCrc) {
        ui->textEdit_radioline->append("Неспівпадіння CRC");
        return;
    }

    // --- Выводим читабельное сообщение ---
    QString responseText = getResponseText(id);

    // --- Если есть Payload — форматируем его ---
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

// ============================================================
// ФОРМАТИРОВАНИЕ PAYLOAD ДЛЯ ВЫВОДА В ЧИТАБЕЛЬНОМ ВИДЕ
// ============================================================
// В зависимости от ID ответа парсит Payload и возвращает строку
// с расшифрованными данными.
// ============================================================
QString MainWindow::formatPayload(const QByteArray &payload, uint8_t id)
{
    if (payload.isEmpty()) {
        return QString();
    }

    QString result;
    int offset = 0;

    switch (id) {
    case 4: { // Аккумулятор разряжен (float напряжение)
        if (payload.size() >= sizeof(float)) {
            float voltage;
            memcpy(&voltage, payload.data(), sizeof(float));
            result = QString("Напруга: %1 V").arg(voltage, 0, 'f', 2);
        }
        break;
    }

    case 5: { // Аккумулятор заряжен (float напряжение)
        if (payload.size() >= sizeof(float)) {
            float voltage;
            memcpy(&voltage, payload.data(), sizeof(float));
            result = QString("Напруга: %1 V").arg(voltage, 0, 'f', 2);
        }
        break;
    }

    case 19: { // Координата, время, вне зоны (float lat, float lon, uint32_t time)
        if (payload.size() >= 12) {
            float lat, lon;
            uint32_t time;
            memcpy(&lat, payload.data(), sizeof(float));
            memcpy(&lon, payload.data() + 4, sizeof(float));
            memcpy(&time, payload.data() + 8, sizeof(uint32_t));
            result = QString("lat=%1, lon=%2, time=%3")
                         .arg(lat, 0, 'f', 6)
                         .arg(lon, 0, 'f', 6)
                         .arg(time);
        }
        break;
    }

    case 20: { // Координата, время, в области (float lat, float lon, uint32_t time)
        if (payload.size() >= 12) {
            float lat, lon;
            uint32_t time;
            memcpy(&lat, payload.data(), sizeof(float));
            memcpy(&lon, payload.data() + 4, sizeof(float));
            memcpy(&time, payload.data() + 8, sizeof(uint32_t));
            result = QString("lat=%1, lon=%2, time=%3")
                         .arg(lat, 0, 'f', 6)
                         .arg(lon, 0, 'f', 6)
                         .arg(time);
        }
        break;
    }

    case 22: { // Данные (Пакет №1)
        // Формат пакета №1:
        // - ПАЕД (float 4)
        // - температура (float 4)
        // - статус БДБГ (1 байт)
        // - высота LIDAR (uint16_t 2)
        // - мощность LIDAR (uint16_t 2)
        // - широта (float 4)
        // - долгота (float 4)
        // - высота GPS (float 4)
        // - кол-во спутников (uint8_t 1)
        // - время (uint32_t 4)
        // Итого: 4+4+1+2+2+4+4+4+1+4 = 30 байт
        if (payload.size() >= 30) {
            float ped, temp, lat, lon, gpsAlt;
            uint16_t lidarDist, lidarStrength;
            uint8_t status, satellites;
            uint32_t time;

            offset = 0;
            memcpy(&ped, payload.data() + offset, sizeof(float)); offset += 4;
            memcpy(&temp, payload.data() + offset, sizeof(float)); offset += 4;
            status = static_cast<uint8_t>(payload[offset]); offset += 1;
            memcpy(&lidarDist, payload.data() + offset, sizeof(uint16_t)); offset += 2;
            memcpy(&lidarStrength, payload.data() + offset, sizeof(uint16_t)); offset += 2;
            memcpy(&lat, payload.data() + offset, sizeof(float)); offset += 4;
            memcpy(&lon, payload.data() + offset, sizeof(float)); offset += 4;
            memcpy(&gpsAlt, payload.data() + offset, sizeof(float)); offset += 4;
            satellites = static_cast<uint8_t>(payload[offset]); offset += 1;
            memcpy(&time, payload.data() + offset, sizeof(uint32_t));

            result = QString("ПЕД=%1 мкЗв/ч, t=%2°C, статус=0x%3, LIDAR=%4 см, сила=%5, "
                             "lat=%6, lon=%7, висота GPS=%8 м, супутників=%9, час=%10")
                         .arg(ped, 0, 'f', 3)
                         .arg(temp, 0, 'f', 2)
                         .arg(status, 2, 16, QChar('0'))
                         .arg(lidarDist)
                         .arg(lidarStrength)
                         .arg(lat, 0, 'f', 6)
                         .arg(lon, 0, 'f', 6)
                         .arg(gpsAlt, 0, 'f', 1)
                         .arg(satellites)
                         .arg(time);
        }
        break;
    }

    case 23: { // Спектр (Пакет №2)
        // Формат: 1024 uint16 (2048 байт) + время набора (uint16_t 2)
        if (payload.size() >= 2050) {
            // Извлекаем время набора (последние 2 байта)
            uint16_t spectreTime;
            memcpy(&spectreTime, payload.data() + payload.size() - 2, sizeof(uint16_t));

            // Первые 2048 байт — спектр (1024 канала)
            // Для вывода показываем только первые и последние несколько значений
            uint16_t firstChannel, lastChannel;
            memcpy(&firstChannel, payload.data(), sizeof(uint16_t));
            memcpy(&lastChannel, payload.data() + (1024 - 1) * 2, sizeof(uint16_t));

            result = QString("Спектр: 1024 канали, час набору=%1 с, "
                             "канал0=%2, канал1023=%3")
                         .arg(spectreTime)
                         .arg(firstChannel)
                         .arg(lastChannel);
        }
        break;
    }

    case 25: { // Дальнейшее проведение разведки невозможно, прибор ...
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

    default:
        // Для остальных ID выводим Payload в HEX
        result = payload.toHex().toUpper();
        break;
    }

    return result;
}

