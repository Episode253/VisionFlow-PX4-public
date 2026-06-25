#include "../include/qt_joystick/sim_remote_window.hpp"

#include <array>

#include <QApplication>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSlider>
#include <QStatusBar>
#include <QTimer>
#include <QVBoxLayout>

namespace qt_joystick {

namespace {

QString badgeText(bool connected, bool armed) {
    if (!connected) {
        return QStringLiteral("ROS OFFLINE");
    }

    return armed ? QStringLiteral("VEHICLE ARMED") : QStringLiteral("VEHICLE SAFE");
}

QString baseStyleSheet() {
    return QString::fromUtf8(
        "QMainWindow { background: #08111a; }"
        "#centralRoot { background: qlineargradient(x1:0,y1:0,x2:1,y2:1, stop:0 #08111a, stop:0.5 #102131, stop:1 #161b2f); }"
        "QGroupBox { color: #e9eefb; border: 1px solid rgba(255,255,255,42); border-radius: 20px; margin-top: 16px; font: 800 12pt 'DejaVu Sans'; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 18px; padding: 0 10px; color: #91d5ff; }"
        "QFrame[card='panel'] { background: rgba(255,255,255,18); border: 1px solid rgba(255,255,255,26); border-radius: 18px; }"
        "QLabel[role='title'] { color: #f7fbff; font: 900 15pt 'DejaVu Sans'; }"
        "QLabel[role='subtitle'] { color: #9eb8cf; font: 600 9pt 'DejaVu Sans'; }"
        "QLabel[role='value'] { color: #8ee1ff; font: 900 12pt 'DejaVu Sans'; }"
        "QLabel[role='badge'] { color: #08111a; background: #8ee1ff; border-radius: 16px; padding: 8px 14px; font: 900 10pt 'DejaVu Sans'; }"
        "QComboBox, QDoubleSpinBox { min-height: 34px; border-radius: 12px; border: 1px solid rgba(142,225,255,90); background: rgba(3,9,16,210); color: #eef8ff; padding: 4px 12px; font: 700 10pt 'DejaVu Sans'; }"
        "QSlider::groove:horizontal { height: 6px; border-radius: 3px; background: rgba(255,255,255,40); }"
        "QSlider::handle:horizontal { width: 18px; margin: -7px 0; border-radius: 9px; background: #8ee1ff; }"
        "QPushButton { min-height: 42px; border-radius: 16px; border: 0; padding: 8px 18px; font: 900 11pt 'DejaVu Sans'; }"
        "QPushButton[role='primary'] { color: #08111a; background: #8ee1ff; }"
        "QPushButton[role='primary']:hover { background: #a4ebff; }"
        "QPushButton[role='warn'] { color: #fff4eb; background: #ff7c5c; }"
        "QPushButton[role='warn']:hover { background: #ff977d; }"
        "QPushButton[role='ghost'] { color: #d7e8ff; background: rgba(255,255,255,20); border: 1px solid rgba(255,255,255,40); }"
        "QPushButton[role='ghost']:hover { border: 1px solid rgba(142,225,255,140); }"
        "QStatusBar { color: #98acc1; background: #08111a; }");
}

}  // namespace

SimRemoteWindow::SimRemoteWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent)
    , node_(argc, argv) {
    buildUi();

    QObject::connect(&node_, SIGNAL(rosShutdown()), this, SLOT(close()));

    state_timer_ = new QTimer(this);
    state_timer_->setInterval(250);
    connect(state_timer_, &QTimer::timeout, this, &SimRemoteWindow::refreshVehicleState);
    state_timer_->start();

    syncUiToCommandState();
    refreshVehicleState();
}

SimRemoteWindow::~SimRemoteWindow() = default;

void SimRemoteWindow::buildUi() {
    setWindowTitle(QStringLiteral("Sim Flight + Arm Remote"));
    setMinimumSize(1280, 860);
    setStyleSheet(baseStyleSheet());

    QWidget *central = new QWidget(this);
    central->setObjectName(QStringLiteral("centralRoot"));
    setCentralWidget(central);

    QVBoxLayout *root = new QVBoxLayout(central);
    root->setContentsMargins(24, 20, 24, 20);
    root->setSpacing(18);

    QWidget *top_bar = buildConnectionBar();
    root->addWidget(top_bar);

    QHBoxLayout *body = new QHBoxLayout();
    body->setSpacing(18);
    root->addLayout(body, 1);

    QWidget *flight_panel = buildFlightPanel();
    QWidget *mission_panel = buildMissionPanel();
    QWidget *arm_panel = buildArmPanel();

    body->addWidget(flight_panel, 2);
    body->addWidget(mission_panel, 2);
    body->addWidget(arm_panel, 3);

    statusBar()->showMessage(QStringLiteral("Ready to connect to ROS master."));
}

QWidget *SimRemoteWindow::buildConnectionBar() {
    QFrame *frame = new QFrame(this);
    frame->setProperty("card", "panel");

    QHBoxLayout *layout = new QHBoxLayout(frame);
    layout->setContentsMargins(22, 18, 22, 18);
    layout->setSpacing(16);

    QLabel *title = new QLabel(QStringLiteral("Simulation Mission Bridge"), frame);
    title->setProperty("role", "title");
    layout->addWidget(title);

    QLabel *subtitle = new QLabel(
        QStringLiteral("Virtual RC on /virtual_joy plus direct joint setpoints for the manipulator."),
        frame);
    subtitle->setProperty("role", "subtitle");
    layout->addWidget(subtitle, 1);

    mode_badge_ = createValueLabel(QStringLiteral("MODE --"));
    mode_badge_->setProperty("role", "badge");
    layout->addWidget(mode_badge_);

    status_badge_ = createValueLabel(QStringLiteral("ROS OFFLINE"));
    status_badge_->setProperty("role", "badge");
    layout->addWidget(status_badge_);

    connect_button_ = new QPushButton(QStringLiteral("Start ROS Link"), frame);
    connect_button_->setProperty("role", "primary");
    connect(connect_button_, &QPushButton::clicked, this, &SimRemoteWindow::onConnectClicked);
    layout->addWidget(connect_button_);

    QPushButton *quit_button = new QPushButton(QStringLiteral("Close Console"), frame);
    quit_button->setProperty("role", "ghost");
    connect(quit_button, &QPushButton::clicked, this, &QWidget::close);
    layout->addWidget(quit_button);

    return frame;
}

QWidget *SimRemoteWindow::buildFlightPanel() {
    QGroupBox *group = new QGroupBox(QStringLiteral("UAV Manual Axes"), this);
    QVBoxLayout *layout = new QVBoxLayout(group);
    layout->setContentsMargins(18, 24, 18, 18);
    layout->setSpacing(14);

    layout->addWidget(createRcSliderCard(QStringLiteral("Roll / RC1"),
                                         QStringLiteral("Horizontal attitude input"),
                                         -100, 100, 0, roll_widgets_));
    layout->addWidget(createRcSliderCard(QStringLiteral("Pitch / RC2"),
                                         QStringLiteral("Forward attitude input"),
                                         -100, 100, 0, pitch_widgets_));
    layout->addWidget(createRcSliderCard(QStringLiteral("Yaw / RC4"),
                                         QStringLiteral("Heading rate input"),
                                         -100, 100, 0, yaw_widgets_));
    layout->addWidget(createRcSliderCard(QStringLiteral("Throttle / RC3"),
                                         QStringLiteral("0% = low throttle, 100% = high throttle"),
                                         0, 100, 0, throttle_widgets_));

    QHBoxLayout *button_row = new QHBoxLayout();
    button_row->setSpacing(10);

    QPushButton *arm_button = new QPushButton(QStringLiteral("Arm Vehicle"), group);
    arm_button->setProperty("role", "primary");
    connect(arm_button, &QPushButton::clicked, this, &SimRemoteWindow::onArmClicked);
    button_row->addWidget(arm_button);

    QPushButton *disarm_button = new QPushButton(QStringLiteral("Disarm"), group);
    disarm_button->setProperty("role", "warn");
    connect(disarm_button, &QPushButton::clicked, this, &SimRemoteWindow::onDisarmClicked);
    button_row->addWidget(disarm_button);

    QPushButton *manual_button = new QPushButton(QStringLiteral("Set MANUAL"), group);
    manual_button->setProperty("role", "ghost");
    connect(manual_button, &QPushButton::clicked, this, &SimRemoteWindow::onModeManualClicked);
    button_row->addWidget(manual_button);

    QPushButton *offboard_button = new QPushButton(QStringLiteral("Set OFFBOARD"), group);
    offboard_button->setProperty("role", "ghost");
    connect(offboard_button, &QPushButton::clicked, this, &SimRemoteWindow::onModeOffboardClicked);
    button_row->addWidget(offboard_button);

    layout->addLayout(button_row);
    layout->addStretch(1);
    return group;
}

QWidget *SimRemoteWindow::buildMissionPanel() {
    QGroupBox *group = new QGroupBox(QStringLiteral("Mode And Switch Matrix"), this);
    QVBoxLayout *layout = new QVBoxLayout(group);
    layout->setContentsMargins(18, 24, 18, 18);
    layout->setSpacing(14);

    auto add_combo_row = [this, layout](const QString &title,
                                        const QString &hint,
                                        const QStringList &items,
                                        QComboBox **target_combo) {
        QFrame *card = new QFrame(this);
        card->setProperty("card", "panel");
        QVBoxLayout *card_layout = new QVBoxLayout(card);
        card_layout->setContentsMargins(16, 14, 16, 14);
        card_layout->setSpacing(8);

        QLabel *title_label = new QLabel(title, card);
        title_label->setProperty("role", "title");
        card_layout->addWidget(title_label);

        QLabel *hint_label = new QLabel(hint, card);
        hint_label->setProperty("role", "subtitle");
        card_layout->addWidget(hint_label);

        *target_combo = createComboBox(items);
        card_layout->addWidget(*target_combo);
        layout->addWidget(card);
    };

    add_combo_row(QStringLiteral("CH5 Primary Flight Mode"),
                  QStringLiteral("Published through /virtual_joy axis[4]"),
                  {QStringLiteral("Stabilized Hold"),
                   QStringLiteral("Position Lock"),
                   QStringLiteral("Offboard Engage")},
                  &flight_mode_combo_);

    add_combo_row(QStringLiteral("CH6 Auto Gate"),
                  QStringLiteral("Published through /virtual_joy axis[5]"),
                  {QStringLiteral("Manual Gate"),
                   QStringLiteral("Auto Gate")},
                  &auto_gate_combo_);

    add_combo_row(QStringLiteral("CH7 Mission Selector"),
                  QStringLiteral("Published through /virtual_joy axis[6]"),
                  {QStringLiteral("Route Track"),
                   QStringLiteral("Launch Hover"),
                   QStringLiteral("Land Sequence")},
                  &mission_combo_);

    add_combo_row(QStringLiteral("CH9 Cooperative Task"),
                  QStringLiteral("Published through /virtual_joy button[0]"),
                  {QStringLiteral("Task Isolated"),
                   QStringLiteral("Task Coordinated")},
                  &coordinate_combo_);

    add_combo_row(QStringLiteral("CH10 Delta Actuator"),
                  QStringLiteral("Published through /virtual_joy button[1]"),
                  {QStringLiteral("Delta Down"),
                   QStringLiteral("Delta Up")},
                  &delta_combo_);

    add_combo_row(QStringLiteral("CH11 Motor Latch"),
                  QStringLiteral("Published through /virtual_joy button[2]"),
                  {QStringLiteral("Latch Safe"),
                   QStringLiteral("Latch Armed")},
                  &motor_latch_combo_);

    const std::array<QComboBox *, 6> combos = {
        flight_mode_combo_, auto_gate_combo_, mission_combo_,
        coordinate_combo_, delta_combo_, motor_latch_combo_
    };

    for (QComboBox *combo : combos) {
        connect(combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &SimRemoteWindow::syncUiToCommandState);
    }

    layout->addStretch(1);
    return group;
}

QWidget *SimRemoteWindow::buildArmPanel() {
    QGroupBox *group = new QGroupBox(QStringLiteral("Manipulator Joint Console"), this);
    QVBoxLayout *layout = new QVBoxLayout(group);
    layout->setContentsMargins(18, 24, 18, 18);
    layout->setSpacing(12);

    for (int i = 0; i < 6; ++i) {
        QString title = QStringLiteral("Joint %1").arg(i + 1);
        layout->addWidget(createJointControlRow(title, i, -3.14, 3.14, 0.0));
    }

    QHBoxLayout *preset_row = new QHBoxLayout();
    preset_row->setSpacing(10);

    QPushButton *zero_button = new QPushButton(QStringLiteral("All Zero"), group);
    zero_button->setProperty("role", "ghost");
    connect(zero_button, &QPushButton::clicked, this, [this]() {
        for (QDoubleSpinBox *spinbox : joint_spinboxes_) {
            if (spinbox) {
                spinbox->setValue(0.0);
            }
        }
        syncUiToCommandState();
    });
    preset_row->addWidget(zero_button);

    QPushButton *flight_button = new QPushButton(QStringLiteral("Flight Pose"), group);
    flight_button->setProperty("role", "ghost");
    connect(flight_button, &QPushButton::clicked, this, [this]() {
        const std::array<double, 6> flight_pose = {0.0, 1.0472, 2.0944, 0.0, -1.0472, -1.5708};
        for (size_t i = 0; i < joint_spinboxes_.size(); ++i) {
            if (joint_spinboxes_[i]) {
                joint_spinboxes_[i]->setValue(flight_pose[i]);
            }
        }
        syncUiToCommandState();
    });
    preset_row->addWidget(flight_button);

    layout->addLayout(preset_row);
    layout->addStretch(1);
    return group;
}

QWidget *SimRemoteWindow::createRcSliderCard(const QString &title,
                                             const QString &subtitle,
                                             int min_value,
                                             int max_value,
                                             int initial_value,
                                             RcSliderWidgets &widgets) {
    QFrame *card = new QFrame(this);
    card->setProperty("card", "panel");

    QVBoxLayout *layout = new QVBoxLayout(card);
    layout->setContentsMargins(16, 14, 16, 14);
    layout->setSpacing(8);

    QLabel *title_label = new QLabel(title, card);
    title_label->setProperty("role", "title");
    layout->addWidget(title_label);

    QLabel *subtitle_label = new QLabel(subtitle, card);
    subtitle_label->setProperty("role", "subtitle");
    layout->addWidget(subtitle_label);

    widgets.slider = new QSlider(Qt::Horizontal, card);
    widgets.slider->setRange(min_value, max_value);
    widgets.slider->setValue(initial_value);
    layout->addWidget(widgets.slider);

    widgets.value_label = createValueLabel(QString::number(initial_value));
    layout->addWidget(widgets.value_label);

    connect(widgets.slider, &QSlider::valueChanged, this, &SimRemoteWindow::syncUiToCommandState);

    return card;
}

QWidget *SimRemoteWindow::createJointControlRow(const QString &title,
                                                int joint_index,
                                                double min_value,
                                                double max_value,
                                                double initial_value) {
    QFrame *row = new QFrame(this);
    row->setProperty("card", "panel");

    QHBoxLayout *layout = new QHBoxLayout(row);
    layout->setContentsMargins(16, 12, 16, 12);
    layout->setSpacing(14);

    QLabel *label = new QLabel(title, row);
    label->setProperty("role", "title");
    layout->addWidget(label, 1);

    QLabel *hint = new QLabel(QStringLiteral("rad"), row);
    hint->setProperty("role", "subtitle");
    layout->addWidget(hint);

    QDoubleSpinBox *spinbox = new QDoubleSpinBox(row);
    spinbox->setDecimals(3);
    spinbox->setSingleStep(0.05);
    spinbox->setRange(min_value, max_value);
    spinbox->setValue(initial_value);
    layout->addWidget(spinbox);
    joint_spinboxes_[joint_index] = spinbox;

    connect(spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SimRemoteWindow::syncUiToCommandState);

    return row;
}

QComboBox *SimRemoteWindow::createComboBox(const QStringList &items) {
    QComboBox *combo = new QComboBox(this);
    combo->addItems(items);
    return combo;
}

QLabel *SimRemoteWindow::createValueLabel(const QString &text) const {
    QLabel *label = new QLabel(text, const_cast<SimRemoteWindow *>(this));
    label->setProperty("role", "value");
    return label;
}

void SimRemoteWindow::onConnectClicked() {
    if (!node_.init()) {
        QMessageBox::warning(this, QStringLiteral("ROS Master"),
                             QStringLiteral("Could not connect to the ROS master."));
        return;
    }

    connect_button_->setEnabled(false);
    statusBar()->showMessage(QStringLiteral("ROS link active. Publishing UAV and arm commands."));
    syncUiToCommandState();
}

void SimRemoteWindow::onArmClicked() {
    if (node_.armVehicle(true)) {
        statusBar()->showMessage(QStringLiteral("Arm command sent."));
    } else {
        statusBar()->showMessage(QStringLiteral("Arm command failed."));
    }
}

void SimRemoteWindow::onDisarmClicked() {
    if (node_.armVehicle(false)) {
        statusBar()->showMessage(QStringLiteral("Disarm command sent."));
    } else {
        statusBar()->showMessage(QStringLiteral("Disarm command failed."));
    }
}

void SimRemoteWindow::onModeManualClicked() {
    if (node_.setFlightMode("MANUAL")) {
        statusBar()->showMessage(QStringLiteral("Requested PX4 mode MANUAL."));
    } else {
        statusBar()->showMessage(QStringLiteral("Failed to request PX4 mode MANUAL."));
    }
}

void SimRemoteWindow::onModeOffboardClicked() {
    if (node_.setFlightMode("OFFBOARD")) {
        statusBar()->showMessage(QStringLiteral("Requested PX4 mode OFFBOARD."));
    } else {
        statusBar()->showMessage(QStringLiteral("Failed to request PX4 mode OFFBOARD."));
    }
}

void SimRemoteWindow::syncUiToCommandState() {
    if (roll_widgets_.value_label) {
        roll_widgets_.value_label->setText(QString::number(roll_widgets_.slider->value()));
    }
    if (pitch_widgets_.value_label) {
        pitch_widgets_.value_label->setText(QString::number(pitch_widgets_.slider->value()));
    }
    if (yaw_widgets_.value_label) {
        yaw_widgets_.value_label->setText(QString::number(yaw_widgets_.slider->value()));
    }
    if (throttle_widgets_.value_label) {
        throttle_widgets_.value_label->setText(QStringLiteral("%1%").arg(throttle_widgets_.slider->value()));
    }

    updateRcCommandsFromUi();
    updateJointCommandsFromUi();
}

void SimRemoteWindow::refreshVehicleState() {
    const bool connected = node_.isVehicleConnected();
    const bool armed = node_.isVehicleArmed();

    status_badge_->setText(badgeText(connected, armed));
    mode_badge_->setText(QStringLiteral("MODE %1").arg(QString::fromStdString(node_.currentMode())));
}

void SimRemoteWindow::updateRcCommandsFromUi() {
    // Keep the published RC channel order aligned with the UI labels:
    // RC1 roll, RC2 pitch, RC3 throttle, RC4 yaw.
    std::array<float, 7> axes = {
        sliderPercentToAxis(roll_widgets_.slider->value()),
        sliderPercentToAxis(pitch_widgets_.slider->value()),
        throttlePercentToAxis(throttle_widgets_.slider->value()),
        sliderPercentToAxis(yaw_widgets_.slider->value()),
        0.0f,
        0.0f,
        0.0f
    };

    std::array<int, 8> buttons = {{0, 0, 0, 0, 0, 0, 0, 0}};

    switch (flight_mode_combo_->currentIndex()) {
    case 0: axes[4] = -1.0f; break;
    case 1: axes[4] = 0.0f; break;
    case 2: axes[4] = 1.0f; break;
    default: axes[4] = 0.0f; break;
    }

    axes[5] = auto_gate_combo_->currentIndex() == 0 ? 0.0f : 1.0f;

    switch (mission_combo_->currentIndex()) {
    case 0: axes[6] = -1.0f; break;
    case 1: axes[6] = 0.0f; break;
    case 2: axes[6] = 1.0f; break;
    default: axes[6] = 0.0f; break;
    }

    buttons[0] = coordinate_combo_->currentIndex() == 1 ? 1 : 0;
    buttons[1] = delta_combo_->currentIndex() == 1 ? 1 : 0;
    buttons[2] = motor_latch_combo_->currentIndex() == 1 ? 1 : 0;

    node_.setRcAxes(axes);
    node_.setRcButtons(buttons);
}

void SimRemoteWindow::updateJointCommandsFromUi() {
    std::array<double, 6> joint_targets = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

    for (size_t i = 0; i < joint_spinboxes_.size(); ++i) {
        if (joint_spinboxes_[i]) {
            joint_targets[i] = joint_spinboxes_[i]->value();
        }
    }

    node_.setJointTargets(joint_targets);
}

float SimRemoteWindow::sliderPercentToAxis(int value) const {
    return static_cast<float>(value) / 100.0f;
}

float SimRemoteWindow::throttlePercentToAxis(int value) const {
    const float normalized = static_cast<float>(value) / 100.0f;
    return 1.0f - 2.0f * normalized;
}

}  // namespace qt_joystick
