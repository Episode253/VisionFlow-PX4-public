#ifndef QT_JOYSTICK_SIM_REMOTE_WINDOW_HPP_
#define QT_JOYSTICK_SIM_REMOTE_WINDOW_HPP_

#include <array>

#include <QtWidgets/QMainWindow>

#include "sim_remote_node.hpp"

class QLabel;
class QPushButton;
class QComboBox;
class QDoubleSpinBox;
class QSlider;
class QTimer;

namespace qt_joystick {

class SimRemoteWindow : public QMainWindow {
    Q_OBJECT

public:
    SimRemoteWindow(int argc, char **argv, QWidget *parent = nullptr);
    ~SimRemoteWindow() override;

private Q_SLOTS:
    void onConnectClicked();
    void onArmClicked();
    void onDisarmClicked();
    void onModeManualClicked();
    void onModeOffboardClicked();
    void syncUiToCommandState();
    void refreshVehicleState();

private:
    struct RcSliderWidgets {
        QSlider *slider{nullptr};
        QLabel *value_label{nullptr};
    };

    void buildUi();
    QWidget *buildFlightPanel();
    QWidget *buildMissionPanel();
    QWidget *buildArmPanel();
    QWidget *buildConnectionBar();

    QWidget *createRcSliderCard(const QString &title,
                                const QString &subtitle,
                                int min_value,
                                int max_value,
                                int initial_value,
                                RcSliderWidgets &widgets);
    QWidget *createJointControlRow(const QString &title,
                                   int joint_index,
                                   double min_value,
                                   double max_value,
                                   double initial_value);
    QComboBox *createComboBox(const QStringList &items);
    QLabel *createValueLabel(const QString &text) const;
    void updateRcCommandsFromUi();
    void updateJointCommandsFromUi();
    float sliderPercentToAxis(int value) const;
    float throttlePercentToAxis(int value) const;

    SimRemoteNode node_;
    QPushButton *connect_button_{nullptr};
    QLabel *status_badge_{nullptr};
    QLabel *mode_badge_{nullptr};

    RcSliderWidgets roll_widgets_;
    RcSliderWidgets pitch_widgets_;
    RcSliderWidgets yaw_widgets_;
    RcSliderWidgets throttle_widgets_;

    QComboBox *flight_mode_combo_{nullptr};
    QComboBox *auto_gate_combo_{nullptr};
    QComboBox *mission_combo_{nullptr};
    QComboBox *coordinate_combo_{nullptr};
    QComboBox *delta_combo_{nullptr};
    QComboBox *motor_latch_combo_{nullptr};

    std::array<QDoubleSpinBox *, 6> joint_spinboxes_ {{nullptr, nullptr, nullptr, nullptr, nullptr, nullptr}};

    QTimer *state_timer_{nullptr};
};

}  // namespace qt_joystick

#endif  // QT_JOYSTICK_SIM_REMOTE_WINDOW_HPP_
