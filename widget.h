#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QSerialPort>
#include <QLabel>
#include <QComboBox>

class OutConsole;
class FourWayIF;
class BF_ROOTLOADER;

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

private slots:

    uint8_t mspSerialChecksumBuf(uint8_t checksum, const uint8_t *data, int len);

    void send_mspCommand(uint8_t cmd, QByteArray payload);

    void on_sendButton_clicked();

    void on_disconnectButton_clicked();

    void on_pushButton_clicked();

    void on_sendMessageButton_clicked();

    void on_passthoughButton_clicked();

    void on_horizontalSlider_sliderMoved(int position);

    void on_serialSelectorBox_currentTextChanged(const QString &arg1);

   // void on_fourWaySendButton_clicked();

    void on_pushButton_2_clicked();


    void on_loadBinary_clicked();

    void on_writeBinary_clicked();

    void on_VerifyFlash_clicked();

   // void on_sendMspComButton_clicked();

    void on_initMotor2_clicked();

    void on_initMotor1_clicked();

    void on_initMotor3_clicked();

    void on_initMotor4_clicked();

    void on_writeEEPROM_clicked();

    void on_sendFirstEEPROM_clicked();

    void on_devSettings_stateChanged(int arg1);

    void on_endPassthrough_clicked();

    QByteArray convertFromHex();

    void on_checkBox_stateChanged(int arg1);

    void on_initMotor1_2_clicked();

    void on_initMotor2_2_clicked();

    void on_initMotor3_2_clicked();

    void on_initMotor4_2_clicked();

    void on_startupPowerSlider_valueChanged(int value);

    void on_timingAdvanceSlider_valueChanged(int value);

    void on_pwmFreqSlider_valueChanged(int value);

    void on_varPWMCheckBox_stateChanged(int arg1);

    void on_m1MSPSlider_valueChanged(int value);

    void on_m2MSPSlider_valueChanged(int value);

    void on_m3MSPSlider_valueChanged(int value);

    void on_m4MSPSlider_valueChanged(int value);

    void on_ConnectedButton_clicked();

    void on_motorKVSlider_valueChanged(int value);

    void on_motorPolesSlider_valueChanged(int value);

  //  void on_beepVolumeSlider_sliderMoved(int position);

    void on_beepVolumeSlider_valueChanged(int value);

  //  void on_thirtymsTelemBox_stateChanged(int arg1);

    void on_lowThresholdLineEdit_editingFinished();

    void on_servoLowSlider_valueChanged(int value);

    void on_servoHighSlider_valueChanged(int value);

    void on_servoNeutralSlider_valueChanged(int value);

    void on_lowVoltageThresholdSlider_valueChanged(int value);

    void on_highThresholdLineEdit_editingFinished();

    void on_servoNeuralLineEdit_editingFinished();

    void on_servoDeadBandSlider_valueChanged(int value);

    void on_servoDeadbandLineEdit_editingFinished();

    void on_lowVoltageLineEdit_editingFinished();

    void on_writeEEPROM_2_clicked();

    void on_initMotor1_3_clicked();

    void on_initMotor2_3_clicked();

    void on_initMotor3_3_clicked();

    void on_initMotor4_3_clicked();

    void serialInfoStuff();

//   void on_serialSelectorBox_activated(const QString &arg1);

 //   void on_uploadMusic_clicked();
    void resetESC();

    void on_dragBrakeSlider_valueChanged(int value);

    void on_sineStartupSlider_valueChanged(int value);

    void on_sineModePowerSlider_valueChanged(int value);

    void on_runningBrakeStrength_valueChanged(int value);

    void on_currentSlider_valueChanged(int value);

    void on_temperatureSlider_valueChanged(int value);

    void on_crawler_default_button_clicked();

    void on_saveConfigButton_clicked();

    void on_loadConfigButton_clicked();

    void on_OfflineCheckBox_stateChanged(int arg1);

    void on_maxRocSlider_valueChanged(int value);

    void on_minDutySlider_valueChanged(int value);

    void on_rocLineEdit_textEdited(const QString &arg1);

    void on_minDutyLineEdit_editingFinished();

    void on_rocLineEdit_editingFinished();

    void on_activeBrakeSlider_valueChanged(int value);

    void on_activeBrakeLineEdit_editingFinished();

    void on_lowVoltageCuttoffBox_stateChanged(int arg1);

    void on_absoluteVotlageCheckbox_stateChanged(int arg1);

    void on_brakecheckbox_stateChanged(int arg1);

    void on_activeBrakeCheckbox_stateChanged(int arg1);

    void on_absoluteVoltageSlider_valueChanged(int value);

    void on_currentLimitPedit_editingFinished();

    void on_currentLimitDedit_editingFinished();

    void on_currentLimitIedit_editingFinished();

    void on_uploadMusic_clicked();

    void on_initMotor1_4_clicked();

    void on_initMotor2_4_clicked();

    void on_initMotor3_4_clicked();

    void on_initMotor4_4_clicked();

private:

//    void initActionsConnections();

private:
    Ui::Widget *ui;
    void loadBinFile();
    int  getshift(int some_number);
    void sendMSPThrottle();
    void showSingleMotor(bool tf);
    void connectSerial();
    void hide4wayButtons(bool b);
    void hideESCSettings(bool b);
    void hideEEPROMSettings(bool b);
    void allup();
    bool connectMotor(uint8_t motor);
    void sendFirstEeprom(uint8_t eeprom_type);
    void closeSerialPort();
    void readInitData();
    void sendDirect(const QByteArray sendbuffer, uint16_t buffer_size, uint16_t address);
    void readData();
    void putData(const QByteArray &data);
    void verifyFlash();
    void writeData(const QByteArray &data);
    void showStatusMessage(const QString &message);
    bool getMusic();
    bool writeMusic();
    void endTimer();
    typedef union __attribute__ ((packed)) {
        uint8_t bytes[2];
        uint16_t word;
    } uint8_16_u;
    enum messages{
        ACK_OK,
        BAD_ACK,
        CRC_ERROR

    };
   bool parseMSPMessage = true;
   bool more_to_come = false;
   bool timerdone = false;

   uint8_t retries = 0;
   uint8_t max_retries = 16;
   uint8_t number_of_ports = 0;

//    typedef struct ioMem_s {
//        uint8_t D_NUM_BYTES;
//        uint8_t D_FLASH_ADDR_H;
//        uint8_t D_FLASH_ADDR_L;
//        uint8_t *D_PTR_I;
//    } ioMem_t;
private:
    bool firstOffline = false;
    FourWayIF *four_way = nullptr;
    BF_ROOTLOADER *RL =nullptr;
    OutConsole *msg_console = nullptr;
    QSerialPort *m_serial = nullptr;
    QLabel *statuslabel = nullptr;
    QByteArray *input_buffer = nullptr;
    QByteArray *bluejay_tune = nullptr;
    QString filename;
    QByteArray *eeprom_buffer = nullptr;
    QByteArray *music_buffer = nullptr;
    uint16_t eeprom_address = 0x7c00;
    bool musicBufferFull = false;
    int motor1throttle;
    int motor2throttle;
    int motor3throttle;
    int motor4throttle;
 //   uint8_t output_bin_data[255] = {0};
  //  QComboBox *m_box = nullptr;

 //   InConsole *rsp_console = nullptr;


};
#endif // WIDGET_H
