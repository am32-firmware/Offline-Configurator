/*
  SitlUdpTransport - a QIODevice that talks to the AM32 SITL bootloader
  over UDP instead of a real serial port, for testing the 4-way / 1-wire
  configuration protocol against the simulated bootloader.

  write() frames the bytes as SITL input packets (magic 0x4453, type 4
  = 19200 serial) and sends them to the SITL input port; the bytes are
  also echoed into the read buffer, reproducing the one-wire self-echo a
  real single-wire adapter produces (the direct-mode protocol code
  relies on reading back its own transmission). Incoming type 4 packets
  from the bootloader are unframed into the read buffer.

  Use a port string of the form "sitl:[host:]port" (e.g. "sitl:57733"
  or "sitl:127.0.0.1:57733").
 */
#pragma once

#include <QIODevice>
#include <QByteArray>
#include <QUdpSocket>
#include <QString>

class SitlUdpTransport : public QIODevice {
  Q_OBJECT
 public:
  explicit SitlUdpTransport(const QString &spec, QObject *parent = nullptr);

  // spec is "sitl:[host:]port"; returns false if it is not a sitl spec
  static bool isSitlSpec(const QString &spec);

  bool open(OpenMode mode) override;
  void close() override;
  bool isSequential() const override { return true; }
  qint64 bytesAvailable() const override;
  bool waitForReadyRead(int msecs) override;
  bool waitForBytesWritten(int msecs) override;

 protected:
  qint64 readData(char *data, qint64 maxlen) override;
  qint64 writeData(const char *data, qint64 len) override;

 private:
  void drainSocket();

  QUdpSocket m_sock;
  QString m_host;
  quint16 m_port = 0;
  QByteArray m_rx;
  bool m_fresh = false;  // new bytes arrived since last readiness report
};
