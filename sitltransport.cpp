#include "sitltransport.h"

#include <QHostAddress>
#include <QElapsedTimer>

static const quint16 SITL_MAGIC = 0x4453;
static const quint8 SITL_TYPE_SERIAL = 4;
static const quint16 SITL_FLAG_IDLE_HIGH = 0x0001;
static const quint16 SITL_FLAG_GAP = 0x0004;  // leading line idle (frame separator)
static const int SITL_SERIAL_MAX = 200;

SitlUdpTransport::SitlUdpTransport(const QString &spec, QObject *parent)
    : QIODevice(parent) {
  // spec: "sitl:[host:]port"
  QString rest = spec.mid(spec.indexOf(':') + 1);
  const int lastColon = rest.lastIndexOf(':');
  if (lastColon >= 0) {
    m_host = rest.left(lastColon);
    m_port = rest.mid(lastColon + 1).toUShort();
  } else {
    m_host = "127.0.0.1";
    m_port = rest.toUShort();
  }
}

bool SitlUdpTransport::isSitlSpec(const QString &spec) {
  return spec.startsWith("sitl:");
}

bool SitlUdpTransport::open(OpenMode mode) {
  if (m_port == 0) {
    setErrorString("invalid sitl port");
    return false;
  }
  // bind on any interface so a remote SITL (started with --bind-any)
  // can reach us; the destination host/port come from the spec
  if (!m_sock.bind(QHostAddress(QHostAddress::AnyIPv4), 0)) {
    setErrorString(m_sock.errorString());
    return false;
  }
  setOpenMode(mode);
  return true;
}

void SitlUdpTransport::close() {
  m_sock.close();
  m_rx.clear();
  setOpenMode(NotOpen);
}

void SitlUdpTransport::drainSocket() {
  while (m_sock.hasPendingDatagrams()) {
    QByteArray dg;
    dg.resize(int(m_sock.pendingDatagramSize()));
    m_sock.readDatagram(dg.data(), dg.size());
    if (dg.size() < 6) {
      continue;
    }
    const quint16 magic = quint8(dg[0]) | (quint8(dg[1]) << 8);
    const quint8 type = quint8(dg[2]);
    const quint8 len = quint8(dg[3]);
    if (magic != SITL_MAGIC || type != SITL_TYPE_SERIAL) {
      continue;
    }
    if (dg.size() != 6 + len) {
      continue;
    }
    m_rx.append(dg.mid(6, len));
    m_fresh = true;
  }
}

qint64 SitlUdpTransport::bytesAvailable() const {
  return m_rx.size() + QIODevice::bytesAvailable();
}

qint64 SitlUdpTransport::readData(char *data, qint64 maxlen) {
  drainSocket();
  const qint64 n = qMin<qint64>(maxlen, m_rx.size());
  memcpy(data, m_rx.constData(), n);
  m_rx.remove(0, int(n));
  return n;
}

qint64 SitlUdpTransport::writeData(const char *data, qint64 len) {
  // frame as one or more type 4 serial packets (<=200 bytes each) and
  // echo the bytes into our own read buffer (one-wire self-echo).
  // Each write() is one protocol command/frame: mark the first packet
  // with a leading line-idle gap so the bootloader treats it as a new
  // frame, as inter-command latency does on a real serial wire
  for (qint64 off = 0; off < len; off += SITL_SERIAL_MAX) {
    const int n = int(qMin<qint64>(SITL_SERIAL_MAX, len - off));
    quint16 flags = SITL_FLAG_IDLE_HIGH;
    if (off == 0) {
      flags |= SITL_FLAG_GAP;
    }
    QByteArray pkt;
    pkt.append(char(SITL_MAGIC & 0xFF));
    pkt.append(char(SITL_MAGIC >> 8));
    pkt.append(char(SITL_TYPE_SERIAL));
    pkt.append(char(n));
    pkt.append(char(flags & 0xFF));
    pkt.append(char(flags >> 8));
    pkt.append(data + off, n);
    m_sock.writeDatagram(pkt, QHostAddress(m_host), m_port);
  }
  m_rx.append(data, int(len));
  m_fresh = true;
  emit readyRead();
  return len;
}

/*
  Return true only when new data has arrived since the last time we
  reported readiness, mirroring a real serial port: callers use
  `while (waitForReadyRead(t)) {}` loops that do not drain the buffer
  between iterations, so returning true on merely-non-empty (unread) data
  would spin forever. m_fresh is set when bytes are appended (socket or
  the local one-wire self-echo) and cleared when we announce them.
 */
bool SitlUdpTransport::waitForReadyRead(int msecs) {
  QElapsedTimer t;
  t.start();
  for (;;) {
    drainSocket();
    if (m_fresh) {
      m_fresh = false;
      return true;
    }
    const int remaining = msecs < 0 ? -1 : int(msecs - t.elapsed());
    if (msecs >= 0 && remaining <= 0) {
      return false;
    }
    m_sock.waitForReadyRead(remaining < 0 ? 100 : qMin(remaining, 100));
  }
}

bool SitlUdpTransport::waitForBytesWritten(int) {
  // datagrams are sent synchronously in writeData
  return true;
}
