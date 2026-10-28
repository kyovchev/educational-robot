#include <Servo.h>

#define VIRTUAL_ID 7
#define PWM_PIN 6
#define PWM_MIN_US 500
#define PWM_MAX_US 2500
#define STS_POS_MAX 4095

#define REG_GOAL_ACC 0x29
#define REG_GOAL_POS 0x2A
#define REG_PRESENT_POS 0x38
#define REG_MOVING 0x3A
#define REG_VOLTAGE 0x3E
#define REG_TEMP 0x3F
#define REG_TORQUE_EN 0x28
#define REG_ID 0x05
#define REG_BAUD 0x06

#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_ACTION 0x05
#define INST_SYNC_WRITE 0x83
#define BROADCAST_ID 0xFE

// ── Parser ────────────────────────────────────────────────────────────────────
#define PKT_MAX 128
uint8_t pkt[PKT_MAX];
uint16_t pktTotal = 0;
uint16_t pktIdx = 0;
uint8_t parserState = 0;  // 0=idle 1=hdr2 2=len 3=data

// ── Virtual servo state ───────────────────────────────────────────────────────
Servo pwm;
int16_t vPos = 2047;
uint8_t vTorque = 1;
uint8_t vMoving = 0;
uint8_t vVolt = 74;
uint8_t vTemp = 25;
bool pending = false;
int16_t pendingPos = 2047;

// ── Helpers ───────────────────────────────────────────────────────────────────
static inline int16_t le16(uint8_t lo, uint8_t hi) {
  return (int16_t)(((uint16_t)hi << 8) | lo);
}

void setPos(int16_t pos) {
  if (pos < 0) pos = 0;
  if (pos > STS_POS_MAX) pos = STS_POS_MAX;
  vPos = pos;
  pwm.writeMicroseconds(
    (int)map((long)pos, 0L, (long)STS_POS_MAX, (long)PWM_MIN_US, (long)PWM_MAX_US));
}

void respond(uint8_t error, const uint8_t *data, uint8_t n) {
  uint8_t len = n + 2;
  uint16_t s = (uint16_t)VIRTUAL_ID + len + error;
  Serial.write((uint8_t)0xFF);
  Serial.write((uint8_t)0xFF);
  Serial.write((uint8_t)VIRTUAL_ID);
  Serial.write(len);
  Serial.write(error);
  for (uint8_t i = 0; i < n; i++) {
    Serial.write(data[i]);
    s += data[i];
  }
  Serial.write((uint8_t)((~s) & 0xFF));
}

void ack() {
  const uint8_t e[1] = { 0 };
  respond(0, e, 0);
}

void handleRead(uint8_t startReg, uint8_t count) {
  uint8_t resp[32];
  uint8_t ri = 0;
  for (uint8_t r = startReg; r < (uint8_t)(startReg + count) && ri < 32; r++) {
    switch (r) {
      case REG_ID: resp[ri++] = VIRTUAL_ID; break;
      case REG_BAUD: resp[ri++] = 0; break;
      case REG_TORQUE_EN: resp[ri++] = vTorque; break;
      case REG_GOAL_POS: resp[ri++] = (uint8_t)(vPos & 0xFF); break;
      case REG_GOAL_POS + 1: resp[ri++] = (uint8_t)((vPos >> 8) & 0xFF); break;
      case REG_PRESENT_POS: resp[ri++] = (uint8_t)(vPos & 0xFF); break;
      case REG_PRESENT_POS + 1: resp[ri++] = (uint8_t)((vPos >> 8) & 0xFF); break;
      case REG_MOVING: resp[ri++] = vMoving; break;
      case REG_VOLTAGE: resp[ri++] = vVolt; break;
      case REG_TEMP: resp[ri++] = vTemp; break;
      default: resp[ri++] = 0; break;
    }
  }
  respond(0, resp, ri);
}

void handleVirtual() {
  uint8_t instr = pkt[4];
  uint8_t *P = pkt + 5;
  uint8_t nP = (pkt[3] >= 2) ? pkt[3] - 2 : 0;
  switch (instr) {
    case INST_PING: ack(); break;
    case INST_READ: (nP >= 2) ? handleRead(P[0], P[1]) : ack(); break;
    case INST_WRITE:
      if (nP >= 1) {
        if (P[0] == REG_GOAL_ACC && nP >= 4) setPos(le16(P[2], P[3]));
        else if (P[0] == REG_GOAL_POS && nP >= 3) setPos(le16(P[1], P[2]));
        else if (P[0] == REG_TORQUE_EN && nP >= 2) vTorque = P[1];
      }
      ack();
      break;
    case INST_REG_WRITE:
      if (nP >= 4 && P[0] == REG_GOAL_ACC) {
        pendingPos = le16(P[2], P[3]);
        pending = true;
      } else if (nP >= 3 && P[0] == REG_GOAL_POS) {
        pendingPos = le16(P[1], P[2]);
        pending = true;
      }
      ack();
      break;
    case INST_ACTION:
      if (pending) {
        setPos(pendingPos);
        pending = false;
      }
      ack();
      break;
    default: ack(); break;
  }
}

void handleSync() {
  uint8_t *P = pkt + 5;
  uint8_t nP = (pkt[3] >= 2) ? pkt[3] - 2 : 0;
  if (nP < 4) goto forward;
  {
    uint8_t startReg = P[0];
    uint8_t dataPerServo = P[1];
    uint8_t off = 2;
    while (off + 1 + dataPerServo <= nP) {
      if (P[off] == VIRTUAL_ID) {
        uint8_t *D = P + off + 1;
        if (startReg == REG_GOAL_ACC && dataPerServo >= 3) setPos(le16(D[1], D[2]));
        else if (startReg == REG_GOAL_POS && dataPerServo >= 2) setPos(le16(D[0], D[1]));
      }
      off += 1 + dataPerServo;
    }
  }
forward:
  // Forward full packet to real bus (real servos ignore ID 99)
  Serial3.write(pkt, 4 + pkt[3]);
}

// ── setup / loop ──────────────────────────────────────────────────────────────
void setup() {
  pwm.attach(PWM_PIN, PWM_MIN_US, PWM_MAX_US);
  pwm.writeMicroseconds(1500);
  Serial.begin(1000000);
  Serial3.begin(1000000);
}

void loop() {
  while (Serial.available()) {
    uint8_t b = Serial.read();

    switch (parserState) {
      case 0:  // idle — wait for first 0xFF
        if (b == 0xFF) {
          pkt[0] = b;
          pktIdx = 1;
          parserState = 1;
        }
        break;

      case 1:  // wait for second 0xFF (absorb extra 0xFF)
        if (b == 0xFF) {
          pkt[1] = b;
          pktIdx = 2;
        } else {
          pkt[1] = 0xFF;
          pkt[2] = b;
          pktIdx = 3;
          parserState = 2;
        }
        break;

      case 2:  // LEN
        pkt[3] = b;
        pktIdx = 4;
        if (b < 2 || (4 + (uint16_t)b) > PKT_MAX) {
          parserState = 0;
          break;
        }
        pktTotal = 4 + b;
        parserState = 3;
        break;

      case 3:  // DATA
        pkt[pktIdx++] = b;
        if (pktIdx < pktTotal) break;
        {
          // verify checksum
          uint16_t s = 0;
          for (uint16_t i = 2; i < pktTotal - 1; i++) s += pkt[i];
          if (((~s) & 0xFF) == pkt[pktTotal - 1]) {
            uint8_t id = pkt[2];
            uint8_t instr = pkt[4];
            if (instr == INST_SYNC_WRITE) handleSync();
            else if (id == VIRTUAL_ID) handleVirtual();
            else {
              // forward to real bus, proxy response back
              Serial3.write(pkt, pktTotal);
              // forward response back to PC
              uint32_t t = millis();
              uint8_t h = 0;
              uint8_t rb[64];
              uint8_t ri = 0;
              uint8_t rlen = 0;
              uint8_t phase = 0;
              while (millis() - t < 10) {
                if (!Serial3.available()) continue;
                uint8_t c = Serial3.read();
                t = millis();
                if (phase == 0) {
                  if (c == 0xFF) phase = 1;
                } else if (phase == 1) {
                  if (c == 0xFF) phase = 2;
                  else phase = 0;
                } else if (phase == 2) {
                  rb[ri++] = c;
                  phase = 3;
                } else if (phase == 3) {
                  rb[ri++] = c;
                  rlen = c;
                  phase = 4;
                } else if (phase == 4) {
                  rb[ri++] = c;
                  if (ri >= rlen + 2) {
                    Serial.write((uint8_t)0xFF);
                    Serial.write((uint8_t)0xFF);
                    Serial.write(rb, ri);
                    break;
                  }
                }
              }
            }
          }
        }
        parserState = 0;
        break;
    }
  }
}
