// cosim.cpp - runs the REAL compiled Mega firmware (build/sketch.ino.elf) in simavr, wired to the SAME
// custom chips (chips/*.chip.c) exactly as in diagram.json, through mock_wokwi. A stand-in for pressing
// "play" in Wokwi, so the whole project can be checked on a PC / in CI:
//   boot (8 ToF on new addresses, gyro), link frames both ways, wheels under PI control, encoders, QTR,
//   TCS3200, sonar, keypad, battery reading and the 250 ms watchdog.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include "mock_wokwi.h"
extern "C" {
#include <simavr/sim_avr.h>
#include <simavr/sim_elf.h>
#include <simavr/avr_ioport.h>
#include <simavr/avr_twi.h>
#include <simavr/avr_uart.h>
#include <simavr/avr_adc.h>
void vl53_init(); void gyro_init(); void tcs_init(); void qtr_init(); void motor_init(); void brain_init();
}
using namespace mock;

// ---------------------------------------------------------------- Arduino Mega pin -> AVR port / bit
static const std::map<std::string, std::pair<char, int>> MEGA = {
  {"2", {'E', 4}}, {"3", {'E', 5}}, {"4", {'G', 5}}, {"5", {'E', 3}}, {"6", {'H', 3}}, {"7", {'H', 4}}, {"8", {'H', 5}},
  {"9", {'H', 6}}, {"10", {'B', 4}}, {"11", {'B', 5}}, {"16", {'H', 1}}, {"17", {'H', 0}}, {"18", {'D', 3}},
  {"19", {'D', 2}}, {"20", {'D', 1}}, {"21", {'D', 0}}, {"22", {'A', 0}}, {"23", {'A', 1}}, {"24", {'A', 2}},
  {"25", {'A', 3}}, {"26", {'A', 4}}, {"27", {'A', 5}}, {"28", {'A', 6}}, {"29", {'A', 7}}, {"30", {'C', 7}},
  {"31", {'C', 6}}, {"32", {'C', 5}}, {"33", {'C', 4}}, {"34", {'C', 3}}, {"35", {'C', 2}}, {"36", {'C', 1}},
  {"37", {'C', 0}}, {"38", {'D', 7}}, {"39", {'G', 2}}, {"40", {'G', 1}}, {"41", {'G', 0}}, {"42", {'L', 7}},
  {"43", {'L', 6}}, {"44", {'L', 5}}, {"45", {'L', 4}}, {"46", {'L', 3}}, {"47", {'L', 2}}, {"48", {'L', 1}},
  {"49", {'L', 0}}, {"50", {'B', 3}}, {"51", {'B', 2}}, {"53", {'B', 0}}, {"A1", {'F', 1}}, {"A2", {'F', 2}}};
static int ddr_addr(char port) {                     // data-space address of DDRx (PINx = DDRx - 1, PORTx = DDRx + 1)
  static const std::map<char, int> a = {{'A', 0x21}, {'B', 0x24}, {'C', 0x27}, {'D', 0x2A}, {'E', 0x2D}, {'F', 0x30},
                                        {'G', 0x33}, {'H', 0x101}, {'J', 0x104}, {'K', 0x107}, {'L', 0x10A}};
  return a.at(port);
}

avr_t *avr;
struct MPin { char port; int bit; int net; avr_irq_t *irq; bool out = false, pu = false; int outv = 0; };
std::vector<MPin> mpins;
std::map<int, int> net_to_mpin;
std::map<std::string, int> part_chip;
std::map<char, avr_ioport_external_t> ext;                // part id -> mock chip index
bool feeding = false;

bool pwm_pin(int net);
static void on_avr_pin(avr_irq_t *, uint32_t value, void *param) {
  MPin &p = mpins[(intptr_t)param];
  p.outv = value ? 1 : 0;
  if (p.out && !pwm_pin(p.net)) mcu_drive(p.net, p.outv);
}
static void sync_directions() {                      // DDR / pull-ups -> MCU side of each net
  for (auto &p : mpins) {
    int d = ddr_addr(p.port);
    bool out = (avr->data[d] >> p.bit) & 1, pu = !out && ((avr->data[d + 1] >> p.bit) & 1);
    if (out != p.out || pu != p.pu) {
      p.out = out; p.pu = pu;
      if (out && !pwm_pin(p.net)) mcu_drive(p.net, (avr->data[d + 1] >> p.bit) & 1);
      else { mcu_drive(p.net, -1); mcu_pullup(p.net, pu); }
    }
  }
}

// ---------------------------------------------------------------- I2C bridge (simavr TWI <-> mock bus)
avr_irq_t *twi_in;
static uint8_t twi_sel = 0;
bool twi_debug = getenv("TWI_DEBUG") != nullptr;
static void on_twi(avr_irq_t *, uint32_t value, void *) {
  avr_twi_msg_irq_t v; v.u.v = value;
  if (twi_debug) printf("twi msg %02X addr %02X data %02X\n", v.u.twi.msg, v.u.twi.addr, v.u.twi.data);
  // simavr 1.6 master: START+addr = address phase, WRITE = data byte out, READ = wants a byte, STOP
  if (v.u.twi.msg & TWI_COND_START) {
    if (twi_sel) i2c_stop();                          // repeated start
    twi_sel = 0;
    if (v.u.twi.addr && i2c_start(v.u.twi.addr >> 1, v.u.twi.addr & 1)) twi_sel = v.u.twi.addr;
    avr_raise_irq(twi_in, avr_twi_irq_msg(TWI_COND_ACK, v.u.twi.addr, twi_sel ? 1 : 0));
  }
  if (v.u.twi.msg & TWI_COND_WRITE) {
    bool ack = twi_sel && i2c_write(v.u.twi.data);
    avr_raise_irq(twi_in, avr_twi_irq_msg(TWI_COND_ACK, twi_sel, ack ? 1 : 0));
  }
  if (v.u.twi.msg & TWI_COND_READ)
    avr_raise_irq(twi_in, avr_twi_irq_msg(TWI_COND_READ, twi_sel, twi_sel ? i2c_read() : 0xFF));
  if (v.u.twi.msg & TWI_COND_STOP) { if (twi_sel) i2c_stop(); twi_sel = 0; }
}

// ---------------------------------------------------------------- UARTs
std::string usb_line;
std::vector<std::string> usb_log;
static void on_uart0(avr_irq_t *, uint32_t v, void *) {
  if (v == '\n') { printf("    [Mega USB] %s\n", usb_line.c_str()); usb_log.push_back(usb_line); usb_line.clear(); }
  else if (v != '\r') usb_line += (char)v;
}
avr_irq_t *uart2_in;
int brain_uart = -1;
#pragma pack(push, 1)
struct Telemetry { uint32_t ms; int32_t encL, encR; float yaw, gz; uint16_t us[2], tof[8], line[8], tcsFloor[4], tcsFront[4], vbat_mv; uint8_t keys, flags, seq; };
#pragma pack(pop)
Telemetry last_t;
int tele_frames = 0;
std::vector<uint8_t> rx2;
static void on_uart2(avr_irq_t *, uint32_t v, void *) {
  uart_inject(brain_uart, (const uint8_t *)&v, 1);    // the brain stub chip decodes it too
  rx2.push_back((uint8_t)v);
  while (rx2.size() >= 4 && !(rx2[0] == 0xA5 && rx2[1] == 0x5A)) rx2.erase(rx2.begin());
  if (rx2.size() >= 4 && rx2.size() >= (size_t)rx2[3] + 5) {
    uint8_t n = rx2[3], s = rx2[2] + n;
    for (int i = 0; i < n; i++) s += rx2[4 + i];
    if (s == rx2[4 + n] && rx2[2] == 0x54 && n == sizeof(Telemetry)) { memcpy(&last_t, &rx2[4], n); tele_frames++; }
    rx2.erase(rx2.begin(), rx2.begin() + n + 5);
  }
}

// ---------------------------------------------------------------- HC-SR04 (Wokwi has its own; a tiny model here)
struct Sonar { int trig, echo; double mm; uint64_t fall = 0; };
std::vector<Sonar> sonars;

static uint64_t sim_ns() { return (uint64_t)(avr->cycle * 62.5); }

// simavr does not drive the pins from phase-correct PWM (Arduino timers 3 / 4), so the PWM on the
// DRV8833 inputs is made here from the timer registers: COM bits on -> 31.37 kHz, high while count < OCR.
struct Pwm { const char *pin; int tccr, com_shift, ocr; int net = -1; bool on = false; };
std::vector<Pwm> pwms = {{"5", 0x90, 6, 0x98}, {"6", 0xA0, 6, 0xA8}, {"7", 0xA0, 4, 0xAA}, {"8", 0xA0, 2, 0xAC}};
static void pwm_step() {
  uint64_t c = avr->cycle % 510;                     // phase correct, 8 bit, no prescaler: up 0..255, down 255..0
  int cnt = c < 255 ? (int)c : (int)(510 - c);
  for (auto &p : pwms) {
    if (p.net < 0) continue;
    bool on = (avr->data[p.tccr] >> p.com_shift) & 3;
    if (!on) { if (p.on) { p.on = false; } continue; }
    p.on = true;
    int ocr = avr->data[p.ocr];
    mcu_drive(p.net, cnt < ocr ? 1 : 0);
  }
}
static void run_ms(double ms) {
  uint64_t end = sim_ns() + (uint64_t)(ms * 1e6);
  int k = 0;
  while (sim_ns() < end) {
    uint64_t target = avr->cycle + 16;                // 1 us slices
    while (avr->cycle < target) {
      int st = avr_run(avr);
      if (st == cpu_Done || st == cpu_Crashed) { printf("AVR stopped (%d)\n", st); exit(2); }
    }
    uint64_t t = sim_ns();
    if (t > now()) run_for_ns(t - now());
    pwm_step();
    if ((++k & 7) == 0) sync_directions();
    for (auto &s : sonars) {                         // echo pulse after a TRIG pulse
      int tl = level(s.trig);
      if (!tl && s.fall == 1) s.fall = t + 400000;
      if (tl && !s.fall) s.fall = 1;
      if (s.fall > 1 && t >= s.fall && !level(s.echo)) mcu_drive(s.echo, 1), s.fall = t + (uint64_t)(s.mm * 5830);
      else if (s.fall > 1 && t >= s.fall && level(s.echo)) mcu_drive(s.echo, -1), s.fall = 0;
    }
  }
}

bool pwm_pin(int net) { for (auto &p : pwms) if (p.net == net && p.on) return true; return false; }
static int fails = 0;
#define CHECK(c, ...) do { if (c) printf("  ok    "); else { printf("  FAIL  "); fails++; } printf(__VA_ARGS__); printf("\n"); } while (0)

int main(int argc, char **argv) {
  const char *elf = argc > 1 ? argv[1] : "../build/sketch.ino.elf";
  const char *wiring = argc > 2 ? argv[2] : "_build/wiring.txt";
  // ---- wiring from diagram.json
  std::map<std::string, std::string> part_type;
  std::map<std::string, std::map<std::string, uint32_t>> part_attr;
  std::map<std::string, int> endpoint_net;
  std::ifstream f(wiring);
  std::string line;
  while (std::getline(f, line)) {
    std::istringstream ss(line);
    std::string kind, a, b;
    ss >> kind >> a;
    if (kind == "part") {
      ss >> b; part_type[a] = b;
      std::string kv;
      while (ss >> kv) { auto e = kv.find('='); if (e != std::string::npos && isdigit((unsigned char)kv[e + 1])) part_attr[a][kv.substr(0, e)] = (uint32_t)atol(kv.c_str() + e + 1); }
    } else if (kind == "net") {
      int n = net(a);
      while (ss >> b) endpoint_net[b] = n;
    }
  }
  // ---- chips, in diagram order, each pin on its diagram net
  std::map<std::string, void (*)()> inits = {{"chip-vl53l0x", vl53_init}, {"chip-l3g4200d", gyro_init}, {"chip-tcs3200", tcs_init},
                                             {"chip-qtr8rc", qtr_init}, {"chip-n20motor", motor_init}, {"chip-brainstub", brain_init}};
  std::vector<std::string> order;
  { std::ifstream g(wiring); while (std::getline(g, line)) { std::istringstream ss(line); std::string k, id; ss >> k >> id; if (k == "part") order.push_back(id); } }
  for (auto &id : order) {
    auto it = inits.find(part_type[id]);
    if (it == inits.end()) continue;
    std::map<std::string, int> nets;
    for (auto &e : endpoint_net) if (e.first.rfind(id + ":", 0) == 0) nets[e.first.substr(id.size() + 1)] = e.second;
    part_chip[id] = chip_count();
    if (part_type[id] == "chip-brainstub") brain_uart = 0;
    begin_chip(it->second, nets, part_attr[id]);
  }
  for (auto &id : order)
    if (part_type[id] == "wokwi-hc-sr04")
      sonars.push_back({endpoint_net[id + ":TRIG"], endpoint_net[id + ":ECHO"], (double)part_attr[id]["distance"]});
  // ---- the AVR
  elf_firmware_t fw;
  memset(&fw, 0, sizeof(fw));
  if (elf_read_firmware(elf, &fw)) { printf("cannot read %s\n", elf); return 2; }
  avr = avr_make_mcu_by_name("atmega2560");
  avr_init(avr);
  avr->frequency = 16000000;
  avr->vcc = avr->avcc = avr->aref = 5000;
  avr_load_firmware(avr, &fw);
  avr->log = 0;
  for (auto &m : MEGA) {
    auto e = endpoint_net.find("mega:" + m.first);
    if (e == endpoint_net.end()) continue;
    MPin p; p.port = m.second.first; p.bit = m.second.second; p.net = e->second;
    p.irq = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(p.port), p.bit);
    mpins.push_back(p);
  }
  for (size_t i = 0; i < mpins.size(); i++) { avr_irq_register_notify(mpins[i].irq, on_avr_pin, (void *)(intptr_t)i); net_to_mpin[mpins[i].net] = (int)i; }
  on_level = [](int n, int lv) {                      // chips -> AVR input pins
    auto it = net_to_mpin.find(n);
    if (it == net_to_mpin.end()) return;
    MPin &p = mpins[it->second];
    // simavr re-applies internal pull-ups on every PORT write unless the pin has an "external" value
    avr_ioport_external_t &e = ext[p.port];
    e.name = p.port; e.mask |= 1 << p.bit;
    e.value = (e.value & ~(1 << p.bit)) | (lv << p.bit);
    avr_ioctl(avr, AVR_IOCTL_IOPORT_SET_EXTERNAL(p.port), &e);
    if (!p.out) avr_raise_irq(p.irq, lv);
  };
  for (auto &e : endpoint_net) {                      // the Mega's 5V and GND pins drive their nets
    if (e.first.rfind("mega:5V", 0) == 0) mcu_drive(e.second, 1);
    if (e.first.rfind("mega:GND", 0) == 0) mcu_drive(e.second, 0);
  }
  for (auto &p : mpins) { mcu_drive(p.net, -1); on_level(p.net, level(p.net)); }
  on_uart_tx = [](int, uint8_t b) { avr_raise_irq(uart2_in, b); };
  twi_in = avr_io_getirq(avr, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_INPUT);
  avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_OUTPUT), on_twi, nullptr);
  uint32_t fl = 0;
  avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS('0'), &fl); fl &= ~AVR_UART_FLAG_STDIO; avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS('0'), &fl);
  avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT), on_uart0, nullptr);
  uart2_in = avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('2'), UART_IRQ_INPUT);
  avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('2'), UART_IRQ_OUTPUT), on_uart2, nullptr);
  avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC0), 2467);   // battery 7.40 V through 20k/10k
  for (auto &p : pwms) { auto e = endpoint_net.find(std::string("mega:") + p.pin); if (e != endpoint_net.end()) p.net = e->second; }
  int brain = part_chip["brain"], gyro = part_chip["gyro"], qtr = part_chip["qtr"];
  set_attr(brain, "enable", 0);                       // first: Mega alone

  printf("Boot (Mega alone, brain stub silent)\n");
  run_ms(3500);
  bool boot = false;
  for (auto &l : usb_log) if (l.find("ToF 8/8 gyro ok") != std::string::npos) boot = true;
  CHECK(boot, "all 8 ToF moved to their own address and the gyro answered");
  CHECK(tele_frames > 50, "telemetry frames on Serial2: %d", tele_frames);
  CHECK(last_t.flags & 0x0A, "flags: gyro ok + all ToF ok (0x%02X)", last_t.flags);
  const int want_tof[8] = {300, 450, 450, 120, 125, 140, 135, 800};
  bool tofs = true;
  for (int i = 0; i < 8; i++) tofs &= abs(last_t.tof[i] - want_tof[i]) <= 4;
  CHECK(tofs, "ToF in telemetry: %u %u %u %u %u %u %u %u (set 300 450 450 120 125 140 135 800)", last_t.tof[0], last_t.tof[1],
        last_t.tof[2], last_t.tof[3], last_t.tof[4], last_t.tof[5], last_t.tof[6], last_t.tof[7]);
  CHECK(abs(last_t.us[0] - 600) < 15 && abs(last_t.us[1] - 900) < 20, "sonar front %u mm, back %u mm (set 600, 900)", last_t.us[0], last_t.us[1]);
  CHECK(last_t.line[3] > 2000 && last_t.line[4] > 2000 && last_t.line[0] < 400 && last_t.line[7] < 400,
        "QTR: %u %u %u %u %u %u %u %u (line under sensors 4 and 5)", last_t.line[0], last_t.line[1], last_t.line[2], last_t.line[3],
        last_t.line[4], last_t.line[5], last_t.line[6], last_t.line[7]);
  CHECK(last_t.tcsFront[2] > 2 * last_t.tcsFront[0], "front TCS3200 sees the blue piece: R %u G %u B %u C %u", last_t.tcsFront[0],
        last_t.tcsFront[1], last_t.tcsFront[2], last_t.tcsFront[3]);
  CHECK(abs((int)last_t.vbat_mv - 7400) < 60, "battery %u mV (7.40 V)", last_t.vbat_mv);
  CHECK(fabs(last_t.yaw) < 0.02, "yaw %.3f rad while still (gyro bias learned)", last_t.yaw);

  printf("Brain stub on: drive 250 mm/s straight\n");
  set_attr(brain, "v_left", 250); set_attr(brain, "v_right", 250); set_attr(brain, "enable", 1);
  set_attr(brain, "servo_left", 30);
  run_ms(1000);
  Telemetry a = last_t;
  run_ms(1000);
  Telemetry b = last_t;
  double mmPerTick = M_PI * 43.0 / 2800.0;
  double vL = (b.encL - a.encL) * mmPerTick / ((b.ms - a.ms) / 1000.0), vR = (b.encR - a.encR) * mmPerTick / ((b.ms - a.ms) / 1000.0);
  CHECK(fabs(vL - 250) < 15 && fabs(vR - 250) < 15, "wheel speed under PI control: left %.0f, right %.0f mm/s (set 250)", vL, vR);
  printf("Turn: gyro reports 90 deg/s for 1 s\n");
  float y0 = last_t.yaw;
  set_attr(gyro, "rate", 90);
  run_ms(1000);
  set_attr(gyro, "rate", 0);
  run_ms(100);
  CHECK(fabs((last_t.yaw - y0) * 57.2958 - 90) < 6, "yaw changed by %.1f deg (expected ~90)", (last_t.yaw - y0) * 57.2958);
  printf("Line moves 30 mm to the right\n");
  set_attr(qtr, "line_mm", (uint32_t)-30);
  run_ms(100);
  CHECK(last_t.line[0] > 2000 && last_t.line[6] < 400, "QTR: sensor 1 (right-most) dark: %u ... %u", last_t.line[0], last_t.line[7]);
  printf("Key K3 pressed\n");
  mcu_drive(endpoint_net["mega:49"], 0); avr_raise_irq(mpins[net_to_mpin[endpoint_net["mega:49"]]].irq, 0);
  run_ms(100);
  if (getenv("KEY_DEBUG")) printf("PINL %02X DDRL %02X PORTL %02X net level %d\n", avr->data[0x109], avr->data[0x10A], avr->data[0x10B], level(endpoint_net["mega:49"]));
  CHECK(last_t.keys == 0x04, "keys = 0x%02X (K3)", last_t.keys);
  mcu_drive(endpoint_net["mega:49"], -1); avr_raise_irq(mpins[net_to_mpin[endpoint_net["mega:49"]]].irq, 1);
  printf("Brain stub goes silent: watchdog\n");
  set_attr(brain, "enable", 0);
  run_ms(700);
  a = last_t; run_ms(300); b = last_t;
  CHECK(abs(b.encL - a.encL) < 10 && abs(b.encR - a.encR) < 10, "wheels stopped %d / %d ticks in 0.3 s after the link went quiet",
        b.encL - a.encL, b.encR - a.encR);
  printf("\n%s (%d failure%s), %d telemetry frames, I/O conflicts: %d\n", fails ? "CO-SIMULATION FAILED" : "CO-SIMULATION PASSED",
         fails, fails == 1 ? "" : "s", tele_frames, conflicts());
  return fails ? 1 : 0;
}
