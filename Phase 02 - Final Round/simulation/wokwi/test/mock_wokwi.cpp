#include "mock_wokwi.h"
extern "C" {
#include "../chips/wokwi-api.h"
}
#include <algorithm>
#include <cstdio>

namespace {
struct Watch { void *ud; uint32_t edge; void (*cb)(void *, pin_t, uint32_t); bool on; };
struct Pin { int net; int chip; uint32_t mode = INPUT; uint32_t val = 0; Watch w{}; };
struct Net { int mcu = -1; bool mcu_pu = false; int level = 0; std::vector<int> pins; };
struct Timer { void *ud; void (*cb)(void *); uint64_t due = 0, period = 0; bool on = false; };
struct I2C { i2c_config_t cfg; };
struct Uart { uart_config_t cfg; std::vector<uint8_t> out; };

std::vector<Pin> pins;
std::vector<Net> nets;
std::map<std::string, int> net_ids;
std::vector<Timer> timers;
std::vector<I2C> i2cs;
std::vector<Uart> uarts;
std::vector<uint32_t> attrs;
std::vector<std::map<std::string, int>> chip_attr;      // per chip: name -> attr id
std::map<std::string, int> cur_nets;
std::map<std::string, uint32_t> cur_attrs;
int cur_chip = -1, active_i2c = -1, n_conflicts = 0;
uint64_t t_now = 0;

bool drives(uint32_t m) { return m == OUTPUT || m == OUTPUT_LOW || m == OUTPUT_HIGH; }

void resolve(int n) {
  Net &N = nets[n];
  int lv = -1, drivers = 0;
  for (int p : N.pins)
    if (drives(pins[p].mode)) { int v = pins[p].val ? 1 : 0; if (lv >= 0 && lv != v) n_conflicts++; lv = v; drivers++; }
  if (N.mcu >= 0) { if (drivers && lv != N.mcu) n_conflicts++; if (lv < 0) lv = N.mcu; }
  if (lv < 0) {
    lv = N.mcu_pu ? 1 : 0;
    for (int p : N.pins) if (pins[p].mode == INPUT_PULLUP) lv = 1;
  }
  if (lv == N.level) return;
  N.level = lv;
  if (mock::on_level) mock::on_level(n, lv);
  for (int p : N.pins) {
    Watch &w = pins[p].w;
    if (!w.on) continue;
    if ((lv && (w.edge & RISING)) || (!lv && (w.edge & FALLING))) w.cb(w.ud, p, lv);
  }
}
}

namespace mock {
std::function<void(int, int)> on_level;
std::function<void(int, uint8_t)> on_uart_tx;
void mcu_pullup(int n, bool on) { nets[n].mcu_pu = on; resolve(n); }
int net_count() { return (int)nets.size(); }
int net(const std::string &name) {
  auto it = net_ids.find(name);
  if (it != net_ids.end()) return it->second;
  nets.push_back(Net());
  return net_ids[name] = (int)nets.size() - 1;
}
void begin_chip(void (*init)(void), const std::map<std::string, int> &n, const std::map<std::string, uint32_t> &a) {
  cur_nets = n; cur_attrs = a; cur_chip = (int)chip_attr.size(); chip_attr.push_back({});
  init();
  cur_chip = -1;
}
void mcu_drive(int n, int lv) { nets[n].mcu = lv; resolve(n); }
int level(int n) { return nets[n].level; }
void set_attr(int chip, const std::string &name, uint32_t v) { attrs[chip_attr[chip].at(name)] = v; }
int chip_count() { return (int)chip_attr.size(); }
uint64_t now() { return t_now; }
int conflicts() { return n_conflicts; }
void run_for_ns(uint64_t ns) {
  uint64_t end = t_now + ns;
  for (;;) {
    int best = -1;
    for (size_t i = 0; i < timers.size(); i++)
      if (timers[i].on && timers[i].due <= end && (best < 0 || timers[i].due < timers[best].due)) best = (int)i;
    if (best < 0) break;
    Timer &t = timers[best];
    t_now = t.due;
    if (t.period) t.due += t.period; else t.on = false;
    void (*cb)(void *) = t.cb;
    void *ud = t.ud;
    cb(ud);
  }
  t_now = end;
}
bool i2c_start(uint8_t addr, bool read) {
  active_i2c = -1;
  for (size_t i = 0; i < i2cs.size(); i++) {
    i2c_config_t &c = i2cs[i].cfg;
    if (c.address && c.address != addr) continue;
    if (c.connect && c.connect(c.user_data, addr, read)) { active_i2c = (int)i; return true; }
  }
  return false;
}
bool i2c_write(uint8_t b) { if (active_i2c < 0) return false; auto &c = i2cs[active_i2c].cfg; return c.write ? c.write(c.user_data, b) : true; }
uint8_t i2c_read() { if (active_i2c < 0) return 0xFF; auto &c = i2cs[active_i2c].cfg; return c.read ? c.read(c.user_data) : 0xFF; }
void i2c_stop() { if (active_i2c >= 0 && i2cs[active_i2c].cfg.disconnect) i2cs[active_i2c].cfg.disconnect(i2cs[active_i2c].cfg.user_data); active_i2c = -1; }
std::vector<uint8_t> &uart_captured(int dev) { return uarts[dev].out; }
void uart_inject(int dev, const uint8_t *d, int n) { for (int i = 0; i < n; i++) uarts[dev].cfg.rx_data(uarts[dev].cfg.user_data, d[i]); }
}

using namespace mock;
extern "C" {
pin_t pin_init(const char *name, uint32_t mode) {
  auto it = cur_nets.find(name);
  int n = it != cur_nets.end() ? it->second : net("nc_" + std::to_string(cur_chip) + "_" + name);
  Pin p; p.net = n; p.chip = cur_chip; p.mode = mode; p.val = mode == OUTPUT_HIGH;
  pins.push_back(p);
  nets[n].pins.push_back((int)pins.size() - 1);
  resolve(n);
  return (pin_t)pins.size() - 1;
}
uint32_t pin_read(pin_t p) { return nets[pins[p].net].level; }
void pin_write(pin_t p, uint32_t v) { pins[p].val = v; resolve(pins[p].net); }
bool pin_watch(pin_t p, const pin_watch_config_t *c) { pins[p].w = {c->user_data, c->edge, c->pin_change, true}; return true; }
void pin_watch_stop(pin_t p) { pins[p].w.on = false; }
void pin_mode(pin_t p, uint32_t m) {
  pins[p].mode = m;
  if (m == OUTPUT_LOW) pins[p].val = 0;
  if (m == OUTPUT_HIGH) pins[p].val = 1;
  resolve(pins[p].net);
}
float pin_adc_read(pin_t p) { return nets[pins[p].net].level ? 5.0f : 0.0f; }
float pin_dac_write(pin_t, float v) { return v; }
uint32_t attr_init(const char *name, uint32_t def) {
  auto it = cur_attrs.find(name);
  attrs.push_back(it != cur_attrs.end() ? it->second : def);
  chip_attr[cur_chip][name] = (int)attrs.size() - 1;
  return (uint32_t)attrs.size() - 1;
}
uint32_t attr_read(uint32_t id) { return attrs[id]; }
float attr_read_float(uint32_t id) { return (float)attrs[id]; }
i2c_dev_t i2c_init(const i2c_config_t *c) { i2cs.push_back({*c}); return (i2c_dev_t)i2cs.size() - 1; }
uart_dev_t uart_init(const uart_config_t *c) { uarts.push_back({*c, {}}); return (uart_dev_t)uarts.size() - 1; }
bool uart_write(uart_dev_t u, uint8_t *b, uint32_t n) {
  uarts[u].out.insert(uarts[u].out.end(), b, b + n);
  if (mock::on_uart_tx) for (uint32_t i = 0; i < n; i++) mock::on_uart_tx((int)u, b[i]);
  // finish after the real transfer time (10 bits per byte), through a one-shot timer
  static std::vector<int> done_timer;
  if (done_timer.size() <= u) done_timer.resize(u + 1, -1);
  if (done_timer[u] < 0) {
    timers.push_back({&uarts[u], [](void *ud) { auto *x = (Uart *)ud; if (x->cfg.write_done) x->cfg.write_done(x->cfg.user_data); }, 0, 0, false});
    done_timer[u] = (int)timers.size() - 1;
  }
  Timer &t = timers[done_timer[u]];
  t.due = t_now + (uint64_t)(n * 10.0 * 1e9 / uarts[u].cfg.baud_rate);
  t.on = true;
  return true;
}
timer_t timer_init(const timer_config_t *c) { timers.push_back({c->user_data, c->callback, 0, 0, false}); return (timer_t)timers.size() - 1; }
void timer_start(const timer_t t, uint32_t us, bool rep) { timers[t].due = t_now + us * 1000ull; timers[t].period = rep ? us * 1000ull : 0; timers[t].on = true; }
void timer_start_ns_d(const timer_t t, double ns, bool rep) { timers[t].due = t_now + (uint64_t)ns; timers[t].period = rep ? (uint64_t)ns : 0; timers[t].on = true; }
void timer_stop(const timer_t t) { timers[t].on = false; }
double get_sim_nanos_d(void) { return (double)t_now; }
spi_dev_t spi_init(const spi_config_t *) { return 0; }
void spi_start(const spi_dev_t, uint8_t *, uint32_t) {}
void spi_stop(const spi_dev_t) {}
buffer_t framebuffer_init(uint32_t *w, uint32_t *h) { *w = *h = 0; return 0; }
void buffer_read(buffer_t, uint32_t, uint8_t *, uint32_t) {}
void buffer_write(buffer_t, uint32_t, uint8_t *, uint32_t) {}
}
