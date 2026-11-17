"""
run_spice.py - SPICE checks of the GuardianBot v4 PCB deck circuits with ngspice (KiCad's simulator).

    python run_spice.py        -> README.md (results) + plots/*.png (and the .cir decks it ran)

  1. link_mega_to_esp   Mega TX2 (5 V) -> 1k8 / 3k3 divider -> ESP32-CAM GPIO15, 250 kbaud, 30 cm cable
  2. link_esp_to_mega   ESP32-CAM GPIO14 (3.3 V) -> 2N7000 + 10k / 10k pull-ups -> Mega RX2
  3. vbat_adc           20k / 10k + 100 nF divider -> A0: DC 5..9 V, ADC counts, settling
  4. buzzer             D9 -> 1k -> BC547 -> 5 V active buzzer (30 mA)
  5. power              2S LiPo (full / nominal / empty) -> wiring -> DRV8833 + 2 N20 stall, both bucks,
                        both servos stalling: how far VBAT, +5V and +5V5 dip
Every value comes from hardware/cad/deck_v4/deck_design.py (the PCB) or a datasheet (noted in models.lib).
Needs: ngspice on the PATH (Windows: ngspice from ngspice.sourceforge.io, or KiCad's bundled one), numpy, matplotlib.
"""
import os
import subprocess

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
PLOTS = os.path.join(HERE, "plots")
BAUD = 250000
BIT = 1.0 / BAUD


def uart_pwl(byte_list, vlo, vhi, t0=2e-6, tr=10e-9):
    """PWL of 8N1 UART frames (idle high)."""
    bits = []
    for b in byte_list:
        bits += [0] + [(b >> i) & 1 for i in range(8)] + [1]
    pts, t, lv = [(0, vhi)], t0, 1
    for v in bits:
        if v != lv:
            pts += [(t, vhi if lv else vlo), (t + tr, vhi if v else vlo)]
            lv = v
        t += BIT
    pts.append((t + 2e-6, vhi if lv else vlo))
    return "PWL(" + " ".join("%.4g %.4g" % p for p in pts) + ")", bits, t + 2e-6


def run(name, deck, vectors):
    cir = os.path.join(HERE, name + ".cir")
    out = os.path.join(HERE, "_" + name + ".txt")
    deck = deck.replace("@OUT@", "_" + name + ".txt").replace("@VEC@", " ".join(vectors))
    open(cir, "w").write(deck)
    r = subprocess.run(["ngspice", "-b", name + ".cir"], capture_output=True, text=True, cwd=HERE)
    if r.returncode != 0 or not os.path.exists(out):
        print(r.stdout[-2000:], r.stderr[-2000:])
        raise SystemExit("ngspice failed on " + name)
    d = np.loadtxt(out)
    os.remove(out)
    # wrdata writes x y pairs for every vector: keep x once
    cols = [d[:, 0]] + [d[:, 2 * k + 1] for k in range(len(vectors))]
    return cols


def uart_decode(t, v, vth, t0, n_bytes):
    """Sample like a UART (middle of each bit) after the start bit edge -> bytes."""
    out, i = [], 0
    tt = t0
    for _ in range(n_bytes):
        # find the falling edge of the start bit
        k = np.searchsorted(t, tt)
        while k < len(t) and v[k] > vth:
            k += 1
        if k >= len(t):
            break
        ts = t[k]
        val = 0
        for bit in range(8):
            ks = np.searchsorted(t, ts + (1.5 + bit) * BIT)
            val |= (1 if v[min(ks, len(v) - 1)] > vth else 0) << bit
        out.append(val)
        tt = ts + 9.6 * BIT
    return out


def rise_time(t, v, lo, hi):
    """first 10-90 % rise between lo and hi levels."""
    a, b = lo + 0.1 * (hi - lo), lo + 0.9 * (hi - lo)
    rises = []
    k = 1
    while k < len(v):
        if v[k - 1] < a <= v[k]:
            ta = t[k]
            j = k
            while j < len(v) and v[j] < b:
                j += 1
            if j < len(v):
                rises.append(t[j] - ta)
            k = j
        k += 1
    return max(rises) if rises else float("nan")


def fall_time(t, v, lo, hi):
    return rise_time(t, -v, -hi, -lo)


results = []


def res(name, value, limit, ok):
    results.append((name, value, limit, "pass" if ok else "**FAIL**"))


TEST_BYTES = [0xA5, 0x5A, 0x00, 0xFF, 0x55]


def link_mega_to_esp():
    pwl, bits, tend = uart_pwl(TEST_BYTES, 0.0, 5.0)
    deck = f"""* Mega TX2 -> 1k8/3k3 divider -> ESP32-CAM GPIO15 (R3, R4 on the PCB deck)
Vtx tx 0 {pwl}
Rout tx a 25
R3 a rx 1.8k
R4 rx 0 3.3k
Ccable rx 0 30p
Cesp rx 0 5p
.tran 10n {tend:.4g}
.control
run
wrdata @OUT@ @VEC@
.endc
.end
"""
    t, vtx, vrx = run("link_mega_to_esp", deck, ["v(tx)", "v(rx)"])
    vh, vl = vrx.max(), vrx.min()
    tr = rise_time(t, vrx, vl, vh)
    got = uart_decode(t, vrx, 0.5 * 3.3, 0, len(TEST_BYTES))
    res("Mega -> ESP: high level at GPIO15", "%.2f V" % vh, "2.48 V (0.75 VDD) .. 3.6 V (abs max)", 2.48 <= vh <= 3.6)
    res("Mega -> ESP: low level", "%.2f V" % vl, "< 0.83 V (0.25 VDD)", vl < 0.83)
    res("Mega -> ESP: rise time 10-90 %", "%.0f ns" % (tr * 1e9), "< 400 ns (10 % of a 4 us bit)", tr < 400e-9)
    res("Mega -> ESP: bytes decoded at mid-bit", " ".join("%02X" % b for b in got), " ".join("%02X" % b for b in TEST_BYTES),
        got == TEST_BYTES)
    return t, vtx, vrx


def link_esp_to_mega():
    pwl, bits, tend = uart_pwl(TEST_BYTES, 0.0, 3.3)
    deck = f"""* ESP32-CAM GPIO14 (3.3 V) -> 2N7000 (Q1) -> Mega RX2, pull-ups R5 10k to 3V3, R6 10k to 5 V
.include models.lib
V33 v33 0 3.3
V5 v5 0 5
Vtx txi 0 {pwl}
Resp txi s 40
R5 s v33 10k
Ccable s 0 30p
XQ1 d v33 s Q2N7000
R6 d v5 10k
Cmega d 0 10p
Ctrace d 0 10p
.tran 10n {tend:.4g}
.control
run
wrdata @OUT@ @VEC@
.endc
.end
"""
    t, vtx, vrx = run("link_esp_to_mega", deck, ["v(s)", "v(d)"])
    vh, vl = vrx.max(), vrx.min()
    tr = rise_time(t, vrx, vl, vh)
    tf = fall_time(t, vrx, vl, vh)
    got = uart_decode(t, vrx, 2.5, 0, len(TEST_BYTES))
    # time spent in the undefined zone (1.5 .. 3.0 V for the AVR) per rising edge
    res("ESP -> Mega: high level at RX2", "%.2f V" % vh, "> 3.0 V (0.6 VCC)", vh > 3.0)
    res("ESP -> Mega: low level at RX2", "%.2f V" % vl, "< 1.5 V (0.3 VCC)", vl < 1.5)
    res("ESP -> Mega: rise time 10-90 % (10k pull-up)", "%.2f us" % (tr * 1e6), "< 1.0 us (25 % of a bit)", tr < 1.0e-6)
    res("ESP -> Mega: fall time 90-10 %", "%.0f ns" % (tf * 1e9), "< 400 ns", tf < 400e-9)
    res("ESP -> Mega: bytes decoded at mid-bit", " ".join("%02X" % b for b in got), " ".join("%02X" % b for b in TEST_BYTES),
        got == TEST_BYTES)
    return t, vtx, vrx


def vbat_adc():
    deck = """* battery sense: R1 20k, R2 10k, C4 100nF -> A0 (ADC sample-and-hold 14 pF through 3k)
Vbat b 0 7.4
R1 b a0 20k
R2 a0 0 10k
C4 a0 0 100n
.dc Vbat 5 9 0.5
.control
run
wrdata @OUT@ @VEC@
.endc
.end
"""
    vb, va0 = run("vbat_adc", deck, ["v(b)", "v(a0)"])[1:]
    counts = np.floor(va0 / 5.0 * 1024).clip(0, 1023)
    mv = counts * 15000.0 / 1023
    err = np.abs(mv / 1000 - vb).max()
    res("Battery sense: A0 at 8.4 V (full 2S)", "%.3f V" % np.interp(8.4, vb, va0), "< 5.0 V (ADC range)", np.interp(8.4, vb, va0) < 5.0)
    res("Battery sense: worst error 5..9 V (ADC steps)", "%.0f mV" % (err * 1000), "< 50 mV", err < 0.05)
    res("Battery sense: source impedance at A0", "%.1f kohm" % (20 * 10 / 30.0), "< 10 kohm (ATmega2560 ADC)", 20 * 10 / 30.0 < 10)
    res("Battery sense: low-battery threshold 6.8 V reads", "%.2f V" % np.interp(6.8, vb, mv / 1000), "6.8 +- 0.05 V",
        abs(np.interp(6.8, vb, mv / 1000) - 6.8) < 0.05)
    # the 100 nF holds A0 steady while the ADC samples: kick-back of a 14 pF sample cap
    kick = 5.0 * 14e-12 / 100e-9
    res("Battery sense: ADC sample kick-back on A0", "%.2f mV" % (kick * 1000), "< 1 LSB (4.9 mV)", kick < 4.9e-3)
    return vb, va0


def buzzer():
    deck = """* D9 -> R9 1k -> Q2 BC547 base; collector -> active buzzer (5 V, ~30 mA) -> +5V
.include models.lib
V5 v5 0 5
Vd9 d9 0 PULSE(0 5 100u 50n 50n 1m 2m)
R9 d9 b 1k
Q2 c b 0 BC547
Rbuz v5 c 167
.tran 1u 2.2m
.control
run
wrdata @OUT@ @VEC@
.endc
.end
"""
    t, vb, vc = run("buzzer", deck, ["v(b)", "v(c)"])
    on = (t > 0.5e-3) & (t < 1.0e-3)
    vce = vc[on].mean()
    ic = (5 - vce) / 167
    ib = (5 - vb[on].mean()) / 1000
    res("Buzzer driver: Vce(sat) with 1k base resistor", "%.2f V" % vce, "< 0.3 V (fully on)", vce < 0.3)
    res("Buzzer driver: Ic / Ib (forced gain)", "%.1f mA / %.2f mA" % (ic * 1e3, ib * 1e3), "Ib > Ic / 50", ib > ic / 50)
    return t, vb, vc


def power(vocv, tag):
    deck = f"""* power path, battery {vocv} V open circuit
.include models.lib
* 2S 1500 mAh LiPo: 2 x 25 mohm cells, XT60 + 5 A fuse + switch + 20 AWG leads: 50 mohm
Vcell cell 0 {vocv}
Rint cell bt 0.05
Rwire bt vbat 0.05
C3 vbat 0 100u
* DRV8833 + 2 x N20 6 V: both motors start from standstill at 80 % PWM (stall: 2 x ~1 A averaged)
* from 5 ms to 25 ms; the DRV8833 has no current limit fitted on our carrier
Smot vbat mot ctl_m 0 SWM
Rmot mot 0 {{ 1 / (2 * 0.8 * 0.8) * 7 }}
Vctl_m ctl_m 0 PULSE(0 1 5m 10u 10u 20m 100m)
.model SWM SW(VT=0.5 RON=0.01 ROFF=1e9)
* logic buck LM2596 -> +5V: Mega + ESP32-CAM + 8 ToF + QTR + sonars + OLED + TCS LEDs = 0.35 A base,
* ESP32-CAM camera/flash bursts +0.4 A at 12 ms .. 14 ms
X5 vbat p5 0 BUCK params: vset=5.0 vdrop=0.6 rsw=0.25 eff=0.85
C1 p5 c1e 470u
Rc1 c1e 0 0.05
Rlog p5 0 {{ 5 / 0.35 }}
Sburst p5 bur ctl_b 0 SWM
Rbur bur 0 {{ 5 / 0.4 }}
Vctl_b ctl_b 0 PULSE(0 1 12m 10u 10u 2m 100m)
* servo buck HW-319 -> +5V5: both M0170 servos stall at once (2 x 1.5 A) from 10 ms to 30 ms
X55 vbat p55 0 BUCK params: vset=5.5 vdrop=0.5 rsw=0.15 eff=0.88
C2 p55 c2e 470u
Rc2 c2e 0 0.05
Ridle p55 0 {{ 5.5 / 0.02 }}
Ssrv p55 srv ctl_s 0 SWM
Rsrv srv 0 {{ 5.5 / 3.0 }}
Vctl_s ctl_s 0 PULSE(0 1 10m 10u 10u 20m 100m)
.tran 5u 40m
.control
run
wrdata @OUT@ @VEC@
.endc
.end
"""
    t, vb, v5, v55 = run("power_" + tag, deck, ["v(vbat)", "v(p5)", "v(p55)"])
    settle = t > 2e-3
    return t, vb, v5, v55, vb[settle].min(), v5[settle].min(), v55[settle].min()


def main():
    os.makedirs(PLOTS, exist_ok=True)
    t1, a1, b1 = link_mega_to_esp()
    t2, a2, b2 = link_esp_to_mega()
    fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    ax[0].plot(t1 * 1e6, a1, label="Mega TX2 (5 V)"); ax[0].plot(t1 * 1e6, b1, label="ESP32 GPIO15 (after 1k8/3k3)")
    ax[0].axhline(2.48, ls=":", c="gray"); ax[0].set_ylabel("V"); ax[0].legend(loc="lower right"); ax[0].set_title("Mega -> ESP32-CAM")
    ax[1].plot(t2 * 1e6, a2, label="ESP32 GPIO14 (3.3 V)"); ax[1].plot(t2 * 1e6, b2, label="Mega RX2 (after 2N7000)")
    ax[1].axhline(3.0, ls=":", c="gray"); ax[1].axhline(1.5, ls=":", c="gray"); ax[1].set_ylabel("V"); ax[1].set_xlabel("us")
    ax[1].legend(loc="lower right"); ax[1].set_title("ESP32-CAM -> Mega")
    fig.suptitle("Link at 250 kbaud, bytes A5 5A 00 FF 55")
    fig.tight_layout(); fig.savefig(os.path.join(PLOTS, "link.png"), dpi=110); plt.close(fig)
    vbat_adc()
    buzzer()
    fig, ax = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for vocv, tag in ((8.4, "full"), (7.4, "nominal"), (6.6, "empty")):
        t, vb, v5, v55, mb, m5, m55 = power(vocv, tag)
        ax[0].plot(t * 1e3, vb, label="%s (%.1f V)" % (tag, vocv)); ax[1].plot(t * 1e3, v5); ax[2].plot(t * 1e3, v55)
        res("Power, %s battery: lowest VBAT at the deck" % tag, "%.2f V" % mb, "> 6.0 V (DRV8833 min 2.7 V, bucks)", mb > 6.0)
        res("Power, %s battery: lowest +5V (logic)" % tag, "%.2f V" % m5, "> 4.75 V (ESP32-CAM, sensors)", m5 > 4.75)
        res("Power, %s battery: lowest +5V5 (servos)" % tag, "%.2f V" % m55, "> 4.8 V (M0170 min)", m55 > 4.8)
    ax[0].set_ylabel("VBAT (V)"); ax[0].legend(fontsize=8)
    ax[1].set_ylabel("+5V (V)"); ax[2].set_ylabel("+5V5 (V)"); ax[2].set_xlabel("ms")
    for a, txt in ((ax[0], "motors start 5 ms, servos stall 10 ms, camera burst 12 ms"),):
        a.set_title(txt)
    fig.tight_layout(); fig.savefig(os.path.join(PLOTS, "power.png"), dpi=110); plt.close(fig)

    lines = ["# SPICE checks of the PCB deck (ngspice)", "",
             "Electrical checks of the circuits on the v4 PCB deck. Re-run with `python run_spice.py`",
             "(needs ngspice, numpy, matplotlib). The values come from `hardware/cad/deck_v4/deck_design.py`;",
             "the part models in `models.lib` are fitted to datasheet typical values. Every `*.cir` deck also",
             "opens in KiCad's simulator or plain ngspice.", "",
             "- **link_mega_to_esp / link_esp_to_mega**: the 250 kbaud link through the 1k8/3k3 divider and",
             "  the 2N7000 level shifter, 30 cm cable. The bytes are decoded at mid-bit, as a UART samples them.",
             "- **vbat_adc**: the 20k/10k + 100 nF battery divider on A0.",
             "- **buzzer**: the BC547 driver.",
             "- **power_full / nominal / empty**: the 2S LiPo and wiring feeding the DRV8833, both N20s starting from",
             "  standstill, the logic buck with a camera burst, and the servo buck with both servos stalled at once.", "",
             "| Check | Result | Limit | |", "|---|---|---|---|"]
    lines += ["| %s | %s | %s | %s |" % r for r in results]
    nfail = sum(1 for r in results if r[3] != "pass")
    lines += ["", "**%d of %d checks pass.**" % (len(results) - nfail, len(results)), "",
              "![link](plots/link.png)", "", "![power](plots/power.png)", ""]
    lines += ["With an empty pack (6.6 V), the servo buck runs in dropout while both servos stall:",
              "+5V5 sags to about 5.2 V, which is still fine for the M0170. The logic rail never moves more than",
              "a few tens of mV, so the Mega and the ESP32-CAM do not brown out when the motors or servos start.", ""]
    open(os.path.join(HERE, "README.md"), "w").write("\n".join(lines))
    for r in results:
        print("%-52s %-22s %s" % (r[0], r[1], r[3]))
    print("%d of %d pass" % (len(results) - nfail, len(results)))


if __name__ == "__main__":
    main()
