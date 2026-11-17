"""
make_deck_schematic.py - writes ../../pcb_v4/guardianbot_v4_pcb_deck.kicad_sch (KiCad 7) from deck_design.py.

Every part is drawn as a box with its real pin numbers and names; connections are made with
net labels (same name = same net), so the schematic and the PCB are generated from the one
netlist in deck_design.py and always agree.
"""
import os
import uuid

import deck_design as D

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "pcb_v4")
NAME = "guardianbot_v4_pcb_deck"
ROOT = str(uuid.uuid4())

TO92 = {"Q1": {"1": "S", "2": "G", "3": "D"}, "Q2": {"1": "C", "2": "B", "3": "E"}}


def all_parts():
    """Mega + everything in D.PARTS."""
    out = [("MEGA1", "Arduino Mega 2560", "", 0, 0, 0, "F", dict(D.MEGA_NET), "Arduino Mega 2560 R3 (upside-down on the deck)")]
    for ref, val, fpid, x, y, d, side, pm, desc in D.PARTS:
        out.append((ref, val, fpid, x, y, 0, side, pm, desc))
    return out


def pin_names(ref, pads):
    if ref == "MEGA1":
        return {p: p for p in pads}
    if ref in TO92:
        return TO92[ref]
    return {p: p for p in pads}


def q(s):
    return '"%s"' % s.replace('"', "'")


FONT = "(effects (font (size 1.27 1.27)))"


def lib_symbol(sym, pins, height, width):
    """pins: list of (number, name, side, row)"""
    out = ['    (symbol %s (pin_names (offset 0.508)) (in_bom yes) (on_board yes)' % q("gb:" + sym),
           '      (property "Reference" "U" (at 0 %.2f 0) %s)' % (height / 2 + 2.54, FONT),
           '      (property "Value" %s (at 0 %.2f 0) %s)' % (q(sym), -height / 2 - 2.54, FONT),
           '      (property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
           '      (property "Datasheet" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
           '      (symbol %s (rectangle (start %.2f %.2f) (end %.2f %.2f) (stroke (width 0.254) (type default)) (fill (type background))))'
           % (q(sym + "_0_1"), -width / 2, height / 2, width / 2, -height / 2),
           '      (symbol %s' % q(sym + "_1_1")]
    for num, name, side, row in pins:
        y = height / 2 - 2.54 - row * 2.54
        if side == "L":
            x, ang = -width / 2 - 2.54, 0
        else:
            x, ang = width / 2 + 2.54, 180
        out.append('        (pin passive line (at %.2f %.2f %d) (length 2.54) (name %s %s) (number %s %s))'
                   % (x, y, ang, q(name), FONT, q(num), FONT))
    out.append("      )")
    out.append("    )")
    return "\n".join(out)


def build():
    libs, body = [], []
    col_x = [35 + 80 * k for k in range(7)]
    col_y = [30.0] * len(col_x)
    ci = 0
    order = sorted(all_parts(), key=lambda p: (0 if p[0] == "MEGA1" else 1 if p[0].startswith("J") else 2, len(p[0]), p[0]))
    for ref, val, fpid, x, y, rot, side, pinmap, desc in order:
        pads = dict(sorted(pinmap.items(), key=lambda kv: (len(kv[0]), kv[0])))
        names = pin_names(ref, pads)
        nums = [p for p in pads]
        if ref == "MEGA1":
            half = (len(nums) + 1) // 2
            left, right = nums[:half], nums[half:]
        elif len(nums) <= 3:
            left, right = nums, []
        else:
            half = (len(nums) + 1) // 2
            left, right = nums[:half], nums[half:]
        rows = max(len(left), len(right))
        height = rows * 2.54 + 2.54
        width = 20.32 if ref == "MEGA1" else 12.7
        pins = [(n, names.get(n, n), "L", i) for i, n in enumerate(left)] + \
               [(n, names.get(n, n), "R", i) for i, n in enumerate(right)]
        sym = "%s_%s" % (ref, "".join(ch for ch in val if ch.isalnum()))[:28]
        libs.append(lib_symbol(sym, pins, height, width))
        # place
        if col_y[ci] + height + 15 > 400:
            ci += 1
        X = col_x[ci]
        Y = col_y[ci] + height / 2 + 5
        col_y[ci] = Y + height / 2 + 10
        X, Y = round(X / 2.54) * 2.54, round(Y / 2.54) * 2.54
        su = str(uuid.uuid4())
        props = [
            '    (property "Reference" %s (at %.2f %.2f 0) %s)' % (q(ref), X, Y - height / 2 - 1.5, FONT),
            '    (property "Value" %s (at %.2f %.2f 0) %s)' % (q(val), X, Y + height / 2 + 1.5, FONT),
            '    (property "Footprint" %s (at %.2f %.2f 0) (effects (font (size 1.27 1.27)) hide))' % (q(fpid), X, Y),
            '    (property "Datasheet" "" (at %.2f %.2f 0) (effects (font (size 1.27 1.27)) hide))' % (X, Y),
            '    (property "Description" %s (at %.2f %.2f 0) (effects (font (size 1.27 1.27)) hide))' % (q(desc), X, Y),
        ]
        pin_uuids = ["    (pin %s (uuid %s))" % (q(n), uuid.uuid4()) for n in nums]
        body.append('  (symbol (lib_id %s) (at %.2f %.2f 0) (unit 1) (in_bom yes) (on_board yes) (dnp no) (uuid %s)\n%s\n%s\n'
                    '    (instances (project %s (path "/%s" (reference %s) (unit 1))))\n  )'
                    % (q("gb:" + sym), X, Y, su, "\n".join(props), "\n".join(pin_uuids), q(NAME), ROOT, q(ref)))
        # labels / no-connects at the pin ends
        for num, name, sd, row in pins:
            py = height / 2 - 2.54 - row * 2.54
            px = -width / 2 - 2.54 if sd == "L" else width / 2 + 2.54
            ax, ay = X + px, Y - py
            net = pads[num]
            if net:
                just = "right" if sd == "L" else "left"
                ang = 180 if sd == "L" else 0
                body.append('  (label %s (at %.2f %.2f %d) (fields_autoplaced) (effects (font (size 1.27 1.27)) (justify %s bottom)) (uuid %s))'
                            % (q(net), ax, ay, ang, just, uuid.uuid4()))
            else:
                body.append('  (no_connect (at %.2f %.2f) (uuid %s))' % (ax, ay, uuid.uuid4()))
    head = ('(kicad_sch (version 20230121) (generator eeschema)\n  (uuid %s)\n  (paper "A2")\n'
            '  (title_block (title "GuardianBot v4 PCB mid deck") (date "2026-11-17") (rev "A") '
            '(company "PeraBots 2026 - University category") (comment 1 "Generated by make_deck_schematic.py from deck_design.py")'
            ' (comment 2 "Nets are connected by label name"))\n'
            % ROOT)
    txt = head + "  (lib_symbols\n" + "\n".join(libs) + "\n  )\n" + "\n".join(body) + \
        '\n  (sheet_instances (path "/" (page "1")))\n)\n'
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, NAME + ".kicad_sch")
    open(p, "w").write(txt)
    print("wrote", p)


if __name__ == "__main__":
    build()
