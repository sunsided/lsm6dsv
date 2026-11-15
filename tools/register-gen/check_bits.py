"""Compare the generated bit layouts against the per-register bit tables of DS13510.

Usage: python3 -I check_bits.py model.json ds16x.txt [16x]
       python3 -I check_bits.py model.json dsv.txt dsv

Compares reserved bits and field boundaries for every register table the datasheet has,
independent of field names. Exits non-zero on an unexpected difference.
"""
import json, re, sys

model = json.load(open(sys.argv[1]))
lines = open(sys.argv[2]).read().split("\n")

def parse_table(start):
    """Return 8 bit names (MSB first) for the table starting after line index `start`."""
    rows = []
    for k in range(start + 1, min(start + 12, len(lines))):
        l = lines[k]
        if re.search(r"Table \d+\.|^\s*\d\.\s|^\s*DS13510|register description", l):
            break
        if l.strip():
            rows.append(l)
    tokens = []
    for r_i, r in enumerate(rows):
        for m in re.finditer(r"\S+", r):
            tokens.append([m.start(), m.end(), m.group(), r_i])
    # cluster by horizontal overlap of token centers
    tokens.sort(key=lambda t: (t[0] + t[1]) / 2)
    cols = []
    for t in tokens:
        c = (t[0] + t[1]) / 2
        if cols and c - cols[-1]["c"] < 6:
            cols[-1]["t"].append(t)
            cols[-1]["c"] = (cols[-1]["c"] + c) / 2
        else:
            cols.append({"c": c, "t": [t]})
    names = ["".join(x[2] for x in sorted(col["t"], key=lambda x: x[3])) for col in cols]
    return names

def norm(n):
    n = re.sub(r"\(\d\)$", "", n)
    if re.fullmatch(r"[01-]", n):
        return None
    n = n.upper()
    n = re.sub(r"_\d+$", "", n)
    n = re.sub(r"(?<=[A-Z])\d+$", "", n)
    return n

def expected(fields):
    bits = []
    for name, width in fields:
        bits += [None if name.startswith("not_used") else name.upper()] * width
    return list(reversed(bits))  # MSB first

GROUP_OF_SECTION = {"9": "main", "11": "if2", "13": "embedded", "15": None, "17": "sensor_hub"}
heading_group = {}
cur = None
for i, l in enumerate(lines):
    m = re.match(r"\s*(\d+)\s{3,}[A-Z]", l)
    if m and m.group(1) in GROUP_OF_SECTION:
        cur = m.group(1)
    heading_group[i] = cur

# Differences that are only naming in the datasheet tables.
KNOWN = {
    "MLC2_SRC": "datasheet typo MLCS2_SRC_2",
    "FSM_EXT_MATRIX_XX_L": "datasheet typo FSM_EXT_XX_0",
    "FSM_EXT_MATRIX_XX_H": "datasheet typo FSM_EXT_XX_9/_8",
}

regs = {}
chip = sys.argv[3] if len(sys.argv) > 3 else "16x"
for chip in (chip,):
    for g, rs in model[chip].items():
        for r in rs:
            regs.setdefault(r["name"], []).append((g, r["fields"]))

checked = mismatched = 0
for i, l in enumerate(lines):
    m = re.search(r"Table \d+\.\s+([A-Z][A-Z0-9_]+) register\s*$", l)
    if not m:
        continue
    name = m.group(1)
    cands = regs.get(name) or regs.get(name.replace("_REG", ""))
    if not cands:
        continue
    got = [norm(n) for n in parse_table(i)]
    if len(got) != 8:
        print(f"SKIP {name}: {len(got)} columns {got}")
        continue
    def shape(bits):
        # reserved mask plus field boundaries, independent of names
        out = []
        for k, b in enumerate(bits):
            if b is None:
                out.append("-")
            elif k > 0 and bits[k - 1] == b:
                out.append("=")
            else:
                out.append("F")
        return "".join(out)
    def plain(bits):
        return [None if b is None else re.sub(r"[\d_]", "", b) for b in bits]
    ok_any = any(shape(expected(f)) == shape(got) or plain(expected(f)) == plain(got) for _, f in cands)
    if not ok_any and name in KNOWN:
        ok_any = True
    checked += 1
    if not ok_any:
        mismatched += 1
        print(f"DIFF {name}:\n   ds  {got}\n   hdr {[expected(f) for _, f in cands]}")
print("checked", checked, "mismatched", mismatched)
sys.exit(1 if mismatched else 0)
