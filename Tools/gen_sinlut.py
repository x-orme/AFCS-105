#!/usr/bin/env python3
"""Q1.30 sin LUT 생성 -> Core/Src/fcs_lut_data.inc (1025 엔트리, 4KB)."""
import math

N = 1024
FRAC_BITS = 30
SCALE = 1 << FRAC_BITS  # 1073741824

out = "/* Q1.30 sin LUT — gen_sinlut.py 자동 생성, 수동 편집 금지 */\n"
out += f"#define LUT_FRAC_BITS  {FRAC_BITS}\n"
out += f"#define LUT_SCALE      0x{SCALE:08X}  /* 2^{FRAC_BITS} */\n\n"
out += f"static const int32_t SIN_LUT[{N + 1}] = {{\n"

for i in range(N + 1):
    x = (math.pi / 2.0) * i / N
    val = round(math.sin(x) * SCALE)
    # sin(pi/2) = 1.0 -> 정확히 2^30
    if i == N:
        val = SCALE

    if i % 8 == 0:
        out += "  "
    out += f"0x{val & 0xFFFFFFFF:08X},"
    if i % 8 == 7 or i == N:
        out += "\n"
    else:
        out += " "

out += "};\n"

with open("../Core/Src/fcs_lut_data.inc", "w") as f:
    f.write(out)

print(f"생성 완료: {N+1}개 엔트리, {(N+1)*4} 바이트")

# 정밀도 검증
max_err = 0
for i in range(N + 1):
    x = (math.pi / 2.0) * i / N
    ref = math.sin(x)
    val = round(ref * SCALE)
    if i == N:
        val = SCALE
    approx = val / SCALE
    err = abs(ref - approx)
    if err > max_err:
        max_err = err

print(f"Q1.30 양자화 최대 오차: {max_err:.2e} (≈ {max_err * 6378137:.4f} m at equator)")
