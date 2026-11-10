#!/usr/bin/env python3
"""shot.py "WINDOW TITLE" out.png [display] -- capture ONE named X window (not the desktop) with xwd and save it as PNG."""
import subprocess, struct, sys
import numpy as np, cv2
name, out = sys.argv[1], sys.argv[2]; disp = sys.argv[3] if len(sys.argv) > 3 else ':1'
raw = subprocess.run(['xwd', '-display', disp, '-name', name, '-silent'], capture_output=True).stdout
if len(raw) < 100: sys.exit(f'no window named {name!r}')
h = struct.unpack('>25I', raw[:100]); hs, w, ht, bpp, bpl, ncol = h[0], h[4], h[5], h[11], h[12], h[19]
data = np.frombuffer(raw, np.uint8, offset=hs + ncol * 12).reshape(ht, bpl)[:, :w * (bpp // 8)].reshape(ht, w, bpp // 8)
cv2.imwrite(out, np.ascontiguousarray(data[:, :, :3])); print('wrote', out, (w, ht))
