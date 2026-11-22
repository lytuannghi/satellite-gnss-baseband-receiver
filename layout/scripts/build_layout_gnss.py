#!/usr/bin/env python3
"""
ASIC Physical Layout Generator (GDSII Stream Format)
Design: Multi-Constellation Satellite GNSS Baseband Receiver ASIC
Process: TSMC 28nm HPC+ CMOS
Dimensions: 1500 um x 1500 um
"""

import struct
import os

def create_gds_record(rec_type, data_type, data_bytes=b""):
    rec_length = len(data_bytes) + 4
    header = struct.pack(">HH", rec_length, (rec_type << 8) | data_type)
    return header + data_bytes

def create_boundary(layer, datatype, coords):
    res = bytearray()
    res += create_gds_record(0x08, 0x00) # BOUNDARY
    res += create_gds_record(0x0D, 0x02, struct.pack(">h", layer))
    res += create_gds_record(0x0E, 0x02, struct.pack(">h", datatype))
    coord_bytes = bytearray()
    for x, y in coords:
        coord_bytes += struct.pack(">ii", int(x), int(y))
    coord_bytes += struct.pack(">ii", int(coords[0][0]), int(coords[0][1]))
    res += create_gds_record(0x10, 0x03, coord_bytes) # XY
    res += create_gds_record(0x11, 0x00) # ENDEL
    return res

def create_box(layer, datatype, x1, y1, x2, y2):
    return create_boundary(layer, datatype, [(x1, y1), (x2, y1), (x2, y2), (x1, y2)])

def main():
    out_dir = os.path.join(os.path.dirname(__file__), "..", "gds")
    os.makedirs(out_dir, exist_ok=True)
    gds_path = os.path.join(out_dir, "satellite_gnss_receiver_top.gds")

    stream = bytearray()
    # HEADER
    stream += create_gds_record(0x00, 0x02, struct.pack(">h", 600))
    # BGNLIB
    stream += create_gds_record(0x01, 0x02, struct.pack(">12h", 2026, 11, 22, 12, 0, 0, 2026, 11, 22, 12, 0, 0))
    # LIBNAME
    lib_name = b"GNSS_RECEIVER_LIB\0"
    if len(lib_name) % 2 != 0: lib_name += b"\0"
    stream += create_gds_record(0x02, 0x06, lib_name)
    # UNITS (0.001 um database unit = 1 nm)
    stream += create_gds_record(0x03, 0x05, struct.pack(">dd", 0.001, 1e-9))

    # BGNSTR
    stream += create_gds_record(0x05, 0x02, struct.pack(">12h", 2026, 11, 22, 12, 0, 0, 2026, 11, 22, 12, 0, 0))
    # STRNAME
    cell_name = b"satellite_gnss_receiver_top\0"
    if len(cell_name) % 2 != 0: cell_name += b"\0"
    stream += create_gds_record(0x06, 0x06, cell_name)

    # 1. Chip Sealring & Boundary: 1500um x 1500um
    stream += create_box(100, 0, 0, 0, 1500000, 1500000)
    stream += create_box(100, 1, 25000, 25000, 1475000, 1475000)

    # 2. IO Pad Ring (Outer perimeter: RF ADC interface pads, APB3 bus pads)
    for i in range(13):
        pos = 65000 + i * 105000
        stream += create_box(105, 0, pos, 30000, pos + 70000, 100000)      # Bottom Pads
        stream += create_box(105, 0, pos, 1400000, pos + 70000, 1470000) # Top Pads
        stream += create_box(105, 0, 30000, pos, 100000, pos + 70000)      # Left Pads
        stream += create_box(105, 0, 1400000, pos, 1470000, pos + 70000) # Right Pads

    # 3. Core Logic Floorplan Blocks
    # u_regs: APB3 Register Bank & Control
    stream += create_box(30, 0, 150000, 150000, 1350000, 320000)

    # 32 Satellite Hardware Tracking Channels (Array 4x8)
    for row in range(4):
        for col in range(8):
            x1 = 150000 + col * 150000
            y1 = 360000 + row * 240000
            x2 = x1 + 135000
            y2 = y1 + 215000
            stream += create_box(31, 0, x1, y1, x2, y2) # Single Tracking Channel Macro

    # 4. Power Grid Network (VDD / VSS Straps in Metal 8 & Metal 9)
    for p in range(160000, 1350000, 85000):
        stream += create_box(48, 0, p, 130000, p + 12000, 1370000)      # M8 VDD/VSS Vertical Straps
        stream += create_box(49, 0, 130000, p, 1370000, p + 12000)      # M9 VDD/VSS Horizontal Straps

    # ENDSTR
    stream += create_gds_record(0x07, 0x00)
    # ENDLIB
    stream += create_gds_record(0x04, 0x00)

    with open(gds_path, "wb") as f:
        f.write(stream)

    print(f"[INFO] GDSII physical layout generated successfully: {gds_path} ({len(stream)} bytes)")

if __name__ == "__main__":
    main()
