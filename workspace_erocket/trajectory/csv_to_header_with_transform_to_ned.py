import csv
import sys
import os

def transform_row(row):
    """
    Transform coordinates from:
    Original -> x=up, y=front, z=right
    Target   -> x=front, y=right, z=up
    """
    t = row[0]

    # Original indices
    x, dx, ddx = row[1], row[2], row[3]
    y, dy, ddy = row[4], row[5], row[6]
    z, dz, ddz = row[7], row[8], row[9]

    # Target system mapping
    # new_x = old_y (front)
    # new_y = old_z (right)
    # new_z = old_x (up)
    return [
        t,
        y, dy, ddy,   # new X-axis
        z, dz, ddz,   # new Y-axis
        x, dx, ddx    # new Z-axis
    ]

def csv_to_header(csv_file):
    with open(csv_file, newline='') as f:
        reader = csv.reader(f, delimiter=',')
        header = next(reader)  # skip header line

        transformed_data = []
        for row in reader:
            if not row:
                continue
            row = [float(val) for val in row]
            transformed_data.append(transform_row(row))

    # Axis names for comments (after transformation)
    col_names = [
        "t [s]",
        "x [m]", "dx [m/s]", "ddx [m/s2]",
        "y [m]", "dy [m/s]", "ddy [m/s2]",
        "z [m]", "dz [m/s]", "ddz [m/s2]"
    ]

    # Output filename
    base, _ = os.path.splitext(csv_file)
    header_file = base + "_setpoints.h"

    # Write header file
    with open(header_file, 'w') as f:
        ncols = len(transformed_data[0])
        f.write("#ifndef SETPOINTS_H\n#define SETPOINTS_H\n\n")

        # Column names as comments
        f.write("// Columns:\n")
        for i, name in enumerate(col_names):
            f.write(f"// {i}: {name}\n")
        f.write("\n")

        f.write(f"const double Setpoints[][ {ncols} ] = {{\n")

        for row in transformed_data:
            formatted = ", ".join(f"{val:.8f}" for val in row)
            f.write(f"    {{ {formatted} }},\n")

        f.write("};\n\n#endif // SETPOINTS_H\n")

    print(f"Header file created: {header_file}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python script.py <input.csv>")
        sys.exit(1)

    csv_file = sys.argv[1]
    csv_to_header(csv_file)
