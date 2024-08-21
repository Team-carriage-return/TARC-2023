"""
    This program was made to fix an issue with early codebase versions which resulted in negative numbers being logged as "-12.-345" instead of "-12.345".

    The name, "happy little data gaps", comes about because our graphing software (Get Curve) would throw out the whole CSV line resulting in data gaps across all of our graphs. We needed a name for this data anomaly and the term/name "happy little data gaps" came about sarcastically.

    To use this program, pass in the file path of one flight log CSV as the first and only argument. The program will then parse the CSV and output a fixed version as a ".happyfix" file.
"""

import csv, sys

filename_to_fix = sys.argv[1]
csv_data = []
csv_data_to_save = []
num_of_items_fixed = 0

# Parse the CSV
with open(filename_to_fix, "r") as f:
    csv_reader = csv.reader(f)

    for row in csv_reader:
        csv_data.append(row)

# Fix double negative numbers (-12.-345 to -12.345)
for y in range(len(csv_data)):
    for x in range(len(csv_data[y])):
        token_split = csv_data[y][x].split(".")

        if len(token_split) == 2 and token_split[1][0] == "-":
            csv_data[y][x] = ("-" if token_split[0][0] != "-" else "") + token_split[0] + "." + token_split[1][1:]
            num_of_items_fixed += 1

    csv_data_to_save.append(",".join(csv_data[y]))

# Save output and report to user
open(filename_to_fix + ".happyfix", "w").write("\n".join(csv_data_to_save))
print("Fixed " + str(num_of_items_fixed) + " errors")