import csv

files = ["Raceline_inside.csv", "Raceline_middle.csv", "Raceline_outside.csv"]

for filename in files:
    try:
        with open(filename, 'r') as f_in, open(f"modified_{filename}", 'w', newline='') as f_out:
            reader = csv.reader(f_in)
            writer = csv.writer(f_out)
            for row in reader:
                if row:
                    row[-1] = "3.0"  # Change last column to 3.0
                    writer.writerow(row)
        print(f"Created: modified_{filename}")
    except FileNotFoundError:
        print(f"Error: {filename} not found in this folder.")