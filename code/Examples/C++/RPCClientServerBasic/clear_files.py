# This script removes all lines containing //!-- from the application files
# and copies the cleaned files to a new directory called clean_files.

import os

WS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.join(WS_DIR, "src")
CLEAN_DIR = os.path.join(WS_DIR, "clean_files")

os.makedirs(CLEAN_DIR, exist_ok=True)
# Remove old clean_files files if not empty
for name in os.listdir(CLEAN_DIR):
    if os.path.isfile(os.path.join(CLEAN_DIR, name)):
        os.remove(os.path.join(CLEAN_DIR, name))
        print(f"Removed old file: {name}")

# Remove //!-- lines from all application files and copy them to clean_files/
for name in os.listdir(SRC_DIR):
    print(f"Processing {name}")
    if name.endswith(".hpp") or name.endswith(".cpp"):
        with open(os.path.join(SRC_DIR, name), "r") as f:
            lines = f.readlines()
        with open(os.path.join(CLEAN_DIR, name), "w") as f:
            for line in lines:
                if "//!--" not in line:
                    f.write(line)
        print(f"Cleaned {name} and copied to {CLEAN_DIR}")
