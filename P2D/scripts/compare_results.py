#!/usr/bin/python3
import sys
import argparse
import pathlib
import numpy as np
from scenario import *

def main():
    # ==================== ARGUMENTS ==============================
    parser = argparse.ArgumentParser(
        description="Display scenarios in different files if the scenarios have different path lengths as the one in the first file. Files must have the same length ",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "oracle_file",
        help="File path for the file to compare to. If other files have different lengths to the scenarios in this file, the scenarios are displayed ",
        type=pathlib.Path,
    )
    parser.add_argument(
        "other_files",
        help="Other files. Usage: 'result_file_path_1, result_file_path_2, ...'",
        action="extend",
        nargs="+",
        type=pathlib.Path,
    )
    args = parser.parse_args()

    # ==================== BUILD ORACLE FILE ======================
    print(f"Reading Oracle File: '{args.oracle_file}'")
    if args.oracle_file.is_dir():
        print(f"'{args.oracle_file}' is a directory. No comparisons are done.")
        return
    elif not args.oracle_file.exists():
        print(f"'{args.oracle_file}' does not exist. No comparisons are done.")
        return

    oracle_scens = []
    num_scens = 0
    with open(args.oracle_file, "r") as oracle_file:
        lines = oracle_file.read().splitlines()
        num_scens = len(lines)
        # oracle_scens = [Scenario()] * num_scens
        for i in range(num_scens):
            scen = Scenario()
            scen.initFromLine(args.oracle_file.stem, i+1, lines[i])
            oracle_scens.append(scen)

    if (num_scens == 0):
        print(f"Oracle file '{args.oracle_file}' has no scenarios. No comparisons are done.")
        return

    # ==================== BUILD OTHER FILES ======================
    other_files = []
    for other_file_path in args.other_files:
        print(f"Reading Other File: '{other_file_path}'")
        if other_file_path.is_dir():
            print(f"'{other_file_path}' is a directory. No comparison is done for this file.")
            return
        elif not other_file_path.exists():
            print(f"'{other_file_path}' does not exist. No comparison is done for this file.")
            return
        with open(other_file_path, "r") as other_file:
            lines = other_file.read().splitlines()
            if (num_scens != len(oracle_scens)):
                print(f"'{other_file_path}' has {num_scens} scenarios but oracle_file has {len(oracle_scens)}. Skip '{other_file_path}'")
            else:
                scens = []
                for i in range(num_scens):
                    scen = Scenario()
                    scen.initFromLine(other_file_path, i+1, lines[i])
                    scens.append(scen)
                other_files.append(scens)
                

    # ===================== COMPARE FILES ============================
    i = 0
    for oracle_scen in oracle_scens:
        has_diff = False
        for other_scens in other_files:
            if abs(other_scens[i].cost - oracle_scen.cost) > THRES:
                print (other_scens[i])
                has_diff = True
        if has_diff:
            print(oracle_scen)
            print("------")
        i += 1
        



if __name__ == "__main__":
    main()
