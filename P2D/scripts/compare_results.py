#!/usr/bin/python3
import sys
import argparse
import pathlib
import numpy as np
from scenario import *

def main():
    class FilePaths:
        oracle_filepath = []
        other_filepaths = []

    # ==================== ARGUMENTS ==============================
    parser = argparse.ArgumentParser(
        description="Show scenarios that have different path lengths between compared result files. Use with --oracle and --others, or --name and --algs. If comparing files 'results/dao/arena.TS2B.results' and 'results/dao/arena.ANYA2B.results' to the oracle file 'results/dao/arena.VG2B.results', the script can be called in two ways: '--oracle results/dao/arena.VG2B.results --others results/dao/arena.TS2B.results results/dao/arena.ANYA2B.results' OR '--name results/dao/arena --algs VG2B TS2B ANYA2B' ",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--oracle",
        "-i",
        help="File path to the main file which other files will be compared to. Used with --others.",
        nargs="?",
        default=None,
        type=pathlib.Path,
    )
    parser.add_argument(
        "--others",
        "-o",
        help="Other files. Multiple files can be specified Used with --oracle.",
        nargs="*",
        default=None,
        type=pathlib.Path,
    )
    parser.add_argument(
        "--name",
        "-n",
        help="Directory and name containing the compared files. All result files in the directory are named like '<dir>/<name>.<alg>.results', and this accepts '<dir>/<name>'. Can be specified multiple times to compare different maps and scenarios. Used with --algs",
        nargs="*",
        type=pathlib.Path,
    )
    parser.add_argument(
        "--algs",
        "-a",
        help="Algorithms of the compared files. The first algorithm is the oracle file. At least two algorithms must be compared. All result files in the directory are named like '<name>.<alg>.results'. Used with --name",
        nargs="*",
        type=str,
    )
    parser.add_argument(
        "--print",
        "-p",
        help="Output file path to print the comparisons to. Default is sysout (print to terminal)",
        default=None,
        type=pathlib.Path,
    )
    args = parser.parse_args()

    wrong = False
    parameter_type = True
    if not args.oracle and not args.others:
        if not args.name:
            wrong = True
            print(f"--name must be specified")
        if not args.algs or len(args.algs) < 2:
            wrong = True
            print(f"--algs must be specified with at least two options")
        parameter_type = False
    elif not args.name and not args.algs:
        if not args.oracle:
            wrong = True
            print(f"--oracle must be specified")
        if not args.others:
            wrong = True
            print(f"--others must be specified")
        parameter_type = True
    else:
        wrong = True
        print(
            "Either --oracle and --others must be specified, or --dir --name and --algs must be specified"
        )

    if wrong == True:
        print(
            "Wrong or invalid parameter(s) specified. Print help messages with -h or --help."
        )
        return

    stream = sys.stdout
    if args.print is not None:
        stream = open(args.print, "w")
        print(f"Comparisons will be shown in '{args.print}'")

    all_paths = []
    if parameter_type == True:
        filepath = FilePaths()
        filepath.oracle_filepath = args.oracle
        filepath.other_filepaths = [args.others]
        all_paths.append(filepath)
    else:
        for name in args.name:
            filepath = FilePaths()
            filepath.oracle_filepath = pathlib.Path(str(name) + "." + args.algs[0] + ".results")
            for i in range(1, len(args.algs)):
                filepath.other_filepaths.append( pathlib.Path(str(name) + "." + args.algs[i] + ".results"))
        all_paths.append(filepath)

    # ==================== BUILD ORACLE FILE ======================
    for filepath in all_paths:
        oracle_filepath = filepath.oracle_filepath
        other_filepaths = filepath.other_filepaths

        print(f"Reading Oracle File: '{oracle_filepath}'")
        if oracle_filepath.is_dir():
            print(f"'{oracle_filepath}' is a directory. No comparisons are done.")
            return
        elif not oracle_filepath.exists():
            print(f"'{oracle_filepath}' does not exist. No comparisons are done.")
            return

        oracle_scens = []
        num_scens = 0
        with open(oracle_filepath, "r") as oracle_file:
            lines = oracle_file.read().splitlines()
            num_scens = len(lines)
            # oracle_scens = [Scenario()] * num_scens
            for i in range(num_scens):
                scen = Scenario()
                scen.initFromLine(oracle_filepath.stem, i + 1, lines[i])
                oracle_scens.append(scen)

        if num_scens == 0:
            print(
                f"Oracle file '{oracle_filepath}' has no scenarios. No comparisons are done."
            )
            return

        # ==================== BUILD OTHER FILES ======================
        other_files = []
        for other_filepath in other_filepaths:
            print(f"Reading Other File: '{other_filepath}'")
            if other_filepath.is_dir():
                print(
                    f"'{other_filepath}' is a directory. No comparison is done for this file."
                )
                return
            elif not other_filepath.exists():
                print(
                    f"'{other_filepath}' does not exist. No comparison is done for this file."
                )
                return
            with open(other_filepath, "r") as other_file:
                lines = other_file.read().splitlines()
                if num_scens != len(oracle_scens):
                    print(
                        f"'{other_filepath}' has {num_scens} scenarios but oracle_file has {len(oracle_scens)}. Skip '{other_filepath}'"
                    )
                else:
                    scens = []
                    for i in range(num_scens):
                        scen = Scenario()
                        scen.initFromLine(other_filepath.stem, i + 1, lines[i])
                        scens.append(scen)
                    other_files.append(scens)

        # ===================== COMPARE FILES ============================
        i = 0
        for oracle_scen in oracle_scens:
            has_diff = False
            for other_scens in other_files:
                if (
                    np.isnan(other_scens[i].cost) != np.isnan(oracle_scen.cost)
                    or abs(other_scens[i].cost - oracle_scen.cost) > THRES
                ):
                    stream.write(repr(other_scens[i]) + "\n")
                    has_diff = True
            if has_diff:
                stream.write(repr(oracle_scen) + "\n")
                stream.write("------\n")
            i += 1


if __name__ == "__main__":
    main()
