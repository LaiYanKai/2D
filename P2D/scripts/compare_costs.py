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
        description="Show scenarios if they have different path lengths between different algorithms (result files). Use either, --oracle and --others, or --name and --algs. E.g. when comparing files 'results/dao/arena.TS2B.results' and 'results/dao/arena.ANYA2B.results' to the oracle file 'results/dao/arena.VG2B.results', the arguments can be specified in two equivalent ways: '--oracle results/dao/arena.VG2B.results --others results/dao/arena.TS2B.results results/dao/arena.ANYA2B.results'    or    '--name results/dao/arena --algs VG2B TS2B ANYA2B' ",
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
        "--dir",
        "-d",
        help="Directory containing result file(s). All result files in the directory are named like '[dir/][sub_dir/]stem.alg.results', and this accepts '[sub_dir/]stem'. Used with --names and --algs",
        nargs="?",
        default="results",
        type=pathlib.Path
    )
    parser.add_argument(
        "--names",
        "-n",
        help="Name of files, including sub_dir if any. All result files in the directory are named like '[dir/][sub_dir/]stem.alg.results', and this accepts '[sub_dir/]stem'. Can be specified multiple times to show files. Used with --algs and --dir",
        nargs="*",
        default=[],
        type=str,
    )
    parser.add_argument(
        "--algs",
        "-a",
        help="Results for files from an algorithm. All result files in the directory are named like '[dir/][sub_dir/]stem.alg.results', and this argument accepts 'alg'. 'alg' can be attached with a suffix like 'R2.0'. Used with --dir and --names",
        nargs="*",
        default=[],
        type=str,
    )
    parser.add_argument(
        "--print",
        "-p",
        help="Output file path to print the differences to. If none is specified, the differences are printed to terminal",
        default=None,
        type=pathlib.Path,
    )
    args = parser.parse_args()

    wrong = False
    parameter_type = True
    if not args.oracle and not args.others:
        if not args.names:
            wrong = True
            print(f"--name must be specified")
        if not args.algs or len(args.algs) < 2:
            wrong = True
            print(f"--algs must be specified with at least two options")
        parameter_type = False
    elif not args.names and not args.algs:
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
        for name in args.names:
            filepath = FilePaths()
            filepath.oracle_filepath = pathlib.Path(name + "." + args.algs[0] + ".results")
            if args.dir is not None:
                filepath.oracle_filepath = args.dir / filepath.oracle_filepath
            filepath.other_filepaths = []
            for i in range(1, len(args.algs)):
                path = pathlib.Path(name + "." + args.algs[i] + ".results")
                if args.dir is not None:
                    path = args.dir / path
                filepath.other_filepaths.append(path)
            all_paths.append(filepath)

    # ==================== BUILD ORACLE FILE ======================
    for filepath in all_paths:
        oracle_filepath = filepath.oracle_filepath
        other_filepaths = filepath.other_filepaths

        print(f"Reading Oracle File: '{oracle_filepath}'")
        if oracle_filepath.is_dir():
            print(f"[WARN] '{oracle_filepath}' is a directory. No comparisons are done for this file.")
            continue
        elif not oracle_filepath.exists():
            print(f"[WARN]' {oracle_filepath}' does not exist. No comparisons are done for this file.")
            continue

        oracle_scens = []
        num_scens = 0
        with open(oracle_filepath, "r") as oracle_file:
            lines = oracle_file.read().splitlines()
            num_scens = len(lines)
            for i in range(num_scens):
                scen = Scenario()
                scen.initFromLine(oracle_filepath.stem, i + 1, lines[i])
                oracle_scens.append(scen)

        if num_scens == 0:
            print(
                f"[WARN] Oracle file '{oracle_filepath}' has no scenarios. No comparisons are done for this file."
            )
            continue

        # ==================== BUILD OTHER FILES ======================
        other_files = []
        num_files_compared = 0
        for other_filepath in other_filepaths:
            print(f"Reading Other File: '{other_filepath}'")
            if other_filepath.is_dir():
                print(
                    f"[WARN] '{other_filepath}' is a directory. No comparisons are done for this file."
                )
                continue
            elif not other_filepath.exists():
                print(
                    f"[WARN] '{other_filepath}' does not exist. No comparisons are done for this file."
                )
                continue
            with open(other_filepath, "r") as other_file:
                lines = other_file.read().splitlines()
                if num_scens != len(oracle_scens):
                    print(
                        f"[WARN] '{other_filepath}' has {num_scens} scenarios but oracle_file has {len(oracle_scens)}. Skip '{other_filepath}'"
                    )
                else:
                    num_files_compared += 1
                    scens = []
                    for i in range(num_scens):
                        scen = Scenario()
                        scen.initFromLine(other_filepath.stem, i, lines[i])
                        scens.append(scen)
                    other_files.append(scens)

        # ===================== COMPARE FILES ============================
        i = 0
        has_differences = False
        for oracle_scen in oracle_scens:
            has_diff = False
            for other_scens in other_files:
                if (
                    np.isnan(other_scens[i].cost) != np.isnan(oracle_scen.cost)
                    or abs(other_scens[i].cost - oracle_scen.cost) > THRES
                ):
                    stream.write(repr(other_scens[i]) + "\n")
                    has_diff = True
                    has_differences = True
            if has_diff:
                stream.write(repr(oracle_scen) + "\n")
                stream.write("------\n")
            i += 1
        if has_differences:
            print(f"[WARN] Some scenario(s) have different path lengths")
        elif num_files_compared != len(other_filepaths):
            print(f"[WARN] Not all files were compared")
        else:
            print(f"[ OK ] All scenarios have the same path length")
        print(f"--- Comparison with Oracle {oracle_filepath} done.")
if __name__ == "__main__":
    main()
