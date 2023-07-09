#!/usr/bin/python3
import sys
import argparse
import pathlib
import numpy as np
from scenario import *

def main():
    # ==================== ARGUMENTS ==============================
    parser = argparse.ArgumentParser(
        description="Generates statistics of scenarios from result file(s)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--scenarios",
        "-s",
        help="-1 for all scenarios, otherwise specify the scenario to print. Can be specified multiple times",
        type=int,
        nargs="*",
        default=-1,
    )
    parser.add_argument(
        "--paths",
        "-f",
        help="Full path for result file(s). Can be specified multiple times",
        action="extend",
        nargs="*",
        type=pathlib.Path,
        default=[],
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
        help="Output file path to print the shown results to. If none is specified, the results are printed to terminal",
        default=None,
        type=pathlib.Path,
    )
    args = parser.parse_args()

    stream = sys.stdout
    if args.print is not None:
        stream = open(args.print, "w")
        print(f"Results will be shown in '{args.print}'")

    # ==================== PARSE PATHS =========================
    if len(args.names) != 0 and len(args.algs) == 0:
        print(f"[WARN] No --algs specified for --names.")
        args.names.clear()
    if len(args.paths) == 0 and len(args.names) == 0:
        print(f"[WARN] No --paths or --names specified.")
        return;
    
    for name in args.names:
        for alg in args.algs:
            path = pathlib.Path(name + "." + alg + ".results");
            if args.dir is not None:
                path = args.dir / path
            args.paths.append(path)

    # ==================== CHECK IF ALL SCENS OR JUST SOME ==============================
    all_scens = False
    if isinstance(args.scenarios, list):
        args.scenarios.sort()
        all_scens = args.scenarios[0] < 0
    else:
        all_scens = args.scenarios < 0

    if all_scens:
        print(f"Showing results for all scenarios")
    else:
        scens_str = ", ".join(map(str, args.scenarios))
        print(f"Showing results for scenario(s): {scens_str}")

    # ==================== OPEN FILES ==============================
    for file_path in args.paths:
        if file_path.is_dir():
            print(f"[WARN] '{file_path}' is a directory. Skip.")
            continue
        elif not file_path.exists():
            print(f"[WARN] '{file_path}' does not exist. Skip.")
            continue

        print(f"Opening Result file: '{file_path}'")
        with open(file_path, "r") as file:
            lines = file.read().splitlines()
            print(f"{file_path.stem} has {len(lines)} scenario results")

            if all_scens:
                args.scenarios = range(len(lines))

            for scen_num in args.scenarios:
                if scen_num >= len(lines):
                    print(f"[WARN] Scen id {scen_num} exceeds the total number of scenarios in results file.")
                    continue
                scenario = Scenario()
                scenario.initFromLine(file_path.stem, scen_num, lines[scen_num])
                stream.write(repr(scenario) + "\n")


if __name__ == "__main__":
    main()
