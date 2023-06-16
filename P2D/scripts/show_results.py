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
        help="0 for all scenarios, >0 for scenario to print. Can be specified multiple times",
        type=scenario_num_type,
        nargs="*",
        default=0,
    )
    parser.add_argument(
        "results",
        help="Result file location. Can be specified multiple times",
        action="extend",
        nargs="+",
        type=pathlib.Path,
    )
    args = parser.parse_args()

    # ==================== CHECK IF ALL SCENS OR JUST SOME ==============================
    all_scens = False
    if isinstance(args.scenarios, list):
        args.scenarios.sort()
        all_scens = args.scenarios[0] == 0
    else:
        all_scens = args.scenarios == 0

    if all_scens:
        print(f"Showing results for all scenarios")
    else:
        scens_str = ", ".join(map(str, args.scenarios))
        print(f"Showing results for scenario(s): {scens_str}")

    # ==================== OPEN FILES ==============================
    for file_path in args.results:
        if file_path.is_dir():
            print(f"'{file_path}' is a directory. Skip.")
            continue
        elif not file_path.exists():
            print(f"'{file_path}' does not exist. Skip.")
            continue

        print(f"Opening Result file: '{file_path}'")
        with open(file_path, "r") as file:
            lines = file.read().splitlines()
            print(f"{file_path.stem} has {len(lines)} scenario results")
            
            
            scen_num = 700
            scen_result = Scenario()
            scen_result.initFromLine(scen_num, lines[scen_num - 1])
            print(scen_result)
            return scen_result
            if all_scens:
                pass
            else:
                pass



if __name__ == "__main__":
    scen = main()
