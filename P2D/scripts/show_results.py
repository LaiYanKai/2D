import sys
import argparse
import pathlib
import numpy


def scenario_num_type(x):
    x = int(x)
    if x < 0:
        raise argparse.ArgumentTypeError("All values provided must be >= 0")
    return x


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
if isinstance(args.scenarios, list):
    args.scenarios.sort()
print(args.scenarios)

THRES = 1e8
# class Scenario:
#     id = 0
#     path = []
#     nsec = float('nan')
#     cost = float('nan')

#     def __init__(self, nsec, path):
#         self.path = path
#         self.nsec = nsec
#         for i in range(len(path) - 1):
#             path[i] path[i-1]


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
        print(lines[0])
        print(lines[1])


print(args)
print(args.results)
print(args.scenarios)
