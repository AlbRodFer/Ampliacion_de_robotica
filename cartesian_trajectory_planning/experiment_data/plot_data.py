from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt

# Set the CSV file to open (inside experiment_data).
# Example: CSV_FILE_NAME = "experiment_20260305_061534.csv"
# Leave as None to use the most recent CSV file in this folder.
CSV_FILE_NAME: str | None = None


def find_csv_file(data_dir: Path, csv_file_name: str | None) -> Path:
    if csv_file_name:
        csv_path = Path(csv_file_name)
        if not csv_path.is_absolute():
            csv_path = data_dir / csv_path
        if not csv_path.exists() or not csv_path.is_file():
            raise FileNotFoundError(f"CSV file not found: {csv_path}")
        return csv_path

    csv_files = sorted(data_dir.glob("*.csv"), key=lambda path: path.stat().st_mtime, reverse=True)
    if not csv_files:
        raise FileNotFoundError(f"No CSV files found in: {data_dir}")
    return csv_files[0]


def load_csv_data(csv_path: Path) -> dict[str, list[float]]:
    data = {"t": [], "X": [], "Y": [], "Z": [], "roll": [], "pitch": [], "yaw": []}
    skipped_rows = 0

    with csv_path.open("r", newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        required_columns = set(data.keys())
        if reader.fieldnames is None or not required_columns.issubset(reader.fieldnames):
            raise ValueError(
                "CSV is missing required columns: t, X, Y, Z, roll, pitch, yaw"
            )

        for row in reader:
            parsed_row: dict[str, float] = {}
            valid_row = True

            for key in data:
                value = row.get(key)
                if value is None:
                    valid_row = False
                    break

                value = value.strip()
                if value == "":
                    valid_row = False
                    break

                try:
                    parsed_row[key] = float(value)
                except ValueError:
                    valid_row = False
                    break

            if not valid_row:
                skipped_rows += 1
                continue

            for key in data:
                data[key].append(parsed_row[key])

    if len(data["t"]) == 0:
        raise ValueError(f"CSV file is empty: {csv_path}")

    if skipped_rows > 0:
        print(f"Warning: skipped {skipped_rows} malformed CSV rows.")

    return data


def plot_positions(data: dict[str, list[float]]) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    fig.suptitle("Cartesian Position vs Time", fontsize=14)

    axes[0].plot(data["t"], data["X"], color="red", label="X")
    axes[0].set_ylabel("X [m]")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(data["t"], data["Y"], color="green", label="Y")
    axes[1].set_ylabel("Y [m]")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(data["t"], data["Z"], color="blue", label="Z")
    axes[2].set_xlabel("t [s]")
    axes[2].set_ylabel("Z [m]")
    axes[2].legend(loc="best")
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.97])


def plot_orientation_rpy(data: dict[str, list[float]]) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    fig.suptitle("RPY Orientation vs Time", fontsize=14)

    axes[0].plot(data["t"], data["roll"], color="lightcoral", label="roll")
    axes[0].set_ylabel("roll [rad]")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(data["t"], data["pitch"], color="lightgreen", label="pitch")
    axes[1].set_ylabel("pitch [rad]")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(data["t"], data["yaw"], color="lightskyblue", label="yaw")
    axes[2].set_xlabel("t [s]")
    axes[2].set_ylabel("yaw [rad]")
    axes[2].legend(loc="best")
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.97])


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot X/Y/Z and roll/pitch/yaw from a CSV file in experiment_data."
    )
    parser.add_argument(
        "csv_file",
        nargs="?",
        default=None,
        help="CSV file name or path. Example: python3 plot_data.py experiment_name.csv",
    )
    args = parser.parse_args()

    data_dir = Path(__file__).resolve().parent
    selected_csv = args.csv_file if args.csv_file is not None else CSV_FILE_NAME
    csv_path = find_csv_file(data_dir, selected_csv)
    print(f"Using CSV: {csv_path}")

    data = load_csv_data(csv_path)
    plot_positions(data)
    plot_orientation_rpy(data)
    plt.show()


if __name__ == "__main__":
    main()
