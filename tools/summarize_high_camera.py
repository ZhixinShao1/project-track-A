
import json
import csv
from pathlib import Path
from statistics import mean, stdev

RESULTS_DIR = Path(__file__).resolve().parents[1] / "results"

SEEDS = [2026, 2027, 2028, 2029, 2030]

records = []

# Select the latest nominal result for each seed.
# For seed 2026, this should select the double-speed run.
for seed in SEEDS:

    candidates = []

    for path in RESULTS_DIR.glob("*.json"):
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)

        if (
            data.get("seed") == seed
            and data.get("scenario") == "high_camera_noise"
            and "trial_success" in data
        ):
            candidates.append((path, data))

    if not candidates:
        raise RuntimeError(
            f"No nominal result found for seed {seed}"
        )

    path, data = max(
        candidates,
        key=lambda item: item[0].stat().st_mtime
    )

    print(f"Seed {seed}: {path.name}")
    records.append(data)

metrics = [
    "position_rmse_m",
    "heading_rmse_rad",
    "x_2sigma_coverage_percent",
    "y_2sigma_coverage_percent",
    "heading_2sigma_coverage_percent",
    "actual_path_length_m",
    "mean_abs_command_v_mps",
    "mean_abs_command_omega_radps",
    "camera_visible_percent",
    "docking_range_error_m",
    "docking_bearing_error_deg"
]

successes = sum(
    bool(r["trial_success"]) for r in records
)

summary = {
    "number_of_trials": len(records),
    "successful_trials": successes,
    "success_rate_percent": (
        100.0 * successes / len(records)
    )
}

for metric in metrics:
    values = [float(r[metric]) for r in records]

    summary[f"mean_{metric}"] = mean(values)
    summary[f"std_{metric}"] = stdev(values)

# Completion time is compared for successful trials.
successful_durations = [
    float(r["duration_s"])
    for r in records
    if r["trial_success"]
]

summary["mean_success_duration_s"] = (
    mean(successful_durations)
    if successful_durations else None
)

print("\n===== HIGH CAMERA NOISE SUMMARY =====")

for key, value in summary.items():
    if isinstance(value, float):
        print(f"{key}: {value:.4f}")
    else:
        print(f"{key}: {value}")

# Save individual trial comparison
csv_path = RESULTS_DIR / "high_camera_noise_summary.csv"

fieldnames = [
    "seed",
    "trial_success",
    "duration_s",
    *metrics
]

with open(
    csv_path, "w",
    newline="",
    encoding="utf-8"
) as f:

    writer = csv.DictWriter(
        f,
        fieldnames=fieldnames
    )

    writer.writeheader()

    for r in records:
        writer.writerow({
            key: r.get(key)
            for key in fieldnames
        })

# Save summary statistics
json_path = RESULTS_DIR / "high_camera_noise_summary.json"

with open(json_path, "w", encoding="utf-8") as f:
    json.dump(summary, f, indent=4)

print("\nSummary files saved successfully.")
