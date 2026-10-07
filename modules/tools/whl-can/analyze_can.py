import argparse
import cantools
import re
import sys
import os

import pandas as pd
from rich.console import Console
from rich.table import Table
from rich import print as rprint


log_pattern = re.compile(
    r"\((?P<time>\d+\.\d+)\)\s+(?P<iface>\w+)\s+(?P<id>[0-9A-Fa-f]+)#(?P<data>[0-9A-Fa-f]*)"
)


def analyze_log(file_path, db, filters=None, target_signals=None):
    """Parse the log and return structured data."""
    parsed_data = []

    with open(file_path, "r") as f:
        for line in f:
            match = log_pattern.match(line)
            if not match:
                continue

            can_id = int(match.group("id"), 16)
            data_hex = match.group("data")
            data = bytes.fromhex(data_hex) if data_hex else b""

            try:
                message = db.get_message_by_frame_id(can_id)
            except KeyError:
                continue

            if filters and not any(s in message.name for s in filters):
                continue

            try:
                decoded = db.decode_message(can_id, data)

                # build base row data
                row = {
                    "Time": float(match.group("time")),
                    "Message": message.name,
                    "ID": f"0x{can_id:X}",
                }

                # if target signals specified, only extract those
                if target_signals:
                    matched_any = False
                    for sig in target_signals:
                        if sig in decoded:
                            row[sig] = decoded[sig]
                            matched_any = True
                    if not matched_any:
                        continue  # skip messages that do not contain requested signals
                else:
                    row.update(decoded)

                parsed_data.append(row)

            except Exception as e:
                pass  # ignore frames that fail to decode

    return parsed_data


def display_dashboard(df):
    """Display terminal latest state dashboard"""
    console = Console()
    table = Table(
        title="Vehicle CAN Messages Latest State Dashboard",
        show_header=True,
        header_style="bold magenta",
    )
    table.add_column("Latest Time", style="dim")
    table.add_column("Message", style="cyan")
    table.add_column("CAN ID", justify="right")
    table.add_column("Key Signals (Signals)", style="green")

    # group by message and take last row (latest state)
    latest_states = df.groupby("Message").last().reset_index()
    latest_states = latest_states.sort_values(by="Time", ascending=False)

    for _, row in latest_states.iterrows():
        msg_name = row["Message"]
        time_str = f"{row['Time']:.3f}"
        can_id = row["ID"]

        # extract signals, ignore base columns and NaN
        signals = []
        for col in df.columns:
            if col not in ["Time", "Message", "ID"] and pd.notna(row[col]):
                val = row[col]
                if isinstance(val, float):
                    signals.append(f"{col}: {val:.2f}")
                else:
                    signals.append(f"{col}: {val}")

        sig_str = " | ".join(signals)
        # truncate long signal string for nicer display
        if len(sig_str) > 80:
            sig_str = sig_str[:77] + "..."

        table.add_row(time_str, msg_name, can_id, sig_str)

    console.print(table)


def display_signal_trace(df, signals):
    """Trace specific signals time series and display table"""
    console = Console()
    table = Table(
        title=f"Signal Time Series Trace: {', '.join(signals)}",
        show_header=True,
        header_style="bold yellow",
    )
    table.add_column("Time", justify="left", style="dim")
    table.add_column("Message", style="cyan")

    for sig in signals:
        table.add_column(sig, justify="right", style="green")

    # drop rows where all these signals are NaN
    df_filtered = df.dropna(subset=signals, how="all")

    # only show first 50 rows to avoid flooding the terminal
    display_limit = 50
    for i, row in df_filtered.head(display_limit).iterrows():
        row_data = [f"{row['Time']:.4f}", str(row["Message"])]
        for sig in signals:
            val = row[sig] if pd.notna(row[sig]) else "-"
            if isinstance(val, float):
                row_data.append(f"{val:.2f}")
            else:
                row_data.append(str(val))
        table.add_row(*row_data)

    console.print(table)
    if len(df_filtered) > display_limit:
        rprint(
            f"[dim]... {len(df_filtered) - display_limit} rows hidden. Consider exporting with --csv to inspect.[/dim]"
        )


def load_dbc(path):
    # preserve original DBC fix logic
    try:
        return cantools.database.load_file(path)
    except Exception as e:
        raise RuntimeError(f"Failed to load DBC file: {e}")


def main():
    parser = argparse.ArgumentParser(description="高级 CAN 日志解析与分析工具")
    parser.add_argument("--dbc", required=True, help="Path to DBC file")
    parser.add_argument("--log", required=True, help="Path to candump log file")
    parser.add_argument(
        "--filter", help="Filter by message name, comma-separated, e.g. Steer,Brake"
    )
    parser.add_argument(
        "--signals",
        help="Extract specific signals, comma-separated, e.g. VehicleSpeed,SteerMod",
    )
    parser.add_argument(
        "--table",
        action="store_true",
        help="Display latest state dashboard in terminal",
    )
    parser.add_argument(
        "--csv", help="Export flattened time series CSV path (e.g. output.csv)"
    )

    args = parser.parse_args()

    try:
        db = load_dbc(args.dbc)
    except Exception as e:
        print(f"Failed to load DBC: {e}")
        sys.exit(1)

    filters = args.filter.split(",") if args.filter else None
    target_signals = args.signals.split(",") if args.signals else None

    # 1. parse logs into a list of dicts
    print("Parsing log...")
    parsed_data = analyze_log(
        args.log, db, filters=filters, target_signals=target_signals
    )

    if not parsed_data:
        print("No matching CAN data found.")
        sys.exit(0)

    # 2. convert to Pandas DataFrame (the analyst's swiss army knife)
    df = pd.DataFrame(parsed_data)

    # 3. sort by time
    df = df.sort_values(by="Time").reset_index(drop=True)

    # branch 1: trace specific signals and print table
    if target_signals:
        display_signal_trace(df, target_signals)

    # branch 2: display overall dashboard (when --table is used and no specific signals)
    elif args.table:
        display_dashboard(df)

    # branch 3: print last few records for quick inspection
    elif not args.csv:
        print("\nNo display option specified, previewing last 5 records:")
        print(df.tail(5).to_string())

    # branch 4: export to CSV
    if args.csv:
        # we usually might ffill() before plotting to align timestamps, but keep NaNs to preserve raw values
        df.to_csv(args.csv, index=False)
        print(f"\n✅ Data successfully exported to: {args.csv}")
        print(
            "Tip: You can open the CSV in Excel, or use Python/PlotJuggler for waveform plotting."
        )


if __name__ == "__main__":
    main()
