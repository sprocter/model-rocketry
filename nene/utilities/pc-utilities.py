"""Utility functions (designed to run on a PC) for use with the Nene model rocket flight computer.

--------------------------------------------------------------------------------
Copyright (C) 2026 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
"""

import sys
import csv
import datetime as dt
from itertools import pairwise
import statistics
import matplotlib.pyplot as plt
import io
import base64


M_2_F = 3.280839895
MS_2_MPH = 2.2369362921
MSS_2_G = 0.1019716213


def parse_csv_header(header: str) -> dict:
    ret = {}
    hdr_elems = header.split(",")
    ret["SystemName"] = hdr_elems[0]
    try:
        ret["LaunchTime"] = dt.datetime.fromisoformat(hdr_elems[2])
    except:
        ret["LaunchTime"] = dt.datetime.today()
    ret["BattStart"] = float(hdr_elems[4])
    ret["BattEnd"] = float(hdr_elems[6])
    ret["MCUTempStart"] = int(hdr_elems[8])
    ret["MCUTempEnd"] = int(hdr_elems[10])
    return ret


def parse_csv(filename: str) -> list:
    ret = []
    with open(filename, newline="") as csvfile:
        ret.append(
            parse_csv_header(next(csvfile))
        )  # Manually process first row, its not columnar
        cread = csv.DictReader(csvfile, skipinitialspace=True)
        for row in cread:
            ret.append(row)
    return ret


def dm2dd(dm: str) -> str:
    if len(dm) < 2:
        return str(0)
    dd = float(dm[0:2])
    mm = float(dm[2:]) / 60
    return str(dd + mm)


def write_kml(data: list) -> None:
    coords = " ".join(
        [
            f"-{dm2dd(row['lon(ddmm.mmmm)'])},{dm2dd(row['lat (ddmm.mmmm)'])},{row['baro_alt (m)']}"
            for row in data[1:]
        ]
    )

    kml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Nene Flightpath</name>
    <Placemark>
      <name>Flight Path</name>
      <LineString>
        <altitudeMode>relativeToGround</altitudeMode>
        <coordinates>{coords}</coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>
"""
    with open(
        f'KML-{data[0]["SystemName"]}-{data[0]["LaunchTime"].strftime("%Y.%m.%d.%I.%M%p")}.kml',
        "w",
    ) as kmlfile:
        kmlfile.write(kml)


def get_rod_velocity(data: list, init_alti: float) -> float:
    for i in range(len(data)):
        if i <= 6:
            continue
        # Now find where we first are 1m higher than the initial altitude
        if float(data[i]["est_alt (m)"]) - init_alti < 1:
            continue
        return float(data[i]["est_speed(m/s)"])
    return 0


def get_ejec_idx(data: list) -> int:
    # Find the biggest positive change in x acceleration over the recent average
    # We're looking for sudden spikes in the direction the nose cone points
    diffs = []
    for i in range(len(data)):
        if i <= 6:
            continue
        avg_x_accs = statistics.mean(
            float(row["acc_x (m/s^2)"]) for row in data[i - 5 : i]
        )
        diffs.append(float(data[i]["acc_x (m/s^2)"]) - avg_x_accs)
    # Now get the index where this spike occurs
    return diffs.index(max(diffs)) + 6


def get_touchdown_idx(data: list) -> int:
    # Working backwards, find the first (last) significant acceleration
    diffs = []
    for i in reversed(range(len(data))):
        if i < 1:
            continue
        if (
            float(data[i]["acc_x (m/s^2)"]) ** 2
            + float(data[i]["acc_y (m/s^2)"]) ** 2
            + float(data[i]["acc_z (m/s^2)"]) ** 2
        ) ** 0.5 > 25:
            return i
    return -1


def get_stage_idxs(data: list) -> list[int]:
    stages = []
    G_2_MSS = 1 / MSS_2_G
    accelerating = False
    start = -1
    end = -1
    for i in range(len(data)):
        if i < 1:
            continue
        if not accelerating:
            if float(data[i]["acc_x (m/s^2)"]) > 2 * G_2_MSS:
                start = i
                accelerating = True
        else:
            if float(data[i]["acc_x (m/s^2)"]) < 0.5 * G_2_MSS:
                end = i
                # Disregard spikes of less than half a second
                if (
                    float(data[end]["time (ms)"]) - float(data[start]["time (ms)"])
                    >= 500.0
                ):
                    stages.append({"ignition": start, "burnout": end})
                accelerating = False
    return stages


def generate_table(data: list) -> str:

    system_name = data[0]["SystemName"]
    launch_date = data[0]["LaunchTime"].strftime("%A, %B %d, %Y")
    launch_time = data[0]["LaunchTime"].strftime("%I:%M:%S %p")

    # Get starting altitude by averaging some initial readings
    init_alti = statistics.mean(float(row["est_alt (m)"]) for row in data[1:6])
    ejec_idx = get_ejec_idx(data)
    touchdown_idx = get_touchdown_idx(data)
    stage_idxs = get_stage_idxs(data)
    duration_descent = dt.timedelta(
        milliseconds=(
            int(float(data[touchdown_idx]["time (ms)"]))
            - int(float(data[ejec_idx]["time (ms)"]))
        )
    )

    altitude_m = max(float(row["est_alt (m)"]) for row in data[1:]) - init_alti
    velocity_ms = max(float(row["est_speed(m/s)"]) for row in data[1:])
    accel_mss = max(float(row["acc_x (m/s^2)"]) for row in data[1 : ejec_idx - 1])

    velocity_rod_ms = get_rod_velocity(data, init_alti)
    velocity_ejec_ms = float(data[ejec_idx]["est_speed(m/s)"])
    velocity_descent_ms = (
        float(data[ejec_idx]["baro_alt (m)"])
        - float(data[touchdown_idx]["baro_alt (m)"])
    ) / duration_descent.seconds

    if len(stage_idxs) > 0:
        duration_stage1 = dt.timedelta(
            milliseconds=(
                int(float(data[stage_idxs[0]["burnout"]]["time (ms)"]))
                - int(float(data[stage_idxs[0]["ignition"]]["time (ms)"]))
            )
        )
        tilt_stage1 = float(data[stage_idxs[0]["ignition"]]["est_tilt (deg)"])
        if len(stage_idxs) > 1:
            # hell yeah

            velocity_stage2_igni_m = float(
                data[stage_idxs[1]["ignition"]]["est_speed(m/s)"]
            )
            altitude_stage2_igni_m = float(
                data[stage_idxs[1]["ignition"]]["est_alt (m)"]
            )

            velocity_stage2_row = f"""<tr>
                    <td class="tg-cly1">… at Second Stage Ignition (m/s, mph)</td>
                    <td class="tg-cly1">{velocity_stage2_igni_m:.2f}</td>
                    <td class="tg-cly1">{velocity_stage2_igni_m * MS_2_MPH:.2f}</td>
                    <td class="tg-0lax"></td>
                </tr>"""

            altitude_stage2_row = f"""<tr>
                    <td class="tg-cly1">… at Second Stage Ignition (m, ft)</td>
                    <td class="tg-cly1">{altitude_stage2_igni_m:.2f}</td>
                    <td class="tg-cly1">{altitude_stage2_igni_m * M_2_F:.2f}</td>
                    <td class="tg-0lax"></td>
                </tr>"""

            duration_stage2 = dt.timedelta(
                milliseconds=(
                    int(float(data[stage_idxs[1]["burnout"]]["time (ms)"]))
                    - int(float(data[stage_idxs[1]["ignition"]]["time (ms)"]))
                )
            )
            tilt_stage2 = float(data[stage_idxs[1]["ignition"]]["est_tilt (deg)"])
        else:
            velocity_stage2_row = ""
            altitude_stage2_row = ""
            duration_stage2 = dt.timedelta(milliseconds=(0))
            tilt_stage2 = None
    else:
        tilt_stage1 = 0.0
        tilt_stage2 = None
        duration_stage1 = dt.timedelta(seconds=0)
        velocity_stage2_row = ""
        altitude_stage2_row = ""
        duration_stage2 = dt.timedelta(milliseconds=(0))

    altitude_ejec_m = float(data[ejec_idx]["est_alt (m)"])

    tilt_ejec = float(data[ejec_idx]["est_tilt (deg)"])

    frametimes = [int(float(row["prev_frame_time (μs)"])) for row in data[1:]]
    frametime_percentiles = statistics.quantiles(frametimes, n=100, method="inclusive")

    return f"""
    <table class="tg">
        <!-- Table formatting generated via https://www.tablesgenerator.com/html_tables -->
        <thead>
            <tr>
                <th class="tg-cly1">{system_name}</th>
                <th class="tg-cly1" colspan="2">{launch_date}</th>
                <th class="tg-cly1">{launch_time}</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td class="tg-0lax"></td>
                <td class="tg-cly1">SI</td>
                <td class="tg-cly1">Imperial</td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-cly1">Maximum:</td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-cly1">… Altitude (m, ft)</td>
                <td class="tg-cly1">{altitude_m:,.2f}</td>
                <td class="tg-cly1">{altitude_m * M_2_F:,.2f}</td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-cly1">… Velocity (m/s, mph)</td>
                <td class="tg-cly1">{velocity_ms:,.2f}</td>
                <td class="tg-cly1">{velocity_ms * MS_2_MPH:,.2f}</td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-cly1">… Acceleration (m/s2, g)</td>
                <td class="tg-cly1">{accel_mss:,.2f}</td>
                <td class="tg-cly1">{accel_mss * MSS_2_G:,.2f}</td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-cly1">Velocity:</td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-cly1">… Off Rod (1m) (m/s, mph)</td>
                <td class="tg-cly1">{velocity_rod_ms:,.2f}</td>
                <td class="tg-cly1">{velocity_rod_ms * MS_2_MPH:,.2f}</td>
                <td class="tg-0lax"></td>
            </tr>
            {velocity_stage2_row}
            <tr>
                <td class="tg-cly1">… at Ejection Charge (m/s, mph)</td>
                <td class="tg-cly1">{velocity_ejec_ms:,.2f}</td>
                <td class="tg-cly1">{velocity_ejec_ms * MS_2_MPH:,.2f}</td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-cly1">… Descent (Average) (m/s, mph)</td>
                <td class="tg-cly1">{velocity_descent_ms:,.2f}</td>
                <td class="tg-cly1">{velocity_descent_ms * MS_2_MPH:,.2f}</td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-cly1">Altitude:</td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
            </tr>
            {altitude_stage2_row}
            <tr>
                <td class="tg-cly1">… at Ejection Charge (m, ft)</td>
                <td class="tg-cly1">{altitude_ejec_m:,.2f}</td>
                <td class="tg-cly1">{altitude_ejec_m * M_2_F:,.2f}</td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-cly1">Tilt: (°)</td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-cly1">… at Motor Ignition (Stages)</td>
                <td class="tg-cly1">{tilt_stage1:.2f}</td>
                <td class="tg-cly1">{"{x:.2f}".format(x=tilt_stage2) if tilt_stage2 is not None else ""}</td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-cly1">… at Ejection Charge</td>
                <td class="tg-cly1">{tilt_ejec:.2f}</td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-cly1">Duration: (Min:Sec)</td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-cly1">… of Stage Burns</td>
                <td class="tg-cly1">{str(duration_stage1)[2:10]}</td>
                <td class="tg-cly1">{str(duration_stage2)[2:10] if duration_stage2.microseconds > 0 else ""}</td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-cly1">… of Descent</td>
                <td class="tg-cly1">{str(duration_descent)[2:10]}</td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-cly1">CPU Utilization (%)</td>
                <td class="tg-cly1">{100*(statistics.mean(frametimes)/(1_000_000/45)):.2f}</td>
                <td class="tg-0lax"></td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-cly1">Frame Time (μs) (Avg, StdDev)</td>
                <td class="tg-cly1">{statistics.mean(frametimes):,.2f}</td>
                <td class="tg-cly1">{statistics.pstdev(frametimes):,.2f}</td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-cly1">Frame Time (μs) (95th %, 99th %, Worst)</td>
                <td class="tg-cly1">{frametime_percentiles[94]:,.2f}</td>
                <td class="tg-cly1">{frametime_percentiles[98]:,.2f}</td>
                <td class="tg-cly1">{max(frametimes):,.2f}</td>
            </tr>
            <tr>
                <td class="tg-cly1">Battery Level (Start, End)</td>
                <td class="tg-cly1">{data[0]["BattStart"]:.2f}</td>
                <td class="tg-cly1">{data[0]["BattEnd"]:.2f}</td>
                <td class="tg-0lax"></td>
            </tr>
            <tr>
                <td class="tg-cly1">MCU Temp (Start, End) (°C) </td>
                <td class="tg-cly1">{data[0]["MCUTempStart"]}</td>
                <td class="tg-cly1">{data[0]["MCUTempEnd"]}</td>
                <td class="tg-0lax"></td>
            </tr>
        </tbody>
    </table>
"""


def fig_to_base64(fig):
    # From https://stackoverflow.com/a/49016797
    img = io.BytesIO()
    fig.savefig(img, format="png", bbox_inches="tight")
    img.seek(0)

    return base64.b64encode(img.getvalue())


def generate_motion_plot(data: list, page_title: str) -> str:
    fig, ax = plt.subplots()
    ax.plot([1, 2, 3, 4], [1, 4, 2, 3])
    encoded = fig_to_base64(fig)
    return '<img src="data:image/png;base64, {}">'.format(encoded.decode("utf-8"))


def write_html(data: list) -> None:
    page_title = (
        f"{data[0]["LaunchTime"].strftime("%Y.%m.%d.%I.%M%p")}-{data[0]["SystemName"]}"
    )

    html = f"""<!DOCTYPE html>
<html>
    <head><title>{page_title}</title>
        <style type="text/css">
        .tg  {{border-collapse:collapse;border-color:#ccc;border-spacing:0;}}
        .tg td{{background-color:#fff;border-color:#ccc;border-style:solid;border-width:1px;color:#333;
        font-family:Arial, sans-serif;font-size:14px;overflow:hidden;padding:10px 5px;word-break:normal;}}
        .tg th{{background-color:#f0f0f0;border-color:#ccc;border-style:solid;border-width:1px;color:#333;
        font-family:Arial, sans-serif;font-size:14px;font-weight:normal;overflow:hidden;padding:10px 5px;word-break:normal;}}
        .tg .tg-cly1{{text-align:left;vertical-align:middle}}
        .tg .tg-0lax{{text-align:left;vertical-align:top}}
        </style>
    </head>"""
    html += generate_table(data)
    html += generate_motion_plot(data, page_title)
    html += f"""
</html>"""

    with open(
        f'Summary-{data[0]["SystemName"]}-{data[0]["LaunchTime"].strftime("%Y.%m.%d.%I.%M%p")}.html',
        "w",
    ) as htmlfile:
        htmlfile.write(html)


filename = sys.argv[1]
parsed_csv = parse_csv(filename)
write_html(parsed_csv)
write_kml(parsed_csv)
