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
from collections import deque
import numpy as np
import statistics
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator
import io
import base64


M_2_F = 3.280839895
MS_2_MPH = 2.2369362921
MSS_2_G = 0.1019716213
SENS_HZ = 45  # Sensor readings per second


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


def fix_timestamp(data: list[dict]) -> list[dict]:
    for i in range(len(data)):
        if i < 2:
            continue
        if float(data[i]["time (ms)"]) - float(data[i - 1]["time (ms)"]) > 500:
            data[i]["time (ms)"] = float(data[i - 1]["time (ms)"]) + 22
    return data


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


def get_event_indexes(data: list) -> dict:
    # Slopes of the altitude line, calculated by fitting a line (1st order polynomial) to the last second of readings
    betas_alt = deque(maxlen=SENS_HZ)

    # Return data structure. Maps event names to their index in the data parameter
    idxs = {}
    idxs["ignition"] = []  # We can have multiple ignitions (multiple stages)
    idxs["burnout"] = []

    # Flags that flip to true when the event has been detected
    # These will occur in the order listed (except apogee and ejection,
    # which may occur as specified or ejection then apogee)
    ignited = False
    launched = False
    apogee_reached = False
    ejected = False
    ground_hit = False

    # We need a normalized amount of time close to ~1/10 of a second
    moment = SENS_HZ // 10

    # Stages after the first are harder to detect and require this secondary
    # flag. It flips to true when we might have detected a second stage
    # ignition but can't confirm it yet
    ignit_candidate_idx = -1

    # Skip the first two rows (which have metadata) and the first moment
    for i in range(2 + moment, len(data)):
        # We can't use the estimated altitude after ejection.
        # But until then, it gives nice data from which to detect ignition,
        # launch, and apogee.
        # Ejection is determined by the accelerometer and having launched.
        if not (ejected and apogee_reached):
            x = np.array([float(row["time (ms)"]) for row in data[i - moment : i]])
            y = np.array([float(row["est_alt (m)"]) for row in data[i - moment : i]])
            beta_alt = np.polyfit(x, y, 1)[0]

            if i > SENS_HZ + 2:
                if not launched:
                    # Significant acceleration = ignition
                    if (
                        not ignited
                        and float(data[i]["h_acc_z (m/s^2)"]) * MSS_2_G > 1.05
                    ):
                        idxs["ignition"].append(i)
                        ignited = True
                    # Increasing altitude = launch
                    if ignited and beta_alt > 0.001:
                        idxs["launch"] = i
                        betas_alt.clear()
                        launched = True
                elif launched:
                    accels = np.array(
                        [abs(float(row["acc_z (m/s^2)"])) for row in data[i - moment : i]]
                    )
                    # Possibly ignited, not sure yet
                    if not ignited and ignit_candidate_idx > 0:
                        # If we have low acceleration, cancel this candidate
                        if float(data[i]["h_acc_z (m/s^2)"]) * MSS_2_G < 1.05:
                            ignit_candidate_idx = -1
                        # If it's been ~1/5 of a second, label it an ignition
                        elif i - ignit_candidate_idx >= moment * 2:
                            ignited = True
                    elif ignited:
                        # If we have low acceleration, we've burned out
                        if float(data[i]["h_acc_z (m/s^2)"]) * MSS_2_G < 1.05:
                            ignited = False
                            # If we have a valid ignition candidate, this was a 
                            # stage burn so we should record the ignition as 
                            # well
                            if ignit_candidate_idx > 0:
                                idxs["ignition"].append(ignit_candidate_idx)
                                ignit_candidate_idx = -1
                            idxs["burnout"].append(i)
                    if not ignited and ignit_candidate_idx < 0:
                        # If we have high acceleration, we might have an 
                        # ignition. We won't know until we continue the 
                        # ignition for ~1/5 of a second
                        if float(data[i]["h_acc_z (m/s^2)"]) * MSS_2_G > 1.05:
                            ignit_candidate_idx = i
                    # Falling but we were climbing until now = apogee
                    if beta_alt < 0 and np.min(betas_alt) > 0:
                        idxs["apogee"] = i
                        apogee_reached = True
                        apogee = float(data[i]["est_alt (m)"])
                    # Significant, sudden acceleration in the Z axis = ejection
                    if (
                        not ejected
                        and abs(float(data[i]["acc_z (m/s^2)"])) - np.mean(accels) > 100
                    ):
                        idxs["ejection"] = i
                        ejected = True
            betas_alt.append(beta_alt)
        else:
            # Fall for at least a second after ejection before looking for
            # ground hit
            if i - idxs["ejection"] < SENS_HZ or i - idxs["apogee"] < SENS_HZ:
                continue
            xs = np.array(
                [
                    float(row["h_acc_x (m/s^2)"])
                    for row in data[i - moment * 3 : i - moment * 2]
                ]
            )
            ys = np.array(
                [
                    float(row["h_acc_y (m/s^2)"])
                    for row in data[i - moment * 3 : i - moment * 2]
                ]
            )
            zs = np.array(
                [
                    float(row["h_acc_z (m/s^2)"])
                    for row in data[i - moment * 3 : i - moment * 2]
                ]
            )
            alts = np.array(
                [float(row["baro_alt (m)"]) for row in data[i - moment * 2 : i]]
            )
            if (
                not ground_hit
                # Significant acceleration bump / jostle
                and np.std(xs) + np.std(ys) + np.std(zs) > 20 
                and np.std(alts) < 0.1 # No significant altimeter change
            ):
                idxs["ground_hit"] = i - moment * 2
                ground_hit = True

    if "ground_hit" not in idxs:
        idxs["ground_hit"] = (
            len(data) - 1  # If we don't find a ground hit, assume end of data
        )
    print(idxs)
    return idxs


def get_rod_velocity(data: list, init_alti: float) -> float:
    for i in range(len(data)):
        if i <= 6:
            continue
        # Now find where we first are 1m higher than the initial altitude
        if float(data[i]["est_alt (m)"]) - init_alti < 1:
            continue
        return float(data[i]["est_speed(m/s)"])
    return 0


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
            if float(data[i]["acc_z (m/s^2)"]) > 2 * G_2_MSS:
                start = i
                accelerating = True
        else:
            if float(data[i]["acc_z (m/s^2)"]) < 0.5 * G_2_MSS:
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
    launch_time = (  # Convert launch time (UTC) to the correct timezone
        data[0]["LaunchTime"]
        .astimezone(data[0]["LaunchTime"].astimezone().tzinfo)
        .strftime("%I:%M:%S %p")
    )

    idxs = get_event_indexes(data)

    # Get starting altitude by averaging some initial readings
    init_alti = statistics.mean(float(row["est_alt (m)"]) for row in data[1:6])
    duration_descent = dt.timedelta(
        milliseconds=(
            int(float(data[idxs["ground_hit"]]["time (ms)"]))
            - int(float(data[idxs["ejection"]]["time (ms)"]))
        )
    )

    # altitude_m = max(float(row["est_alt (m)"]) for row in data[1:]) - init_alti
    altitude_m = float(data[idxs["apogee"]]["est_alt (m)"]) - init_alti
    velocity_ms = max(float(row["est_speed(m/s)"]) for row in data[1:])
    accel_mss = max(
        float(row["acc_z (m/s^2)"]) for row in data[1 : idxs["ejection"] - 1]
    )

    # Print these now cuz it sucks waiting on the whole file to process
    print(
        f"Max: Altitude {altitude_m:,.2f}m\tVelocity {velocity_ms:,.2f}m/s\tAcceleration {accel_mss:,.2f}m/s/s"
    )

    velocity_rod_ms = get_rod_velocity(data, init_alti)
    velocity_ejec_ms = float(data[idxs["ejection"]]["est_speed(m/s)"])
    velocity_descent_ms = (
        float(data[idxs["ejection"]]["baro_alt (m)"])
        - float(data[idxs["ground_hit"]]["baro_alt (m)"])
    ) / duration_descent.seconds

    if len(idxs["ignition"]) > 0:
        duration_stage1 = dt.timedelta(
            milliseconds=(
                int(float(data[idxs["burnout"][0]]["time (ms)"]))
                - int(float(data[idxs["ignition"][0]]["time (ms)"]))
            )
        )
        tilt_stage1 = float(data[idxs["ignition"][0]]["est_tilt (deg)"])
        if len(idxs["ignition"]) > 1:
            # hell yeah

            velocity_stage2_igni_m = float(data[idxs["ignition"][1]]["est_speed(m/s)"])
            altitude_stage2_igni_m = float(data[idxs["ignition"][1]]["est_alt (m)"])

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
                    int(float(data[idxs["burnout"][1]]["time (ms)"]))
                    - int(float(data[idxs["ignition"][1]]["time (ms)"]))
                )
            )
            tilt_stage2 = float(data[idxs["ignition"][1]]["est_tilt (deg)"])
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

    altitude_ejec_m = float(data[idxs["ejection"]]["est_alt (m)"])

    tilt_ejec = float(data[idxs["ejection"]]["est_tilt (deg)"])

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
                <td class="tg-cly1">{100*(statistics.mean(frametimes)/(1_000_000/SENS_HZ)):.2f}</td>
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


def get_spin(data: list[dict]) -> list[float]:
    spin = [0]
    for x, y in pairwise(row["est_roll(deg)"] for row in data):
        x = float(x)
        y = float(y)
        if abs(y - x) < 180:
            spin.append((y - x) * SENS_HZ)
        else:  # rollover
            if y > x:
                spin.append((y - x - 360) * SENS_HZ)
            else:
                spin.append((y - x + 360) * SENS_HZ)
    return spin


def fig_to_base64(fig):
    # From https://stackoverflow.com/a/49016797
    img = io.BytesIO()
    fig.savefig(img, format="png", bbox_inches="tight", dpi=100)
    img.seek(0)

    return base64.b64encode(img.getvalue())


def generate_plot(
    data: list[dict],
    ydata: list[list[float]],
    ylabels: list[str],
    yaxislabel: str,
) -> str:
    fig, ax = plt.subplots()
    x = [int(float(row["time (ms)"])) / 1000 for row in data[1 : len(ydata[0]) + 1]]
    for i in range(len(ydata)):
        ax.plot(x, ydata[i], label=ylabels[i])
    ax.legend()
    ax.xaxis.set_minor_locator(AutoMinorLocator(2))
    ax.set_xlabel("Time (seconds)")
    ax.yaxis.set_minor_locator(AutoMinorLocator(2))
    ax.set_ylabel(yaxislabel)
    ax.grid(which="both", linestyle=":")
    encoded = fig_to_base64(fig)
    return '<img src="data:image/png;base64, {}" />'.format(encoded.decode("utf-8"))


def generate_motion_plot(data: list) -> str:
    range_end = get_event_indexes(data)["ejection"]
    ydata = []
    ydata.append([float(row["est_alt (m)"]) for row in data[1:range_end]])
    ydata.append([float(row["acc_z (m/s^2)"]) for row in data[1:range_end]])
    ydata.append([float(row["est_speed(m/s)"]) for row in data[1:range_end]])
    ylabels = ["Altitude (m)", "Vertical Acceleration (m/s^2)", "Estimated Speed (m/s)"]
    return generate_plot(data, ydata, ylabels, "Meters")


def generate_alti_plot(data: list) -> str:
    ydata = []
    ydata.append([float(row["baro_alt (m)"]) for row in data[1:]])
    return generate_plot(data, ydata, ["Altitude (m)"], "Meters")


def generate_orientation_plot(data: list) -> str:
    range_end = get_event_indexes(data)["ejection"]
    ydata = []
    ydata.append(get_spin(data[1:range_end]))
    ydata.append([float(row["est_tilt (deg)"]) for row in data[1:range_end]])
    ylabels = ["Spin (°/s)", "Tilt (°)"]
    return generate_plot(data, ydata, ylabels, "Degrees")


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
    html += generate_motion_plot(data)
    html += generate_alti_plot(data)
    html += generate_orientation_plot(data)
    html += f"""
</html>"""

    with open(
        f'Summary-{data[0]["SystemName"]}-{data[0]["LaunchTime"].strftime("%Y.%m.%d.%I.%M%p")}.html',
        "w",
    ) as htmlfile:
        htmlfile.write(html)


filename = sys.argv[1]
parsed_csv = parse_csv(filename)
fixed_csv = fix_timestamp(parsed_csv)
write_html(fixed_csv)
write_kml(fixed_csv)
