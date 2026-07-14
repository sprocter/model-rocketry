"""Utility functions (designed to run on a PC) for use with the Nene model rocket flight computer.

--------------------------------------------------------------------------------
Copyright (C) 2026 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
"""

import sys

def dm2dd(dm : str) -> str:
    dd = float(dm[0:2])
    mm = float(dm[2:]) / 60
    return str(dd + mm)

def csv_to_kml(filename: str) -> None:
    import csv
    print("""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Nene Flightpath</name>
    <Placemark>
      <name>Flight Path</name>
      <LineString>
        <altitudeMode>relativeToGround</altitudeMode>
        <coordinates>""", end='')
    with open(filename, newline='') as csvfile:
        next(csvfile) # Skip first row
        cread = csv.DictReader(csvfile)
        for row in cread:
            lon = dm2dd(row[' lon(ddmm.mmmm)'].strip())
            lat = dm2dd(row[' lat (ddmm.mmmm)'].strip())
            alt = row[' baro_alt (m)'].strip()
            print(f"-{lon},{lat},{alt}", end=' ')
    print("""</coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>""")

csv_to_kml(sys.argv[1])