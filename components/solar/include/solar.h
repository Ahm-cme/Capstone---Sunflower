#pragma once
#include <stdbool.h>
#include <time.h>

/*
    Solar Position & Events

    What this module provides:
    - solar_compute: sun azimuth/elevation at a given lat/lon and UTC time
    - solar_events: sunrise/sunset times for a given day and location
    - solar_julian_day: UTC -> Julian Day helper

    Conventions:
    - Angles in degrees.
    - Azimuth: 0°=North, 90°=East, 180°=South, 270°=West.
    - Elevation: 0°=horizon, 90°=zenith (negative = below horizon).
    - Times are UTC epoch seconds (time_t). Callers must handle local timezones.

    Accuracy/assumptions:
    - Based on simplified NOAA-style equations; suitable for tracking (≈0.1° class).
    - Sunrise/sunset use -0.833° apparent elevation (refraction + solar radius).
    - Valid for typical project years; extreme epochs or polar regions require care.

    Coordinate frames:
    - All outputs are Earth-fixed (not mount coordinates).
    - Tracking applies mount offsets separately.
*/

typedef struct {
    double azimuth_deg;     // 0..360 from North, clockwise
    double elevation_deg;   // -90..+90 relative to horizon
    bool is_daylight;       // elevation_deg > 0
} sun_pos_t;

/*
    Compute sun position for a location and time (UTC).

    Inputs:
    - lat_deg: latitude  [-90 .. +90]
    - lon_deg: longitude [-180 .. +180] (East positive)
    - t_utc:  UTC epoch seconds

    Returns:
    - sun_pos_t with azimuth_deg, elevation_deg, and is_daylight flag.
*/
sun_pos_t solar_compute(double lat_deg, double lon_deg, time_t t_utc);

/*
    Convert broken-down UTC time to Julian Day (fractional).

    Inputs:
    - utc: struct tm in UTC

    Returns:
    - Julian Day number (e.g., J2000.0 = 2451545.0)
*/
double solar_julian_day(const struct tm *utc);

typedef struct {
    time_t sunrise_utc;     // UTC epoch seconds
    time_t sunset_utc;      // UTC epoch seconds
    bool has_sunrise;       // false during polar night
    bool has_sunset;        // false during midnight sun
} solar_events_t;

/*
    Compute sunrise and sunset for the UTC calendar day containing t_utc.

    Method:
    - Standard NOAA approximation at -0.833° apparent elevation.
    - Includes average refraction and solar radius.
    - Ignores local terrain/obstructions.

    Polar handling:
    - has_sunrise/has_sunset indicate validity of timestamps.

    Inputs:
    - lat_deg, lon_deg: observer position
    - t_utc: any UTC time on the target day

    Returns:
    - solar_events_t with UTC timestamps and validity flags.
*/
solar_events_t solar_events(double lat_deg, double lon_deg, time_t t_utc);