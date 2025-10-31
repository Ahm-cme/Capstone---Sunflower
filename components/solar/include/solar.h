#pragma once
#include <stdbool.h>
#include <time.h>

/*
    Solar Position & Events Module

    What this module provides:
    - solar_compute: Calculate sun azimuth/elevation at any location and time
    - solar_events: Find sunrise/sunset times for any day and location
    - solar_julian_day: Convert UTC time to Julian Day (for advanced calculations)

    Algorithms:
    - Based on simplified NOAA solar position equations
    - Accuracy: ≈0.1° (sufficient for solar tracking applications)
    - Valid for years 1900-2100 (outside this range, accuracy degrades)
    - Handles polar regions (midnight sun / polar night detection)

    Coordinate System:
    - Azimuth: 0°=North, 90°=East, 180°=South, 270°=West (clockwise from North)
    - Elevation: 0°=horizon, 90°=zenith, -90°=nadir (negative = below horizon)
    - All angles in decimal degrees
    - All times in UTC (Unix epoch seconds)

    Sunrise/Sunset Conventions:
    - Uses -0.833° apparent elevation for civil sunrise/sunset
    - Includes atmospheric refraction (≈0.57°) + solar radius (≈0.27°)
    - Does NOT account for local terrain or obstructions
    - Results are for sea level observer

    Usage Example:
    ```c
    // Get current sun position
    time_t now = time(NULL);
    sun_pos_t sun = solar_compute(43.6532, -79.3832, now);  // Toronto
    printf("Sun: Az=%.1f° El=%.1f°\n", sun.azimuth_deg, sun.elevation_deg);
    
    // Get today's sunrise/sunset
    solar_events_t events = solar_events(43.6532, -79.3832, now);
    if (events.has_sunrise) {
        printf("Sunrise: %s", ctime(&events.sunrise_utc));
    }
    ```

    Integration with Tracking System:
    - Called every tracking cycle (1-60 seconds)
    - Provides target angles for motor control
    - Used to determine sleep/wake schedule
    - Logged to SD card for analysis

    Notes:
    - All calculations use UTC internally (no timezone conversions)
    - Caller must handle local timezone offsets if needed
    - Polar regions: has_sunrise/has_sunset flags indicate validity
    - High latitudes (>66.5°): expect midnight sun/polar night seasonally
*/

typedef struct {
    double azimuth_deg;     // 0..360 from North, clockwise (0=N, 90=E, 180=S, 270=W)
    double elevation_deg;   // -90..+90 relative to horizon (0=horizon, 90=zenith)
    bool is_daylight;       // true if elevation > 0° (sun above horizon)
} sun_pos_t;

/*
    Compute sun position for a location and time (UTC).

    What it does:
    - Calculates sun's azimuth and elevation angles in local horizon coordinates
    - Uses observer's latitude/longitude and current UTC time
    - Based on NOAA solar position algorithm (simplified for performance)
    - Returns Earth-fixed coordinates (not mount-relative)

    Accuracy:
    - Typical error: <0.1° for dates 1900-2100
    - Sufficient for solar tracking (±0.5° is acceptable)
    - More accurate near solar noon
    - Slightly less accurate near horizon (atmospheric refraction varies)

    Inputs:
    - lat_deg: Observer latitude, decimal degrees [-90 .. +90]
              Positive = North, Negative = South
    - lon_deg: Observer longitude, decimal degrees [-180 .. +180]
              Positive = East, Negative = West
    - t_utc:  UTC time (Unix epoch seconds, from time(NULL))

    Returns:
    - sun_pos_t structure with:
      * azimuth_deg: Horizontal angle from North (0-360°)
      * elevation_deg: Vertical angle above horizon (-90 to +90°)
      * is_daylight: true if sun is above horizon (elevation > 0°)

    Example:
    ```c
    time_t now = time(NULL);
    sun_pos_t sun = solar_compute(43.6532, -79.3832, now);  // Toronto
    printf("Azimuth: %.2f°, Elevation: %.2f°\n", 
           sun.azimuth_deg, sun.elevation_deg);
    ```

    Notes:
    - Called frequently (every 1-60 seconds during tracking)
    - Very fast (< 1ms on ESP32)
    - Thread-safe (no static state)
*/
sun_pos_t solar_compute(double lat_deg, double lon_deg, time_t t_utc);

/*
    Convert broken-down UTC time to Julian Day (fractional).

    What it is:
    - Julian Day (JD) is a continuous count of days since noon UTC
      on January 1, 4713 BC (proleptic Julian calendar)
    - J2000.0 epoch (Jan 1, 2000, 12:00 UTC) = JD 2451545.0
    - Used internally for orbital mechanics calculations

    Why you might need this:
    - Advanced solar calculations (equation of time, etc.)
    - Comparing dates across different calendar systems
    - Accurate time interval calculations

    Inputs:
    - utc: Pointer to struct tm in UTC (from gmtime())

    Returns:
    - Julian Day number as fractional days (e.g., JD 2451545.5 = noon on Jan 2, 2000)

    Example:
    ```c
    time_t now = time(NULL);
    struct tm *utc_tm = gmtime(&now);
    double jd = solar_julian_day(utc_tm);
    printf("Julian Day: %.5f\n", jd);
    ```

    Notes:
    - Valid for Gregorian calendar dates (post Oct 15, 1582)
    - Includes fractional day (0.0 = midnight, 0.5 = noon)
    - Used internally by solar_compute() and solar_events()
*/
double solar_julian_day(const struct tm *utc);

typedef struct {
    time_t sunrise_utc;     // UTC epoch seconds of sunrise
    time_t sunset_utc;      // UTC epoch seconds of sunset
    bool has_sunrise;       // false during polar night (sun never rises)
    bool has_sunset;        // false during midnight sun (sun never sets)
} solar_events_t;

/*
    Compute sunrise and sunset for the UTC calendar day containing t_utc.

    What it does:
    - Calculates civil sunrise/sunset times (sun crosses -0.833° elevation)
    - Includes atmospheric refraction (0.57°) + solar radius (0.27°)
    - Handles polar regions (midnight sun / polar night)
    - Returns UTC timestamps (no timezone conversion)

    Method:
    - Standard NOAA approximation using equation of time
    - Solves for hour angle when sun elevation = -0.833°
    - Assumes flat horizon (ignores local terrain)
    - Valid for all latitudes (with polar day/night handling)

    Civil Twilight:
    - Sunrise: moment top of sun appears above horizon
    - Sunset: moment top of sun disappears below horizon
    - Different from nautical (-6°) or astronomical (-18°) twilight
    - Good enough light for outdoor activities without artificial lighting

    Polar Handling:
    - has_sunrise/has_sunset indicate validity of timestamps
    - Midnight sun (summer): both flags false, sun never sets
    - Polar night (winter): both flags false, sun never rises
    - Check flags before using sunrise_utc/sunset_utc values!

    Inputs:
    - lat_deg: Observer latitude [-90 .. +90]
    - lon_deg: Observer longitude [-180 .. +180]
    - t_utc: Any UTC time on the target day (Unix epoch seconds)

    Returns:
    - solar_events_t structure with:
      * sunrise_utc: Sunrise time (if has_sunrise == true)
      * sunset_utc: Sunset time (if has_sunset == true)
      * has_sunrise: false during polar night
      * has_sunset: false during midnight sun

    Example:
    ```c
    time_t now = time(NULL);
    solar_events_t events = solar_events(43.6532, -79.3832, now);
    
    if (events.has_sunrise) {
        struct tm *sunrise_tm = localtime(&events.sunrise_utc);
        printf("Sunrise: %02d:%02d\n", sunrise_tm->tm_hour, sunrise_tm->tm_min);
    } else {
        printf("No sunrise today (polar night)\n");
    }
    ```

    Notes:
    - Called once per day (at midnight or startup)
    - Results cached for display and sleep scheduling
    - Fast calculation (< 2ms on ESP32)
    - Times are for sea level observer (add terrain corrections if needed)

    Accuracy:
    - Typical error: ±2 minutes for mid-latitudes
    - Larger errors near polar circles due to rapid twilight
    - Atmospheric pressure/temperature variations can shift by ±2 minutes
*/
solar_events_t solar_events(double lat_deg, double lon_deg, time_t t_utc);