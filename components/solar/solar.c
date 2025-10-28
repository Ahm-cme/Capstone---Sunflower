#include "solar.h"
#include <math.h>
#include <time.h>
#include "esp_log.h"

/*
    Solar position and sunrise/sunset calculations.

    Basis:
    - Simplified NOAA equations (sufficient for tracking accuracy).
    https://gml.noaa.gov/grad/solcalc/solareqns.PDF   

    Units/conventions:
    - Inputs: latitude/longitude in degrees, UTC time_t.
    - Azimuth: 0°=North, clockwise 0..360.
    - Elevation: degrees above horizon (negative = below).
*/

#define TAG "SOLAR"

// Degrees ↔ radians helpers
static double deg2rad(double d){ return d * M_PI / 180.0; }
static double rad2deg(double r){ return r * 180.0 / M_PI; }

// Clamp value to [lo, hi]
static double clamp(double v, double lo, double hi){
    return v < lo ? lo : (v > hi ? hi : v);
}

/*
    Compute UTC midnight (day start) for t and the day-of-year (1..366).
    Used by sunrise/sunset so all math stays within one UTC day.
*/
static void utc_day_start(time_t t, time_t *day0_out, int *yday_out){
    struct tm *utc = gmtime(&t);                            // Break down to UTC components

    // Seconds since midnight = h*3600 + m*60 + s
    time_t day0 = t - (utc->tm_hour * 3600 + utc->tm_min * 60 + utc->tm_sec);

    if (day0_out) *day0_out = day0;                         // Return day start epoch
    if (yday_out) *yday_out = utc->tm_yday + 1;             // tm_yday is 0-based

    ESP_LOGV(TAG, "UTC day start: %ld (day %d of year)", (long)day0, utc->tm_yday + 1);
}

/*
    Convert broken-down UTC time to Julian Day (fractional).
    Valid for Gregorian dates (post 1582-10-15).
*/
double solar_julian_day(const struct tm *utc){
    int y = utc->tm_year + 1900;                            // Year (YYYY)
    int m = utc->tm_mon + 1;                                // Month (1..12)
    int d = utc->tm_mday;                                   // Day of month

    // Jan/Feb are treated as months 13/14 of previous year
    if (m <= 2){ y--; m += 12; }

    int A = y / 100;                                        // Century
    int B = 2 - A + A / 4;                                  // Gregorian correction

    // Fraction of day (0 at midnight)
    double dayfrac = (utc->tm_hour + utc->tm_min / 60.0 + utc->tm_sec / 3600.0) / 24.0;

    // JD formula (midnight-based)
    double JD = floor(365.25 * (y + 4716)) +
                floor(30.6001 * (m + 1)) +
                d + dayfrac + B - 1524.5;

    ESP_LOGV(TAG, "Julian Day: %04d-%02d-%02d %02d:%02d:%02d UTC → %.3f",
             utc->tm_year + 1900, utc->tm_mon + 1, utc->tm_mday,
             utc->tm_hour, utc->tm_min, utc->tm_sec, JD);

    return JD;
}

/*
    Sun position at (lat, lon) for UTC time t_utc.

    Steps:
    - Convert to Julian Day and days since J2000.0.
    - Compute mean anomaly M and equation of center C.
    - Ecliptic longitude λ = M + C + 180° + perihelion (102.9372°).
    - Declination δ from λ and obliquity (≈23.44°).
    - Local solar time from UTC hour + longitude.
    - Hour angle H, then elevation/azimuth in local horizon frame.
*/
sun_pos_t solar_compute(double lat_deg, double lon_deg, time_t t_utc){
    sun_pos_t s = {0};

    struct tm *utc = gmtime(&t_utc);                        // Use UTC for all math
    double JD = solar_julian_day(utc);                      // Julian Day

    double n = JD - 2451545.0;                              // Days since J2000.0
    ESP_LOGD(TAG, "Days since J2000.0: %.3f", n);

    double M = fmod(357.5291 + 0.98560028 * n, 360.0);      // Mean anomaly (deg)
    ESP_LOGV(TAG, "Mean anomaly: %.3f°", M);

    double C = 1.9148 * sin(deg2rad(M)) +                   // Equation of center (deg)
               0.02   * sin(deg2rad(2 * M)) +
               0.0003 * sin(deg2rad(3 * M));
    ESP_LOGV(TAG, "Equation of center: %.4f°", C);

    double lambda = fmod(M + C + 180.0 + 102.9372, 360.0);  // Apparent ecliptic longitude (deg)
    ESP_LOGV(TAG, "Solar longitude: %.3f°", lambda);

    double delta = asin(sin(deg2rad(lambda)) *              // Declination (rad)
                        sin(deg2rad(23.44)));
    ESP_LOGV(TAG, "Solar declination: %.3f°", rad2deg(delta));

    double lst = utc->tm_hour +                              // Local solar time (hours)
                 utc->tm_min / 60.0 +
                 utc->tm_sec / 3600.0 +
                 lon_deg / 15.0;
    ESP_LOGV(TAG, "Local Solar Time: %.3f hours", lst);

    double H = deg2rad((lst - 12.0) * 15.0);                // Hour angle (rad; 0 at solar noon)
    ESP_LOGV(TAG, "Hour angle: %.3f° (%.3f rad)", rad2deg(H), H);

    double lat = deg2rad(lat_deg);                          // Observer latitude (rad)

    // Elevation: asin(sin δ sin φ + cos δ cos φ cos H)
    double elev = asin(sin(delta) * sin(lat) + cos(delta) * cos(lat) * cos(H));
    s.elevation_deg = rad2deg(elev);

    // Azimuth (from North, clockwise): atan2(sin H, cos H sin φ − tan δ cos φ)
    double az = atan2(sin(H), cos(H) * sin(lat) - tan(delta) * cos(lat));
    s.azimuth_deg = fmod(rad2deg(az) + 180.0 + 360.0, 360.0); // Map to 0..360

    s.is_daylight = s.elevation_deg > 0.0;                  // Simple daylight flag

    ESP_LOGD(TAG, "Solar position: Az=%.2f° El=%.2f° (daylight=%s) at %.3f,%.3f",
             s.azimuth_deg, s.elevation_deg, s.is_daylight ? "yes" : "no", lat_deg, lon_deg);

    return s;
}

/*
    Sunrise/sunset for the UTC calendar day containing t_utc.

    Steps:
    - Compute day-of-year and equation of time (EoT).
    - Daily declination from series expansion in γ (day angle).
    - Solve for hour angle at -0.833° apparent elevation (h0).
    - Convert solar noon (minutes from midnight) ± hour-angle minutes to rise/set.
    - Handle polar day/night by checking cosH0 bounds.
*/
solar_events_t solar_events(double lat_deg, double lon_deg, time_t t_utc){
    solar_events_t ev = {0};

    time_t day0; int yday;
    utc_day_start(t_utc, &day0, &yday);                     // UTC midnight and day-of-year

    ESP_LOGD(TAG, "Computing sunrise/sunset for day %d at %.4f,%.4f", yday, lat_deg, lon_deg);

    double gamma = 2.0 * M_PI / 365.0 * (yday - 1);         // Day angle (rad)
    ESP_LOGV(TAG, "Gamma (day angle): %.4f rad", gamma);

    // Equation of time (minutes)
    double EoT = 229.18 * (0.000075 +
                           0.001868 * cos(gamma) - 0.032077 * sin(gamma) -
                           0.014615 * cos(2 * gamma) - 0.040849 * sin(2 * gamma));
    ESP_LOGV(TAG, "Equation of time: %.2f minutes", EoT);

    // Declination (rad) for the day
    double decl = 0.006918 - 0.399912 * cos(gamma) + 0.070257 * sin(gamma) -
                  0.006758 * cos(2 * gamma) + 0.000907 * sin(2 * gamma) -
                  0.002697 * cos(3 * gamma) + 0.00148  * sin(3 * gamma);
    ESP_LOGV(TAG, "Solar declination: %.4f rad (%.2f°)", decl, rad2deg(decl));

    // Apparent sunrise/sunset elevation threshold
    double lat = deg2rad(lat_deg);
    double h0  = deg2rad(-0.833);                           // -0.833° (refraction + solar radius)

    // cos(H0) at rise/set where elevation = h0
    double cosH0 = (sin(h0) - sin(lat) * sin(decl)) / (cos(lat) * cos(decl));

    // Polar conditions: sun never rises/sets
    if (cosH0 > 1.0) {                                      // Always below h0
        ESP_LOGD(TAG, "Polar night: cosH0=%.3f > 1.0", cosH0);
        ev.has_sunrise = false;
        ev.has_sunset = false;
        return ev;
    }
    if (cosH0 < -1.0) {                                     // Always above h0
        ESP_LOGD(TAG, "Midnight sun: cosH0=%.3f < -1.0", cosH0);
        ev.has_sunrise = false;
        ev.has_sunset = false;
        return ev;
    }

    double H0 = acos(clamp(cosH0, -1.0, 1.0));              // Hour angle magnitude (rad)
    double H0_min = 4.0 * rad2deg(H0);                      // 1° = 4 minutes
    ESP_LOGV(TAG, "Sunrise hour angle: %.4f rad (%.2f°, %.1f min)", H0, rad2deg(H0), H0_min);

    double noon_min = 720.0 - 4.0 * lon_deg - EoT;          // Minutes from midnight to solar noon
    ESP_LOGV(TAG, "Solar noon: %.1f minutes from UTC midnight", noon_min);

    double rise_min = noon_min - H0_min;                    // Sunrise minutes from midnight
    double set_min  = noon_min + H0_min;                    // Sunset minutes from midnight

    ESP_LOGD(TAG, "Sunrise: %.1f min (%.0f:%02.0f), Sunset: %.1f min (%.0f:%02.0f)",
             rise_min, floor(rise_min / 60), fmod(rise_min, 60),
             set_min,  floor(set_min  / 60), fmod(set_min,  60));

    int rise_sec = (int)lrint(rise_min * 60.0);             // To seconds
    int set_sec  = (int)lrint(set_min  * 60.0);

    ev.sunrise_utc = day0 + rise_sec;                       // Epoch seconds
    ev.sunset_utc  = day0 + set_sec;
    ev.has_sunrise = true;
    ev.has_sunset  = true;

    ESP_LOGD(TAG, "Final times: sunrise=%ld, sunset=%ld",
             (long)ev.sunrise_utc, (long)ev.sunset_utc);

    return ev;
}