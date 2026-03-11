"""
geo_utils.py — Geodetic helper functions.
==========================================
All distance / offset calculations use a flat-Earth approximation
valid for areas up to a few kilometres (error < 0.01 %).
"""

import math

_EARTH_R    = 6_371_000.0                        # mean radius (m)
_DEG_LAT_M  = _EARTH_R * math.pi / 180.0         # metres per degree latitude ≈ 111 195 m/°


def deg_lon_m(lat_deg: float) -> float:
    """Metres per degree of longitude at the given latitude."""
    return _EARTH_R * math.pi / 180.0 * math.cos(math.radians(lat_deg))


def latlon_to_xy(lat: float, lon: float,
                 origin_lat: float, origin_lon: float,
                 dlat_m: float, dlon_m: float) -> tuple[float, float]:
    """
    Convert (lat, lon) to local East/North (x, y) metres
    relative to the given origin.
    """
    x = (lon - origin_lon) * dlon_m   # East  (+x)
    y = (lat - origin_lat) * dlat_m   # North (+y)
    return x, y


def xy_to_latlon(x: float, y: float,
                 origin_lat: float, origin_lon: float,
                 dlat_m: float, dlon_m: float) -> tuple[float, float]:
    """Inverse of latlon_to_xy."""
    lat = origin_lat + y / dlat_m
    lon = origin_lon + x / dlon_m
    return lat, lon


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres between two (lat, lon) points."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi  = math.radians(lat2 - lat1)
    dlam  = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return 2 * _EARTH_R * math.asin(math.sqrt(a))
