#!/usr/bin/env python3
"""Download Mapbox satellite tiles for a lat/lon box at ONE zoom and write a georeferenced GeoTIFF.

Needs only requests + Pillow + numpy (no GDAL/rasterio). Output is EPSG:3857 (web mercator), RGB.

  export MAPBOX_TOKEN=pk.xxxxx
  python3 mapbox_to_geotiff.py --bbox MIN_LON MIN_LAT MAX_LON MAX_LAT --zoom 18 --out site_z18.tif

Tiles are cached in --cache (default ~/map_tiles/mapbox/<z>) so reruns do not re-download.
Mapbox terms limit bulk downloading/storing: keep the box small and read the current ToS.
The box is rounded OUT to whole tiles, so the file covers slightly more than asked.
"""
import argparse, math, os, sys, time
import numpy as np
import requests
from PIL import Image, TiffImagePlugin

Image.MAX_IMAGE_PIXELS = None
ORIGIN = 20037508.342789244      # web-mercator half-extent, metres
TILE_PX = 512                    # we request @2x tiles


def lonlat_to_tile(lon, lat, z):
    n = 2 ** z
    lat = max(min(lat, 85.0511), -85.0511)
    x = int((lon + 180.0) / 360.0 * n)
    y = int((1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n)
    return min(max(x, 0), n - 1), min(max(y, 0), n - 1)


def fetch(z, x, y, token, cache, style):
    path = os.path.join(cache, "%d_%d.jpg" % (x, y))
    if os.path.exists(path) and os.path.getsize(path) > 0:
        return path
    url = "https://api.mapbox.com/v4/%s/%d/%d/%d@2x.jpg" % (style, z, x, y)
    for attempt in range(4):
        r = requests.get(url, params={"access_token": token}, timeout=20)
        if r.status_code == 200:
            with open(path, "wb") as f:
                f.write(r.content)
            return path
        if r.status_code in (429, 500, 502, 503):
            time.sleep(2 * (attempt + 1))
            continue
        sys.exit("tile %d/%d/%d failed: HTTP %d %s" % (z, x, y, r.status_code, r.text[:200]))
    sys.exit("tile %d/%d/%d failed after retries" % (z, x, y))


def write_geotiff(arr, out, x_left, y_top, res):
    ifd = TiffImagePlugin.ImageFileDirectory_v2()
    ifd[33550] = (res, res, 0.0)                       # ModelPixelScale
    ifd.tagtype[33550] = 12
    ifd[33922] = (0.0, 0.0, 0.0, x_left, y_top, 0.0)    # ModelTiepoint: pixel (0,0) -> map (x,y)
    ifd.tagtype[33922] = 12
    keys = [1, 1, 0, 3,                                 # GeoKey directory header, 3 keys
            1024, 0, 1, 1,                              # GTModelType = projected
            1025, 0, 1, 1,                              # GTRasterType = PixelIsArea
            3072, 0, 1, 3857]                           # ProjectedCSType = EPSG:3857
    ifd[34735] = tuple(keys)
    ifd.tagtype[34735] = 3
    Image.fromarray(arr).save(out, format="TIFF", tiffinfo=ifd, compression="tiff_adobe_deflate")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bbox", nargs=4, type=float, required=True, metavar=("MIN_LON", "MIN_LAT", "MAX_LON", "MAX_LAT"))
    ap.add_argument("--zoom", type=int, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--style", default="mapbox.satellite")
    ap.add_argument("--cache", default=None)
    ap.add_argument("--max-tiles", type=int, default=400, help="safety limit (default 400)")
    ap.add_argument("--dry-run", action="store_true", help="only print tile count and size")
    a = ap.parse_args()

    min_lon, min_lat, max_lon, max_lat = a.bbox
    if not (min_lon < max_lon and min_lat < max_lat):
        sys.exit("bbox must be MIN_LON MIN_LAT MAX_LON MAX_LAT with min < max")
    z = a.zoom
    x0, y1 = lonlat_to_tile(min_lon, min_lat, z)   # SW corner -> max y
    x1, y0 = lonlat_to_tile(max_lon, max_lat, z)   # NE corner -> min y
    nx, ny = x1 - x0 + 1, y1 - y0 + 1
    res = 2 * ORIGIN / (TILE_PX * 2 ** z)
    mid_lat = 0.5 * (min_lat + max_lat)
    print("zoom %d: %d x %d = %d tiles, %.3f m/px at equator (%.3f at lat %.1f), image %d x %d px (%.0f MB raw)" % (
        z, nx, ny, nx * ny, res, res * math.cos(math.radians(mid_lat)), mid_lat,
        nx * TILE_PX, ny * TILE_PX, nx * ny * TILE_PX * TILE_PX * 3 / 1e6))
    if a.dry_run:
        return
    if nx * ny > a.max_tiles:
        sys.exit("%d tiles exceeds --max-tiles %d; shrink the box or raise the limit" % (nx * ny, a.max_tiles))
    token = os.environ.get("MAPBOX_TOKEN")
    if not token:
        sys.exit("set MAPBOX_TOKEN")

    cache = a.cache or os.path.expanduser("~/map_tiles/mapbox/%d" % z)
    os.makedirs(cache, exist_ok=True)
    canvas = np.zeros((ny * TILE_PX, nx * TILE_PX, 3), dtype=np.uint8)
    n = 0
    for ty in range(y0, y1 + 1):
        for tx in range(x0, x1 + 1):
            p = fetch(z, tx, ty, token, cache, a.style)
            im = Image.open(p).convert("RGB")
            if im.size != (TILE_PX, TILE_PX):
                im = im.resize((TILE_PX, TILE_PX))
            canvas[(ty - y0) * TILE_PX:(ty - y0 + 1) * TILE_PX, (tx - x0) * TILE_PX:(tx - x0 + 1) * TILE_PX] = np.asarray(im)
            n += 1
            if n % 20 == 0:
                print("  %d/%d tiles" % (n, nx * ny), flush=True)

    tile_m = 2 * ORIGIN / 2 ** z
    x_left = -ORIGIN + x0 * tile_m
    y_top = ORIGIN - y0 * tile_m
    write_geotiff(canvas, a.out, x_left, y_top, res)
    print("wrote %s  (EPSG:3857, top-left %.2f, %.2f, %.4f m/px)" % (a.out, x_left, y_top, res))


if __name__ == "__main__":
    main()
