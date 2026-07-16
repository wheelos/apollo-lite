#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interactive Ground Segmentation Visualizer Web Tool
Calls REAL C++ ground segmentation algorithm via ground_analyzer
Copyright 2026 The WheelOS Team. All Rights Reserved.
"""

import argparse
import os
import sys
import subprocess
import json
from http.server import HTTPServer, SimpleHTTPRequestHandler
import webbrowser
import threading
import time
from urllib.parse import urlparse, parse_qs

# Path to the C++ ground_analyzer tool
def get_ground_analyzer_path():
    # Try to find the ground_analyzer binary
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Try common build output paths
    possible_paths = [
        os.path.join(script_dir, '../../bazel-bin/modules/perception/lidar/tools/ground_visualizer/ground_analyzer'),
        '/apollo/bazel-bin/modules/perception/lidar/tools/ground_visualizer/ground_analyzer',
        os.path.join(script_dir, 'ground_analyzer'),
    ]

    for path in possible_paths:
        if os.path.exists(path):
            return path

    # Default to apollo path
    return '/apollo/bazel-bin/modules/perception/lidar/tools/ground_visualizer/ground_analyzer'

def run_ground_segmentation(pcd_path, config):
    """Run C++ ground segmentation via ground_analyzer tool"""

    # Get the actual path to the tool
    tool_path = get_ground_analyzer_path()

    # Check if tool exists
    if not os.path.exists(tool_path):
        return {'error': f'Ground analyzer tool not found at {tool_path}. Build it first with: bazel build //modules/perception/lidar/tools/ground_visualizer:ground_analyzer'}

    # Build command
    cmd = [
        tool_path,
        pcd_path,
        '--json',
        '--small_grid_size', str(config.get('small_grid_size', 64)),
        '--big_grid_size', str(config.get('big_grid_size', 512)),
        '--grid_size', str(config.get('grid_size', 32)),
        '--nr_smooth_iter', str(config.get('nr_smooth_iter', 6)),
        '--roi_rad_x', str(config.get('roi_rad_x', 120.0)),
        '--roi_rad_y', str(config.get('roi_rad_y', 120.0)),
        '--roi_rad_z', str(config.get('roi_rad_z', 100.0)),
        '--roi_near_rad', str(config.get('roi_near_rad', 32.0)),
        '--sample_region_z_lower', str(config.get('sample_region_z_lower', -3.0)),
        '--sample_region_z_upper', str(config.get('sample_region_z_upper', -1.0)),
        '--planefit_dist_thres_near', str(config.get('planefit_dist_thres_near', 1.0)),
        '--planefit_dist_thres_far', str(config.get('planefit_dist_thres_far', 1.5)),
        '--planefit_orien_threshold', str(config.get('planefit_orien_threshold', 30.0)),
        '--inliers_min_threshold', str(config.get('inliers_min_threshold', 3)),
        '--z_compare_thres', str(config.get('z_compare_thres', 0.3)),
        '--smooth_z_thres', str(config.get('smooth_z_thres', 2.0)),
        '--ground_thres', str(config.get('ground_thres', 3.0)),
        '--near_range_dist', str(config.get('near_range_dist', 10.0)),
        '--near_range_ground_thres', str(config.get('near_range_ground_thres', 3.0)),
        '--middle_range_dist', str(config.get('middle_range_dist', 20.0)),
        '--middle_range_ground_thres', str(config.get('middle_range_ground_thres', 3.0)),
        '--vehicle_x', str(config.get('vehicle_x', 0.0)),
        '--vehicle_y', str(config.get('vehicle_y', 0.0)),
        '--vehicle_z', str(config.get('vehicle_z', 0.0)),
    ]

    # Print command for debugging
    print(f"Running: {' '.join(cmd)}")

    try:
        # Run C++ tool and capture output
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)

        if result.returncode != 0:
            return {'error': f'Ground analyzer failed: {result.stderr}'}

        # Parse JSON output
        return json.loads(result.stdout)

    except subprocess.TimeoutExpired:
        return {'error': 'Ground analyzer timeout'}
    except json.JSONDecodeError as e:
        return {'error': f'Failed to parse output: {str(e)}'}
    except Exception as e:
        return {'error': str(e)}

class PCDRequestHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        parsed = urlparse(self.path)

        # Serve static files
        if parsed.path == '/' or parsed.path == '/visualizer.html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()

            html_path = os.path.join(os.path.dirname(__file__), 'visualizer.html')
            with open(html_path, 'r') as f:
                self.wfile.write(f.read().encode())
            return

        # API endpoint to run ground segmentation
        if parsed.path.startswith('/api/segment'):
            params = parse_qs(parsed.query)
            pcd_path = params.get('path', [''])[0]

            # Get config parameters with error handling
            try:
                config = {
                    'small_grid_size': int(float(params.get('small_grid_size', [64])[0])),
                    'big_grid_size': int(float(params.get('big_grid_size', [512])[0])),
                    'grid_size': int(float(params.get('grid_size', [32])[0])),
                    'nr_smooth_iter': int(float(params.get('nr_smooth_iter', [6])[0])),
                    'roi_rad_x': float(params.get('roi_rad_x', [120.0])[0]),
                    'roi_rad_y': float(params.get('roi_rad_y', [120.0])[0]),
                    'roi_rad_z': float(params.get('roi_rad_z', [100.0])[0]),
                    'roi_near_rad': float(params.get('roi_near_rad', [32.0])[0]),
                    'sample_region_z_lower': float(params.get('sample_region_z_lower', [-3.0])[0]),
                    'sample_region_z_upper': float(params.get('sample_region_z_upper', [-1.0])[0]),
                    'planefit_dist_thres_near': float(params.get('planefit_dist_thres_near', [1.0])[0]),
                    'planefit_dist_thres_far': float(params.get('planefit_dist_thres_far', [1.5])[0]),
                    'planefit_orien_threshold': float(params.get('planefit_orien_threshold', [30.0])[0]),
                    'inliers_min_threshold': int(float(params.get('inliers_min_threshold', [3])[0])),
                    'z_compare_thres': float(params.get('z_compare_thres', [0.3])[0]),
                    'smooth_z_thres': float(params.get('smooth_z_thres', [2.0])[0]),
                    'ground_thres': float(params.get('ground_thres', [3.0])[0]),
                    'near_range_dist': float(params.get('near_range_dist', [10.0])[0]),
                    'near_range_ground_thres': float(params.get('near_range_ground_thres', [3.0])[0]),
                    'middle_range_dist': float(params.get('middle_range_dist', [20.0])[0]),
                    'middle_range_ground_thres': float(params.get('middle_range_ground_thres', [3.0])[0]),
                    # Vehicle pose parameters
                    'vehicle_x': float(params.get('vehicle_x', [0.0])[0]),
                    'vehicle_y': float(params.get('vehicle_y', [0.0])[0]),
                    'vehicle_z': float(params.get('vehicle_z', [0.0])[0]),
                }
            except (ValueError, IndexError) as e:
                self.send_response(400)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'error': f'Invalid parameter: {str(e)}'}).encode())
                return

            if not pcd_path or not os.path.exists(pcd_path):
                self.send_response(404)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({'error': 'File not found'}).encode())
                return

            try:
                # Run ground segmentation using C++ tool
                result = run_ground_segmentation(pcd_path, config)

                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps(result).encode())
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({'error': str(e)}).encode())
            return

        # Default: serve file from directory
        super().do_GET()

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

def start_web_server(port=8080):
    """Start web server"""
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    server = HTTPServer(('localhost', port), PCDRequestHandler)
    print(f"Web server started at http://localhost:{port}")
    server.serve_forever()

def main():
    parser = argparse.ArgumentParser(description="Interactive Ground Segmentation Visualizer")
    parser.add_argument("pcd_file", nargs='?', help="Path to PCD file to visualize")
    parser.add_argument("--port", type=int, default=8080, help="Port for web server")
    parser.add_argument("--no-browser", action="store_true", help="Don't open browser automatically")

    args = parser.parse_args()

    # Start web server in background
    server_thread = threading.Thread(target=start_web_server, args=(args.port,), daemon=True)
    server_thread.start()

    # Open browser
    time.sleep(1)
    if not args.no_browser:
        url = f"http://localhost:{args.port}/visualizer.html"
        if args.pcd_file:
            url += f"?pcd_path={args.pcd_file}"
        print(f"Opening browser at: {url}")
        webbrowser.open(url)

    print("\n" + "="*60)
    print("Ground Segmentation Visualizer")
    print("="*60)
    print(f"\nWeb interface available at: http://localhost:{args.port}/visualizer.html")
    print("\nThis tool calls the REAL C++ ground segmentation algorithm!")
    print(f"Ground analyzer path: {get_ground_analyzer_path()}")
    print("\nPress Ctrl+C to stop the server")
    print("="*60 + "\n")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")

if __name__ == "__main__":
    sys.exit(main())
