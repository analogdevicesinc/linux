#!/usr/bin/env python3
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)

########################################################################
#
# File name: upload_to_artifactory_parallel.py
# Author: Liviu Tomoiaga (based on upload_to_artifactory.py by Raus Stefan)
#
# This script uploads files to ADI internal Artifactory server using
# parallel uploads for improved performance. Based on the sequential
# version, with parallelism modeled after upload_to_cloudsmith_parallel.py.
#
# Copyright (C) 2019-2026 Analog Devices Inc.
#
########################################################################

import argparse
import os
import signal
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from threading import Lock

# Thread-safe print
print_lock = Lock()

# Shutdown flag for graceful termination
shutdown_requested = False

# Whitelist of allowed server folders
SERVER_FOLDERS_LIST = ["linux", "linux_rpi"]


def signal_handler(sig, frame):
    """Handle SIGINT/SIGTERM for graceful shutdown."""
    global shutdown_requested
    if shutdown_requested:
        # Second signal - force exit
        safe_print("\nForced exit requested")
        sys.exit(1)
    shutdown_requested = True
    safe_print("\nShutdown requested, waiting for current uploads to finish...")


def safe_print(msg):
    """Thread-safe printing."""
    with print_lock:
        print(msg, flush=True)


def upload_file(file_path, base_path, server_path, api_token, properties,
                no_rel_path, base_local_path, log_file):
    """
    Upload a single file to Artifactory.
    Returns (file_path, success, message).
    """
    # Determine the artifact path
    if no_rel_path:
        file_name = os.path.basename(file_path)
        art_path = f"{base_path}/{server_path}/{file_name}"
    else:
        rel_path = os.path.relpath(file_path, base_local_path)
        art_path = f"{base_path}/{server_path}/{rel_path}"

    safe_print(f"\n=== Uploading {file_path} to {art_path}...")

    # Build curl command
    # Format: curl -H "X-JFrog-Art-Api:<token>" -X PUT "<url>;<props>" -T "<file>"
    url_with_props = f"{art_path};{properties}" if properties else art_path

    cmd = [
        "curl",
        "-H", f"X-JFrog-Art-Api:{api_token}",
        "-X", "PUT",
        url_with_props,
        "-T", file_path,
        "-s",  # Silent mode (no progress bar)
        "-S",  # Show errors
        "-f",  # Fail silently on HTTP errors (returns non-zero exit code)
        "-w", "%{http_code}"  # Write HTTP status code to stdout
    ]

    max_retries = 3
    for attempt in range(1, max_retries + 1):
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=300  # 5 minute timeout per file
            )

            # Check exit code and HTTP status
            http_code = result.stdout.strip()[-3:] if result.stdout else ""

            if result.returncode == 0:
                safe_print(f"=== Successfully uploaded {os.path.basename(file_path)} (HTTP {http_code})")
                return (file_path, True, f"Success (HTTP {http_code})")
            else:
                error_msg = result.stderr or f"HTTP {http_code}" or "Unknown error"
                if attempt < max_retries:
                    safe_print(f"=== Retry {attempt}/{max_retries} for {os.path.basename(file_path)}: {error_msg}")
                else:
                    safe_print(f"=== Failed to upload {os.path.basename(file_path)}: {error_msg}")
                    return (file_path, False, error_msg)

        except subprocess.TimeoutExpired:
            if attempt < max_retries:
                safe_print(f"=== Timeout, retry {attempt}/{max_retries} for {os.path.basename(file_path)}")
            else:
                safe_print(f"=== Timeout uploading {os.path.basename(file_path)}")
                return (file_path, False, "Timeout")
        except Exception as e:
            safe_print(f"=== Exception uploading {os.path.basename(file_path)}: {e}")
            return (file_path, False, str(e))

    return (file_path, False, "Max retries exceeded")


def set_folder_properties(base_path, server_path, local_path, api_token,
                          properties, no_rel_path, props_level):
    """Set properties on parent folders up to props_level."""
    if not properties or props_level <= 0:
        return

    if no_rel_path:
        art_path = f"{base_path}/{server_path}"
    else:
        # Get the directory part of the first file
        art_path = f"{base_path}/{server_path}/{os.path.dirname(local_path)}"

    for i in range(props_level):
        url_with_props = f"{art_path}/;{properties}"
        cmd = [
            "curl",
            "-H", f"X-JFrog-Art-Api:{api_token}",
            "-X", "PUT",
            url_with_props,
            "-s", "-S"
        ]
        safe_print(f"Setting properties on folder: {art_path}")
        try:
            subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        except Exception as e:
            safe_print(f"Warning: Failed to set properties on {art_path}: {e}")

        art_path = os.path.dirname(art_path)


def collect_files(local_path):
    """Collect all files from a path (file or directory)."""
    files = []
    abs_path = os.path.abspath(local_path)

    if os.path.isdir(abs_path):
        print("Detected directory, collecting files...")
        for dpath, _dnames, fnames in os.walk(abs_path):
            for fname in fnames:
                file_path = os.path.join(dpath, fname)
                files.append(file_path)
                print(f"Found: {file_path}")
    elif os.path.isfile(abs_path):
        files.append(abs_path)
        print(f"Detected that {abs_path} is a file.")
    else:
        print(f"Error: Path does not exist: {abs_path}")
        sys.exit(1)

    return files, abs_path


def main():
    parser = argparse.ArgumentParser(
        description='Upload files/folders to Artifactory with parallel uploads.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''Examples:
  ./upload_to_artifactory_parallel.py --server_path="linux/main/2020_11_25/arm/" \\
      --local_path="./artifacts" --properties="git_sha=abc123" --max_workers 10
'''
    )
    parser.add_argument("--base_path",
                        help="Artifactory Base Path - Internal ADI Artifactory server and development folder")
    parser.add_argument("--server_path", required=True,
                        help="Artifactory folder where the files/folders will be saved, for example 'linux' or 'linux/main/...'")
    parser.add_argument("--local_path", required=True,
                        help="Local path to file/folder to upload. It can be relative or absolute.")
    parser.add_argument("--properties",
                        help="Properties to be added to file/folder. If multiple ones, split them by ';'.")
    parser.add_argument("--no_rel_path", action="store_true",
                        help="If set, the relative path until local file will NOT be appended to artifactory path")
    parser.add_argument("--props_level", type=int, default=0,
                        help="Set for how many levels of folders to set specified properties (default: 0)")
    parser.add_argument("--token",
                        help="Artifactory authentication token (or set API_TOKEN env var)")
    parser.add_argument("--log_file",
                        help="Local file where to save the logs (currently unused in parallel mode)")
    parser.add_argument("--max_workers", type=int, default=10,
                        help="Maximum parallel uploads (default: 10)")

    args = parser.parse_args()

    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Get API token
    api_token = args.token or os.environ.get('API_TOKEN')
    if not api_token:
        print("Error: --token or API_TOKEN environment variable required")
        sys.exit(1)

    # Get base path
    base_path = args.base_path or os.environ.get('UPLOAD_BASE_PATH')
    if not base_path:
        print("Error: --base_path or UPLOAD_BASE_PATH environment variable required")
        sys.exit(1)

    # Validate server path
    server_path = args.server_path.lstrip('/')
    server_folder = server_path.split("/", 1)[0]
    if server_folder not in SERVER_FOLDERS_LIST:
        print(f"Error: --server_path must start with an allowed folder: {SERVER_FOLDERS_LIST}")
        print("If you want to add new folders, please edit this script or contact the script owner.")
        sys.exit(1)

    # Collect files
    files, base_local_path = collect_files(args.local_path)

    if not files:
        print("No files found to upload")
        sys.exit(1)

    print(f"\nFound {len(files)} files to upload")
    print(f"Using {args.max_workers} parallel workers\n")

    # Upload files in parallel
    success_count = 0
    failed_files = []

    with ThreadPoolExecutor(max_workers=args.max_workers) as executor:
        futures = {}

        # Submit files, checking for shutdown before each submission
        for f in files:
            if shutdown_requested:
                continue
            future = executor.submit(
                upload_file,
                f,
                base_path,
                server_path,
                api_token,
                args.properties or "",
                args.no_rel_path,
                base_local_path,
                args.log_file
            )
            futures[future] = f

        # Process completed futures
        for future in as_completed(futures):
            if shutdown_requested:
                # Cancel remaining pending futures
                for pending_future in futures:
                    pending_future.cancel()
                break

            file_path, success, message = future.result()
            if success:
                success_count += 1
            else:
                failed_files.append((file_path, message))

    # Set properties on folders if requested
    if args.props_level > 0 and files and not shutdown_requested:
        first_file_rel = os.path.relpath(files[0], base_local_path)
        set_folder_properties(
            base_path, server_path, first_file_rel,
            api_token, args.properties or "", args.no_rel_path, args.props_level
        )

    # Summary
    print("\n" + "=" * 60)
    print(f"Upload complete: {success_count}/{len(files)} succeeded")

    if shutdown_requested:
        not_started = len(files) - success_count - len(failed_files)
        print(f"\nShutdown requested - {not_started} uploads were cancelled")
        sys.exit(130)  # Standard exit code for SIGINT termination

    if failed_files:
        print(f"\nFailed uploads ({len(failed_files)}):")
        for f, msg in failed_files:
            print(f"  - {os.path.basename(f)}: {msg}")
        sys.exit(1)
    else:
        print("All uploads successful!")
        sys.exit(0)


if __name__ == "__main__":
    main()
