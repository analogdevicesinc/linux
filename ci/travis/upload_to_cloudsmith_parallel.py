#!/usr/bin/env python3
# SPDX-License-Identifier: (GPL-1.0-only OR BSD-2-Clause)

########################################################################
#
# File name: upload_to_cloudsmith_parallel.py
# Author: Liviu Tomoiaga
#
# This script uploads files to ADI Cloudsmith repository using parallel
# uploads for improved performance. Based on the sequential version from
# wiki-scripts, with parallelism inspired by ghdl/jenkins2.0/migration.
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

def upload_file(file_path, version, repo, api_key, tags, no_rel_path, base_local_path):
    """
    Upload a single file to Cloudsmith.
    Returns (file_path, success, message).
    """
    if not no_rel_path:
        # Get relative path from base_local_path
        rel_dir = os.path.dirname(os.path.relpath(file_path, base_local_path))
        if rel_dir:
            file_version = f"{version}{rel_dir}/"
        else:
            file_version = version
    else:
        file_version = version

    safe_print(f"\n=== Uploading {file_path} to adi/{repo} (version: {file_version})...")

    cmd = ["cloudsmith", "push", "raw", "--republish"]

    if tags:
        cmd.extend(["--tags", tags])

    if api_key:
        cmd.extend(["--api-key", api_key])

    cmd.extend(["--version", file_version])
    cmd.append(f"adi/{repo}")
    cmd.append(file_path)

    max_retries = 3
    for attempt in range(1, max_retries + 1):
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=300  # 5 minute timeout per file
            )

            if result.returncode == 0:
                safe_print(f"=== Successfully uploaded {os.path.basename(file_path)}")
                return (file_path, True, "Success")
            else:
                error_msg = result.stderr or result.stdout or "Unknown error"
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


def collect_files(local_path):
    """Collect all files from a path (file or directory)."""
    files = []
    abs_path = os.path.abspath(local_path)

    if os.path.isdir(abs_path):
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
        description='Upload files/folders to Cloudsmith with parallel uploads.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''Examples:
  ./upload_to_cloudsmith_parallel.py --repo sdg-linux --version "linux/main/" \\
      --local_path "./artifacts" --tags "git_sha-abc123" --max_workers 10
'''
    )
    parser.add_argument("--no_rel_path", action="store_true",
                        help="Don't append relative path to version")
    parser.add_argument("--local_path", required=True,
                        help="Local path to file/folder to upload")
    parser.add_argument("--tags",
                        help="Tags to add to files (semicolon-separated)")
    parser.add_argument("--version", required=True,
                        help="Version/path prefix for uploaded files")
    parser.add_argument("--token",
                        help="Cloudsmith API key (or set CLOUDSMITH_API_KEY env var)")
    parser.add_argument("--repo", required=True,
                        help="Cloudsmith repository name")
    parser.add_argument("--max_workers", type=int, default=10,
                        help="Maximum parallel uploads (default: 10)")

    args = parser.parse_args()

    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Get API key
    api_key = args.token or os.environ.get('CLOUDSMITH_API_KEY')
    if not api_key:
        print("Error: --token or CLOUDSMITH_API_KEY environment variable required")
        sys.exit(1)

    # Normalize version path
    version = args.version
    if not version.endswith('/'):
        version += '/'

    # Collect files
    files, base_path = collect_files(args.local_path)

    if not files:
        print("No files found to upload")
        sys.exit(1)

    print(f"\nFound {len(files)} files to upload")
    print(f"Using {args.max_workers} parallel workers\n")

    # Upload files in parallel
    success_count = 0
    failed_files = []
    cancelled_count = 0

    with ThreadPoolExecutor(max_workers=args.max_workers) as executor:
        futures = {}

        # Submit files, checking for shutdown before each submission
        for f in files:
            if shutdown_requested:
                cancelled_count += 1
                continue
            future = executor.submit(
                upload_file,
                f,
                version,
                args.repo,
                api_key,
                args.tags,
                args.no_rel_path,
                base_path
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
