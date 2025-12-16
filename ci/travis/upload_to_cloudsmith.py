# SPDX-License-Identifier: (GPL-1.0-only OR BSD-2-Clause)

########################################################################
#
# File name: upload_to_cloudsmith.py
# Author: Liviu Tomoiaga
#
# This script is used to upload files to ADI Cloudsmith repository
#
# Copyright (C) 2019-2024 Analog Devices Inc.
#
#######################################################################

#!/usr/bin/python3.6

import os
import argparse
import glob
import shutil
import sys
from glob import glob

LOCAL_PATHS_LIST = []
# If you try to upload files in a different folder than the ones in below list, there will be printed a message and files won't be uploaded
SERVER_FOLDERS_LIST = ["linux", "linux_rpi"]

## Comparison with upload_to_artifactory.py
# artifactory   | cloudsmith
# --------------|-----------
# --base_path   | "adi" - there are no other base paths
# --server_path | --version
# --base_path   | --repo
# --properties  | --tags applied to all files, split by ','
# --local_path  | --local_path
# --no_rel_path | --no_rel_path
# --props_level | N/A

########### Define arguments and help section #################

parser = argparse.ArgumentParser(
   description='This script is uploading files or folders to Cloudsmith. Parameter order doesn\'t matter.', \
   formatter_class=argparse.RawDescriptionHelpFormatter,
   epilog='Examples: '\
                + '\n-> "./upload_to_cloudsmith.py --repo <CLOUDSMITH_REPO> --version "hdl/main/2020_12_12/pluto" --local_path "../projects/pluto/log_file.txt"'\
                + ' --tags "git_sha_928ggraf,timestamp_2020_12_12" --no_rel_path" will upload file "log_file.txt" to '\
                + ' adi/<CLOUDSMITH_REPO> with the version hdl/main/2020_12_12/pluto and add tags git_sha=928ggraf and git_commit_date=2010_12_12 on it.'\
                + '\n-> "./upload_to_cloudsmith.py --repo <CLOUDSMITH_REPO> --version "linux" --local_path "main/2020_11_25/arm/zynq_zed_adv7511.dtb"" will upload dtb'\
                + ' file to adi/<CLOUDSMITH_REPO> with version linux/main/2020_11_25/arm')
parser.add_argument("--no_rel_path", help="If this exists, the relative path until local file will be appended to artifactory path", action="store_true")
parser.add_argument("--local_path",  help="Local path to file/folder to upload. It can be relative or absolute.")
parser.add_argument("--tags",        help="Tags to be added to file.. If multiple ones, split them by ';'.")
parser.add_argument("--version",     help="Version corresponding to the folder structure path where you expect to find the package, for example 'hdl' or 'linux'.")
parser.add_argument("--token",       help="Cloudsmith authentication token. Otherwise you can export CLOUDSMITH_API_KEY in terminal before calling this script.")
parser.add_argument("--repo",        help="Cloudsmith repository where we are uploading the files.")
parser.add_argument("--log_file",    help="Local file where to save the logs from curl command, if no file is specified the logs will be printed in terminal")
args = parser.parse_args()
parser.parse_args()

########## Check if required and optional arguments are set  #################
if args.token:
   CLOUDSMITH_API_KEY = args.token
else:
   if "CLOUDSMITH_API_KEY" in os.environ:
      CLOUDSMITH_API_KEY = os.environ['CLOUDSMITH_API_KEY']
   else:
      print('\nError:Parameter "--token" is not set. This is Artifactory Authentication Token and can be set even using parameter "--token" on upload command, even by exporting API_TOKEN variable in terminal, before calling upload script.')
      exit(1)

if args.repo:
   CLOUDSMITH_REPO = args.repo
else:
   if "CLOUDSMITH_REPO" in os.environ:
      CLOUDSMITH_REPO = os.environ['CLOUDSMITH_REPO']
   else:
      print('\nError: Parameter "--repo" is not set. This is Cloudsmith repository where you want to upload. It can be set even using parameter "--repo" on upload command, even by exporting CLOUDSMITH_REPO variable in terminal, before calling upload script.')
      exit(1)

if args.no_rel_path:
   NO_REL_PATH = True
else:
   NO_REL_PATH = False

if args.log_file:
   LOG_FILE = ">> " + args.log_file
else:
   LOG_FILE = ''

if args.tags:
   TAGS = args.tags
else:
   TAGS = ''

if args.version:
   VERSION = args.version
   if VERSION[-1] != '/':
      VERSION += "/"
elif not NO_REL_PATH:
   VERSION = ""
else:
   print('\nError: Parameter "--version" is required if you are using "--no_rel_path". It should be set to the path in a folder structure where you expect to find the files/folders. Check help section.')
   exit(1)

if args.local_path:
   LOCAL_PATH = os.path.abspath(args.local_path) if '../' in args.local_path else args.local_path
   # if there was given a dir as local_path parameter, get all the files inside it in a list
   if os.path.isdir(LOCAL_PATH):
      for dpath, dnames, fnames in os.walk(LOCAL_PATH):
         for i, FILE_NAME in enumerate([os.path.join(dpath, fname) for fname in fnames]):
            print(FILE_NAME)
            LOCAL_PATHS_LIST.append(str(FILE_NAME))
   elif os.path.isfile(LOCAL_PATH):
      LOCAL_PATHS_LIST = [LOCAL_PATH]
      print(f"\nDetected that {LOCAL_PATHS_LIST} is a file.")
   else:
      print('\nError:It looks that parameter "--local_path" is wrong defined/does not exists. Plese check: ' + LOCAL_PATH)
      exit(1)
else:
   print('\nParameter "--local_path" is required. It should point to local file/folder to upload.')
   exit(1)


########## Upload files ##########
# If files with same name already exists at specified server path, they will be overwritten

for FILE in LOCAL_PATHS_LIST:
   if not NO_REL_PATH:
      FILE_VERSION = VERSION + os.path.dirname(FILE)
   else:
      FILE_VERSION = VERSION
   print('\n\n === Start uploading ' + str(FILE) + ' on adi/' + str(CLOUDSMITH_REPO) + '...')
   upload_cmd = "cloudsmith push raw -W -S --republish "
   if TAGS:
      upload_cmd += f"--tags '{TAGS}' "
   if CLOUDSMITH_API_KEY:
      upload_cmd += f"--api-key '{CLOUDSMITH_API_KEY}' "
   upload_cmd += f"--version '{FILE_VERSION}' "
   upload_cmd += f"adi/{CLOUDSMITH_REPO} {FILE}"
   os.system(upload_cmd)

#################################################
