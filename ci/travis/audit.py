#!/usr/bin/python3

import json
import re
import urllib.error
import urllib.request
import sys
import subprocess

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def print_bold(text):
    print(bcolors.BOLD + text + bcolors.ENDC)

def print_fail(text):
    print(bcolors.FAIL + text + bcolors.ENDC)

def print_green(text):
    print(bcolors.OKGREEN + text + bcolors.ENDC)

def print_warning(text):
    print(bcolors.WARNING + text + bcolors.ENDC)

if (len(sys.argv) < 2):
    print_fail("Required JSON configuration file")
    sys.exit(1)

with open(sys.argv[1]) as f:
    config = json.load(f)

def print_bold(text):
    print(bcolors.BOLD + text + bcolors.ENDC)

def print_fail(text):
    print(bcolors.FAIL + text + bcolors.ENDC)

def clear_duplicates_in_list(lst):
    return list(dict.fromkeys(lst))

def get_driver_files(name, ignore_list = []):
    driver_files = []
    proc = subprocess.Popen(["git", "grep", name], stdout=subprocess.PIPE)
    for line in proc.stdout.readlines():
        line_s = line.decode("utf-8")
        line_s = line_s[:line_s.find(":")]
        if (line_s.endswith(".c") and (not line_s in ignore_list)):
            driver_files.append(line_s)

    return clear_duplicates_in_list(driver_files)

def get_part_name(line):
    idx = line.find("\"")
    if (idx < 0):
        return None

    part_name = line[idx + 1:]
    part_name = part_name[:part_name.find("\"")]

    idx = part_name.find(",")
    if (idx < 0):
        return part_name

    return part_name[idx + 1:]

# Very dumb state-machine for parsing part names from a kernel C code file.
# It currently relies on some basic formatting style that are found in Linux drivers.
def get_part_names_for_table_type(tbl_type, fname):
    parts_list = []
    parse_parts = False
    with open(fname, "r") as f:
        for line in f:
            if (line.find("static const struct " + tbl_type) > -1):
                parse_parts = True
            if (not parse_parts):
                continue

            part = get_part_name(line)
            if (part):
                parts_list.append(part)

            if (line.find(";") > -1):
                parse_parts = False

    return parts_list

def check_url(url):

    if (url == "<not_applicable>"):
        return False

    headers = {"User-Agent": "ADI Python Linux Driver checker tool"}

    errored = False

    try:
        url_req = urllib.request.Request(url, headers = headers)
        req = urllib.request.urlopen(url_req)
        if (req.getcode() != 200):
            errored = True
    except Exception as e:
        errored = True

    return errored

def do_audit(part_id, config):

    parts_table_config = config["parts"]

    non_adi_parts = config["ignored_parts"]

    if (part_id in non_adi_parts):
        return False

    non_analog_com_parts = config["with-no-analog.com-page"]

    adi_part_id = config["linux_to_analog.com_map"].get(part_id, part_id)
    if (adi_part_id == "<ignore>"):
        return False

    if (not part_id in non_analog_com_parts):
        product_url = "https://www.analog.com/en/products/%s.html" % adi_part_id
    else:
        product_url = "<not_applicable>"

    errored = check_url(product_url)

    # Try to find a dash or underscore and try to strip that
    if (errored):
        errored1 = True
        idx = adi_part_id.find("-")
        if (idx > -1):
            adi_part_id1 = adi_part_id[:idx]
            product_url = "https://www.analog.com/en/products/%s.html" % adi_part_id1
            errored1 = check_url(product_url)
            if (not errored1):
                print_warning("Found possible alternative for '%s' ADI part page on %s" % (adi_part_id, product_url))
        idx = adi_part_id.find("_")
        if (idx > -1):
            adi_part_id1 = adi_part_id[:idx]
            product_url = "https://www.analog.com/en/products/%s.html" % adi_part_id1
            errored1 = check_url(product_url)
            if (not errored1):
                print_warning("Found possible alternative for '%s' ADI part page on %s" % (adi_part_id, product_url))
        if (errored1):
            print_fail("Could not find any product page for '%s'" % adi_part_id)
            parts_table_config[part_id] = {
                "product-url" : "<invalid>"
            }
        else:
            parts_table_config[part_id] = {
                "product-url" : product_url
            }
    else:
        parts_table_config[part_id] = {
            "product-url" : product_url,
        }

    #if (not errored):
    #    print_green("'%s' seems OK" % part_id)

    return errored

have_errors = False

parts_list = []

for cfg in config["search_parts_config"]:
    print_bold("Processing files for '%s'" % cfg["name"])
    file_names = get_driver_files(cfg["name"], config["ignored_files"])

    for fname in file_names:
        for tbl_type in ["of_device_id", "spi_device_id", "i2c_device_id"]:
            parts_list.extend(get_part_names_for_table_type(tbl_type, fname))

    parts_list = clear_duplicates_in_list(parts_list)

cnt = len(parts_list)
print_bold("Collected %d part IDs from Linux drivers" % cnt)
for part in parts_list:
    cnt = cnt - 1
    if ((cnt % 30) == 0 or cnt == 10):
        print("Remaining %d to process" % cnt)
    if (not do_audit(part, config)):
        have_errors = True

with open("output.json", "w") as write_file:
    json.dump(config, write_file, indent=4, sort_keys=True)

if (have_errors):
    sys.exit(1)
