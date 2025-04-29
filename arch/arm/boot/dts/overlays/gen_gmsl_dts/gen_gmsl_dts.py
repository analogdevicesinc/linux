#!/usr/bin/env python3

from argparse import ArgumentParser
import json
import sys
from os import path

from jinja2 import Environment, FileSystemLoader

configs = sys.argv[1:]

vars_type = dict[str, str | int]

def hex_remove_0x(value):
    assert value[:2] == '0x'
    return value[2:]

def str_quote(value):
    return f'"{value}"'

def raise_exception(value):
    raise ValueError(value)

def debug_print(value):
    print(value)

def add_filter(env, fn):
    env.filters[fn.__name__] = fn

def add_global(env, fn):
    env.globals[fn.__name__] = fn

def read_template(dir: str, name: str, vars: vars_type) -> str:
    template_name = f'{name}.dtsi.in'

    env = Environment(
        loader=FileSystemLoader(dir),
        keep_trailing_newline=True,
        trim_blocks=True,
        lstrip_blocks=True,
    )
    add_filter(env, hex_remove_0x)
    add_filter(env, str_quote)
    add_global(env, raise_exception)
    add_global(env, debug_print)
    template = env.get_template(template_name)

    return template.render(**vars)

def write_config(
        config_path: str,
        dts_path: str | None = None,
        dtbo = False,
):
    config_dir = path.dirname(config_path)

    config_name = path.basename(config_path)
    config_root, _ = path.splitext(config_name)

    if dts_path is None:
        ext = 'dtso' if dtbo else 'dtsi'
        dts_name = f'{config_root}.{ext}'
        dts_path = path.join(config_dir, dts_name)

    with open(config_path, 'r') as f:
        config = json.load(f)

    with open(dts_path, 'w') as f:
        data = ''
        if dtbo:
            data += '''
/dts-v1/;
/plugin/;

'''.lstrip()

        for i, des_cfg in enumerate(config):
            data += read_template(config_dir, des_cfg['name'], {
                'des_cfg': des_cfg,
                'des_idx': i,
            })
        f.write(data)

if __name__ == '__main__':
    parser = ArgumentParser(description='Generate GMSL DTS')
    parser.add_argument('-o', '--output', help='DTS output path')
    parser.add_argument('--dtbo', action='store_true', help='Output as overlay')
    parser.add_argument('config', help='JSON configuration file')

    args = parser.parse_args()

    write_config(args.config, args.output, args.dtbo)
