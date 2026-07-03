#!/usr/bin/env python
## @ ResignSlimBootloader.py
# Re-sign a built SlimBootloader.bin using a replacement SBL key set.
#
# Copyright (c) 2026, Intel Corporation. All rights reserved.<BR>
# SPDX-License-Identifier: BSD-2-Clause-Patent

import os
import sys
import ast
import shutil
import argparse
import tempfile
from ctypes import sizeof

from CommonUtility import (
    HASH_DIGEST_SIZE,
    HASH_VAL_STRING,
    get_file_data,
    gen_file_from_object,
    load_source,
    gen_pub_key,
    get_key_type,
)
from BuildUtility import (
    STITCH_OPS,
    HASH_USAGE,
    HashStoreData,
    HashStoreTable,
    gen_pub_key_hash_store,
    gen_hash_file,
)
from CfgDataTool import CCfgData
from GenContainer import CONTAINER, gen_container_bin


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SBL_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..'))


def init_env(key_dir):
    os.environ['SBL_SOURCE'] = SBL_ROOT
    os.environ.setdefault('WORKSPACE', SBL_ROOT)
    os.environ.setdefault('PLT_SOURCE', SBL_ROOT)
    os.environ['SBL_KEY_DIR'] = os.path.abspath(key_dir)
    if SCRIPT_DIR not in sys.path:
        sys.path.append(SCRIPT_DIR)
    if SBL_ROOT not in sys.path:
        sys.path.append(SBL_ROOT)


def load_board(board_config):
    module = load_source('resign_board_cfg', board_config)
    board = module.Board()
    return board


def get_layout_map(board):
    image_layout = board.GetImageLayout()
    return {name: file_list for name, file_list in image_layout}


def flatten_image(image_name, layout_map, base_offset=0):
    leaves = []
    current = base_offset
    for src, algo, value, mode, pos in layout_map[image_name]:
        entry = {
            'name': src,
            'algo': algo,
            'value': value,
            'mode': mode,
            'pos': pos,
            'offset': current,
            'parent': image_name,
        }
        if src in layout_map:
            leaves.extend(flatten_image(src, layout_map, current))
        else:
            leaves.append(entry)
        current += value
    return leaves


def get_image_span(image_name, layout_map):
    return sum(item[2] for item in layout_map[image_name])


def pad_region(data, entry):
    size = entry['value']
    mode = entry['mode']
    pos = entry['pos']
    data_len = len(data)
    if mode == STITCH_OPS.MODE_FILE_IGNOR:
        return None
    if mode == STITCH_OPS.MODE_FILE_NOP:
        if data_len != size:
            raise Exception("Component '%s' size 0x%X does not match expected 0x%X" % (entry['name'], data_len, size))
        return data
    if data_len > size:
        raise Exception("Component '%s' size 0x%X exceeds region size 0x%X" % (entry['name'], data_len, size))
    padding = b'\xff' * (size - data_len)
    if pos == STITCH_OPS.MODE_POS_HEAD:
        return padding + data
    return data + padding


def extract_leaf_regions(sbl_path, leaf_entries, work_dir):
    image = bytearray(get_file_data(sbl_path))
    extracted = {}
    for entry in leaf_entries:
        slot = image[entry['offset']:entry['offset'] + entry['value']]
        if entry['name'] in extracted:
            continue
        out_path = os.path.join(work_dir, entry['name'])
        gen_file_from_object(out_path, slot)
        extracted[entry['name']] = out_path
    return image, extracted


def write_leaf_region(image, entry, file_path):
    data = get_file_data(file_path)
    padded = pad_region(data, entry)
    image[entry['offset']:entry['offset'] + entry['value']] = padded


def read_container_layout(layout_path):
    lines = []
    for line in get_file_data(layout_path, 'r').splitlines():
        striped = line.strip()
        if not striped or striped.startswith('#'):
            continue
        lines.append(striped.rstrip(','))
    if not lines:
        raise Exception("Container layout '%s' is empty" % layout_path)
    return ast.literal_eval('[%s]' % ','.join(lines))


def build_board_container_key_map(board):
    key_map = {}
    if not getattr(board, 'GetContainerList', None):
        return key_map
    for layout in board.GetContainerList():
        out_name = layout[0][1] if layout[0][1] else layout[0][0] + '.bin'
        entry_map = {}
        for name, _file, _alg, _auth, key_file, _align, _size, _svn in layout:
            entry_map[name] = key_file
        key_map[out_name] = entry_map
    return key_map


def resign_container(container_path, out_path, key_dir, temp_dir, board_key_map=None):
    if os.path.abspath(container_path) == os.path.abspath(out_path):
        return
    data = get_file_data(container_path)
    if len(data) < 4:
        shutil.copyfile(container_path, out_path)
        return
    if data.count(b'\xff') == len(data):
        shutil.copyfile(container_path, out_path)
        return

    container = CONTAINER(data)
    extract_dir = os.path.join(temp_dir, os.path.splitext(os.path.basename(container_path))[0])
    os.makedirs(extract_dir, exist_ok=True)
    container.set_dir_path(extract_dir, '.', '.', SCRIPT_DIR)
    container.extract('', container_path)

    layout_file = os.path.join(extract_dir, container.header.signature.decode() + '.txt')
    layout = read_container_layout(layout_file)
    key_name_map = board_key_map or {}
    rebuilt = []
    for idx, item in enumerate(layout):
        name, file_name, comp_alg, auth_type, key_file, alignment, region_size, svn = item
        if idx == 0:
            file_name = os.path.basename(out_path)
        if name in key_name_map and key_name_map[name]:
            key_file = key_name_map[name]
        rebuilt.append((name, file_name, comp_alg, auth_type, key_file, alignment, region_size, svn))

    gen_container_bin([rebuilt], os.path.dirname(out_path), extract_dir, key_dir, SCRIPT_DIR)


def resign_cfgdata(cfg_path, out_path, board):
    cfg_bins = bytearray(get_file_data(cfg_path))
    cfg_hdr = CCfgData.CDATA_BLOB_HEADER.from_buffer(cfg_bins)
    if cfg_hdr.Signature != b'CFGD':
        raise Exception("Invalid CFGDATA blob '%s'" % cfg_path)
    unsigned = bytearray(cfg_bins[:cfg_hdr.TotalLength])
    unsigned_hdr = CCfgData.CDATA_BLOB_HEADER.from_buffer(unsigned)
    unsigned_hdr.Attribute &= ~CCfgData.CDATA_BLOB_HEADER.ATTR_SIGNED
    tmp_path = out_path + '.unsigned'
    gen_file_from_object(tmp_path, unsigned)
    try:
        from BuildUtility import cfg_data_tool
        cfg_data_tool(
            'sign',
            ['-k', board._CFGDATA_PRIVATE_KEY, '-a', HASH_VAL_STRING[board.SIGN_HASH_TYPE], '-s', board._SIGNING_SCHEME, '-svn', str(get_cfgdata_svn(board)), tmp_path],
            out_path,
        )
    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)


def generate_master_key_hash(work_dir, board):
    mst_pub_key_file = os.path.join(work_dir, 'MSTKEY.bin')
    gen_pub_key(board._MASTER_PRIVATE_KEY, mst_pub_key_file)
    hash_alg = HASH_VAL_STRING[board.SIGN_HASH_TYPE]
    mst_hash = gen_hash_file(mst_pub_key_file, hash_alg, '', True)
    return mst_hash


def patch_stage1a_master_hash(stage1a_path, master_hash):
    data = bytearray(get_file_data(stage1a_path))
    hs_offset = data.find(HashStoreTable.HASH_STORE_SIGNATURE)
    if hs_offset < 0:
        raise Exception("HashStoreTable not found in '%s'" % stage1a_path)

    table = HashStoreTable.from_buffer(data, hs_offset)
    offset = hs_offset + sizeof(HashStoreTable)
    end_off = hs_offset + table.UsedLength
    while offset < end_off:
        entry = HashStoreData.from_buffer(data, offset)
        digest_off = offset + sizeof(HashStoreData)
        digest_end = digest_off + entry.DigestLen
        if entry.Usage == HASH_USAGE['PUBKEY_MASTER']:
            if len(master_hash) != entry.DigestLen:
                raise Exception('Master key hash length mismatch: 0x%X vs 0x%X' % (len(master_hash), entry.DigestLen))
            data[digest_off:digest_end] = master_hash
            gen_file_from_object(stage1a_path, data)
            return
        offset = digest_end

    raise Exception("PUBKEY_MASTER entry not found in '%s'" % stage1a_path)


def verify_key_contract(board):
    expected_key_type = board._RSA_SIGN_TYPE
    keys_to_check = [
        board._MASTER_PRIVATE_KEY,
        board._CFGDATA_PRIVATE_KEY,
        board._CONTAINER_PRIVATE_KEY,
    ]

    seen = set()
    for key_name in keys_to_check:
        if not key_name or key_name in seen:
            continue
        seen.add(key_name)
        key_type = get_key_type(key_name)
        if key_type != expected_key_type:
            raise Exception("Key '%s' type '%s' does not match board signing type '%s'" % (key_name, key_type, expected_key_type))


def get_cfgdata_svn(board):
    return getattr(board, 'CFGDATA_SVN', 0)


def resign_sbl(args):
    init_env(args.key_dir)
    board = load_board(os.path.abspath(args.board_config))
    verify_key_contract(board)

    image_path = os.path.abspath(args.input_image)
    layout_map = get_layout_map(board)
    image_size = os.path.getsize(image_path)
    top_pad = image_size - get_image_span('SlimBootloader.bin', layout_map)
    if top_pad < 0:
        raise Exception('Input image is smaller than the board layout expects')
    leaf_entries = flatten_image('SlimBootloader.bin', layout_map, top_pad)
    component_map = {}
    for entry in leaf_entries:
        component_map.setdefault(entry['name'], []).append(entry)

    required = ['KEYHASH.bin', 'STAGE1A_A.fd']
    for item in required:
        if item not in component_map or not component_map[item]:
            raise Exception("Required component '%s' is not present in the image" % item)
    if board.REDUNDANT_SIZE > 0 and ('STAGE1A_B.fd' not in component_map or not component_map['STAGE1A_B.fd']):
        raise Exception("STAGE1A_B.fd is missing for redundant layout")

    work_dir = os.path.abspath(args.work_dir) if args.work_dir else tempfile.mkdtemp(prefix='sbl-resign-')
    cleanup = not args.work_dir
    os.makedirs(work_dir, exist_ok=True)

    try:
        image, extracted = extract_leaf_regions(image_path, leaf_entries, work_dir)

        hash_alg = HASH_VAL_STRING[board.SIGN_HASH_TYPE]
        key_hash_list = board.GetKeyHashList() if getattr(board, 'GetKeyHashList', None) else []
        gen_pub_key_hash_store(
            board._MASTER_PRIVATE_KEY,
            key_hash_list,
            hash_alg,
            board._SIGNING_SCHEME,
            getattr(board, 'KEYH_SVN', 0),
            os.environ['SBL_KEY_DIR'],
            os.path.join(work_dir, 'KEYHASH.bin'),
        )

        if 'CFGDATA.bin' in extracted and getattr(board, '_CFGDATA_PRIVATE_KEY', ''):
            resign_cfgdata(extracted['CFGDATA.bin'], os.path.join(work_dir, 'CFGDATA.bin'), board)

        board_container_map = build_board_container_key_map(board)
        for container_name, key_map in board_container_map.items():
            if container_name in extracted:
                resign_container(
                    extracted[container_name],
                    os.path.join(work_dir, container_name),
                    os.environ['SBL_KEY_DIR'],
                    os.path.join(work_dir, 'containers'),
                    key_map,
                )

        if 'EPAYLOAD.bin' in extracted and os.path.getsize(extracted['EPAYLOAD.bin']) > 0 and get_file_data(extracted['EPAYLOAD.bin']).count(b'\xff') != os.path.getsize(extracted['EPAYLOAD.bin']):
            resign_container(
                extracted['EPAYLOAD.bin'],
                os.path.join(work_dir, 'EPAYLOAD.bin'),
                os.environ['SBL_KEY_DIR'],
                os.path.join(work_dir, 'containers'),
            )

        master_hash = generate_master_key_hash(work_dir, board)
        stage1a_a_path = os.path.join(work_dir, 'STAGE1A_A.fd')
        if os.path.abspath(extracted['STAGE1A_A.fd']) != os.path.abspath(stage1a_a_path):
            shutil.copyfile(extracted['STAGE1A_A.fd'], stage1a_a_path)
        patch_stage1a_master_hash(stage1a_a_path, master_hash)

        if 'STAGE1A_B.fd' in extracted:
            stage1a_b_path = os.path.join(work_dir, 'STAGE1A_B.fd')
            if os.path.abspath(extracted['STAGE1A_B.fd']) != os.path.abspath(stage1a_b_path):
                shutil.copyfile(extracted['STAGE1A_B.fd'], stage1a_b_path)
            patch_stage1a_master_hash(stage1a_b_path, master_hash)

        replacements = {
            'KEYHASH.bin': os.path.join(work_dir, 'KEYHASH.bin'),
            'STAGE1A_A.fd': stage1a_a_path,
        }
        if 'CFGDATA.bin' in extracted and os.path.exists(os.path.join(work_dir, 'CFGDATA.bin')):
            replacements['CFGDATA.bin'] = os.path.join(work_dir, 'CFGDATA.bin')
        if 'STAGE1A_B.fd' in extracted:
            replacements['STAGE1A_B.fd'] = stage1a_b_path
        for container_name in board_container_map:
            out_path = os.path.join(work_dir, container_name)
            if os.path.exists(out_path):
                replacements[container_name] = out_path
        if os.path.exists(os.path.join(work_dir, 'EPAYLOAD.bin')):
            replacements['EPAYLOAD.bin'] = os.path.join(work_dir, 'EPAYLOAD.bin')

        for comp_name, file_path in replacements.items():
            for entry in component_map[comp_name]:
                write_leaf_region(image, entry, file_path)

        gen_file_from_object(os.path.abspath(args.output_image), image)
        print("Re-signed SlimBootloader image was created successfully at:\n  %s" % os.path.abspath(args.output_image))
        if not cleanup:
            print("Working files are available at:\n  %s" % work_dir)
    finally:
        if cleanup and os.path.isdir(work_dir):
            shutil.rmtree(work_dir)


def main():
    parser = argparse.ArgumentParser(description='Re-sign a built SlimBootloader.bin using a replacement SBL key directory.')
    parser.add_argument('-i', '--input-image', dest='input_image', required=True, help='Input SlimBootloader.bin path')
    parser.add_argument('-o', '--output-image', dest='output_image', required=True, help='Output SlimBootloader.bin path')
    parser.add_argument('-b', '--board-config', dest='board_config', required=True, help='BoardConfig*.py path used for the original build')
    parser.add_argument('-k', '--key-dir', dest='key_dir', required=True, help='Directory containing the replacement SBL keys')
    parser.add_argument('-w', '--work-dir', dest='work_dir', default='', help='Optional working directory to keep extracted intermediate files')
    args = parser.parse_args()

    resign_sbl(args)


if __name__ == '__main__':
    sys.exit(main())