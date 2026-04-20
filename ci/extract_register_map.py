#!/bin/python3
"""
Trys to extract register map from datasheet PDF
"""
import camelot
import pdfplumber
import sys
import re


def detect_pdf_register_style(pdf_path):
    """Detection of register documentation style.

    Scans a sample of pages to count style-specific markers:
      - 'tmc':   BIT 31..24 rows + Field/Reset/Access labels
      - 'maxim': D15 D14 .. D0 bit-layout tables
      - 'adi':   "Bit Descriptions for REGISTER" table headers

    Returns one of 'tmc', 'maxim', or 'adi' (the default fallback).
    """
    tmc_hits = 0
    maxim_hits = 0
    adi_hits = 0

    with pdfplumber.open(pdf_path) as pdf:
        for page in pdf.pages:
            text = page.extract_text() or ""
            if re.search(
                r'\bBIT\b.*\b(31|30|29|28|27|26|25|24)\b', text
            ) and re.search(r'\b(Field|Reset|Access)', text):
                tmc_hits += 1
            if re.search(r'D15\s+D14', text):
                maxim_hits += 1
            if re.search(r'bit descriptions for', text, re.IGNORECASE):
                adi_hits += 1

            # Early exit: 3 hits is enough to be confident
            if tmc_hits >= 3 or maxim_hits >= 3 or adi_hits >= 3:
                break

    if tmc_hits >= maxim_hits and tmc_hits >= adi_hits and tmc_hits > 0:
        return 'tmc'
    if maxim_hits >= adi_hits and maxim_hits > 0:
        return 'maxim'
    return 'adi'


def clean_watermark(text):
    """Remove common watermark/header text from extracted content."""
    watermarks = [
        r'A\s*N\s*A\s*L\s*O\s*G\s+D\s*E\s*V\s*I\s*C\s*E\s*S\s+I\s*N\s*C',
        r'CONFIDENTIAL',
        r'FOR ADI INTERNAL USE ONLY',
        r'AND NOT FOR FURTHER DISTRIBUTION',
        r'PRELIMINARY',
        r'www\.analog\.com.*',
        r'Analog Devices \| \d+',
    ]
    for pattern in watermarks:
        text = re.sub(pattern, '', text, flags=re.IGNORECASE)
    return text.strip()


def find_register_details_pages(pdf_path):
    """Find pages containing register definitions in TMC format (REGISTER_NAME (0xADDR))."""
    register_pages = {}

    with pdfplumber.open(pdf_path) as pdf:
        for i, page in enumerate(pdf.pages, 1):
            text = page.extract_text() or ""

            # Only consider pages that have register detail table markers
            # These pages have BIT row followed by Field/Reset/Access rows
            is_register_detail_page = bool(
                re.search(r'\bBIT\b.*\b(31|30|29|28|27|26|25|24)\b', text) and
                re.search(r'\b(Field|Reset|Access)', text)
            )

            if not is_register_detail_page:
                continue

            # Match TMC-style register headers: REGISTER_NAME (0xADDR) on its own line
            matches = re.findall(r'^([A-Z][A-Z0-9_]+)\s+\(0x([0-9A-Fa-f]+)\)\s*$', text, re.MULTILINE)
            if not matches:
                # Try pattern with optional description after
                matches = re.findall(r'^([A-Z][A-Z0-9_]+)\s+\(0x([0-9A-Fa-f]+)\)\s*\n', text, re.MULTILINE)

            for name, addr in matches:
                addr_int = int(addr, 16)
                # Only update if this is an earlier page (first occurrence of register details)
                if name not in register_pages or register_pages[name][0] > i:
                    register_pages[name] = (i, addr_int, addr)

    return register_pages


def find_maxim_register_details_pages(pdf_path):
    """Find pages containing register definitions in Maxim/ADI format.

    Detects register headers like 'RegisterName Register (XXXh)' or
    'RegisterName (XXXh)' paired with D15..D0 bit layout tables.
    Returns a dict mapping register name to (page_num, addr_int, addr_hex).
    """
    register_pages = {}

    with pdfplumber.open(pdf_path) as pdf:
        for i, page in enumerate(pdf.pages, 1):
            text = page.extract_text() or ""

            # Only consider pages that have D15..D0 bit layout markers
            if not re.search(r'D15\s+D14', text):
                continue

            # Match Maxim-style register headers:
            #   RegisterName Register (XXXh)
            #   RegisterName (XXXh)
            #   Table N. RegisterName Register (XXXh) Format
            # Address is hex digits followed by 'h', e.g. 0DAh, 1C2h, 000h
            # Register names can start with lowercase 'n' (nonvolatile) or 's' (SBS)
            header_pattern = r'(?:^|Table\s+\d+\.\s*)([ns]?[A-Z][A-Za-z0-9_]*)\s+(?:Register\s*)?\((?:0x)?([0-9A-Fa-f]+)h\)'
            matches = re.findall(header_pattern, text, re.MULTILINE)

            for name, addr in matches:
                addr_int = int(addr, 16)
                if name not in register_pages or register_pages[name][0] > i:
                    register_pages[name] = (i, addr_int, addr)

    return register_pages


def extract_register_summary(pdf_path, output_md, register_pages):
    """Extract register summary table from the PDF."""
    with pdfplumber.open(pdf_path) as pdf:
        summary_pages = []
        for i, page in enumerate(pdf.pages, 1):
            text = page.extract_text() or ""
            lower_text = text.lower()

            if i <= 10:
                continue

            score = 0
            reason = ""

            if re.search(r'table\s+\d+\.\s*register\s*(details|summary)', lower_text):
                score = 120
                reason = "Table N. Register Details/Summary"

            if 'register details' in lower_text and ('addr' in lower_text or 'bit' in lower_text):
                score = max(score, 110)
                reason = "register details"

            if 'registry summary' in lower_text:
                score = max(score, 100)
                reason = "registry summary"

            if 'address' in lower_text and 'name' in lower_text and 'type' in lower_text:
                score = max(score, 90)
                reason = "address+name+type columns"

            if 'register summary' in lower_text and ('addr' in lower_text or '0x' in text):
                score = max(score, 80)
                reason = "register summary"

            if 'register map' in lower_text and '0x' in text and 'see register' not in lower_text:
                score = max(score, 50)
                reason = "register map"

            if score == 0 and '0x' in text and 'R/W' in text and 'addr' in lower_text:
                score = 40
                reason = "register continuation"

            if score > 0:
                summary_pages.append((i, score, reason))

    if not summary_pages:
        print("No register summary pages found")
        return 0

    summary_pages.sort(key=lambda x: (-x[1], x[0]))
    start_page, start_score, start_reason = summary_pages[0]
    print(f"Found {start_reason} on page {start_page} (score: {start_score})")

    summary_pages.sort(key=lambda x: x[0])

    all_pages = [start_page]
    for page, score, reason in summary_pages:
        if page <= start_page:
            continue
        if page <= all_pages[-1] + 2 and score >= 40:
            all_pages.append(page)
        elif score >= 80:
            all_pages.append(page)

    if len(all_pages) > 1:
        print(f"  Extracting from {len(all_pages)} pages: {all_pages[0]}-{all_pages[-1]}")

    page_range = ','.join(str(p) for p in all_pages)

    try:
        lattice_tables = camelot.read_pdf(pdf_path, pages=page_range, flavor='lattice')
    except Exception:
        lattice_tables = []

    try:
        stream_tables = camelot.read_pdf(pdf_path, pages=page_range, flavor='stream')
    except Exception:
        stream_tables = []

    lattice_rows = sum(t.df.shape[0] for t in lattice_tables if t.df.shape[0] >= 5)
    stream_rows = sum(t.df.shape[0] for t in stream_tables if t.df.shape[0] >= 5)

    if stream_rows > lattice_rows:
        tables = stream_tables
        mode = "stream"
    else:
        tables = lattice_tables
        mode = "lattice"

    total_rows = 0
    if tables:
        with open(output_md, 'a') as f:
            f.write("## Register Summary\n\n")

            for table in tables:
                df = table.df
                if df.shape[0] >= 5:
                    header_row = -1
                    for idx in range(min(5, df.shape[0])):
                        row_text = str(df.iloc[idx].values).lower()
                        if ('addr' in row_text and 'name' in row_text) or \
                           ('address' in row_text and 'name' in row_text) or \
                           ('register' in row_text and 'bit' in row_text):
                            header_row = idx
                            break

                    if header_row >= 0:
                        df = df[header_row:]
                        df.columns = df.iloc[0]
                        df = df[1:]

                    total_rows += df.shape[0]
                    f.write(df.to_markdown(index=False))
                    f.write("\n\n---\n\n")

    if total_rows > 0:
        print(f"  Extracted {total_rows} register entries ({mode})")

    return total_rows


def extract_tmc_register_details(pdf_path, output_md, register_pages):
    """Extract TMC-style register details with bit fields."""
    if not register_pages:
        return 0

    sorted_by_page = sorted(register_pages.items(), key=lambda x: x[1][0])

    page_ranges = {}
    for i, (reg_name, (page_num, addr_int, addr_hex)) in enumerate(sorted_by_page):
        if i + 1 < len(sorted_by_page):
            next_page = sorted_by_page[i + 1][1][0]
            end_page = min(next_page - 1, page_num + 3)  # Up to next register or 3 pages
        else:
            end_page = page_num + 3  # Last register: check up to 3 more pages
        page_ranges[reg_name] = (page_num, end_page, addr_int, addr_hex)

    sorted_regs = sorted(page_ranges.items(), key=lambda x: x[1][2])

    with open(output_md, 'a') as f:
        f.write("## Register Bit Field Details\n\n")

    print(f"Found {len(sorted_regs)} registers with bit field details")

    extracted_count = 0
    for reg_name, (start_page, end_page, addr_int, addr_hex) in sorted_regs:
        page_range = ','.join(str(p) for p in range(start_page, end_page + 1))

        try:
            tables = camelot.read_pdf(pdf_path, pages=page_range, flavor='lattice')
        except Exception:
            tables = []

        if not tables:
            continue

        bit_layout_tables = []
        bitfield_desc_tables = []

        for table in tables:
            df = table.df
            if df.shape[0] < 2:
                continue

            # Check first column for table type
            first_col = [str(x).replace('\n', ' ').strip().upper() for x in df.iloc[:, 0].tolist()]

            # Bit layout table has BIT, Field, Reset, Access Type rows
            if 'BIT' in first_col and 'FIELD' in first_col:
                bit_layout_tables.append(df)
            # Bitfield description table has BITFIELD column header
            elif 'BITFIELD' in first_col or (df.shape[0] > 0 and 'BITFIELD' in str(df.iloc[0, 0]).upper()):
                bitfield_desc_tables.append(df)

        if not bit_layout_tables and not bitfield_desc_tables:
            continue

        with open(output_md, 'a') as f:
            f.write(f"### {reg_name} (0x{addr_hex})\n\n")

            # Write bit layout tables
            if bit_layout_tables:
                f.write("#### Bit Layout\n\n")
                for df in bit_layout_tables:
                    # Clean up the dataframe
                    df = df.map(lambda x: clean_watermark(str(x).replace('\n', ' ')))
                    f.write(df.to_markdown(index=False, headers='firstrow'))
                    f.write("\n\n")

            # Write bitfield descriptions
            if bitfield_desc_tables:
                f.write("#### Field Descriptions\n\n")
                for df in bitfield_desc_tables:
                    # Clean up the dataframe
                    df = df.map(lambda x: clean_watermark(str(x).replace('\n', ' ')))

                    # Try to identify and set proper headers
                    first_row = [str(x).strip().upper() for x in df.iloc[0].tolist()]
                    if 'BITFIELD' in first_row or 'BITS' in first_row:
                        df.columns = df.iloc[0]
                        df = df[1:]

                    f.write(df.to_markdown(index=False))
                    f.write("\n\n")

            f.write("---\n\n")

        extracted_count += 1

    print(f"  Extracted details for {extracted_count} registers")
    return extracted_count


def _parse_maxim_bit_table(table_rows, continuation_row=None):
    """Parse a Maxim/ADI D15..D0 bit layout table into a list of (field_name, bit_high, bit_low) tuples.

    Handles:
    - 16-column tables (D15..D0 in one row)
    - 8-column split tables (D15..D8 header, fields, separator, D7..D0 header, fields)
    - Multi-bit fields indicated by None in subsequent columns

    Args:
        table_rows: list of rows from pdfplumber extract_tables()
        continuation_row: optional value row for D7..D0 when the table is
                          split across pages (the D7 header exists but the
                          value row ended up on the next page)
    """
    fields = []

    def _extract_fields_from_header_and_values(header_row, value_row):
        """Extract fields from a header row (D15..D0) and corresponding value row."""
        result = []
        # Map header cells to bit numbers
        bit_numbers = []
        for cell in header_row:
            cell_str = str(cell or '').strip()
            m = re.match(r'^D(\d+)$', cell_str)
            if m:
                bit_numbers.append(int(m.group(1)))
            else:
                bit_numbers.append(None)

        i = 0
        while i < len(value_row):
            cell = value_row[i]
            cell_str = str(cell or '').strip()
            if not cell_str or i >= len(bit_numbers) or bit_numbers[i] is None:
                i += 1
                continue

            bit_high = bit_numbers[i]
            bit_low = bit_high

            # Check how many subsequent columns are None (multi-bit field)
            j = i + 1
            while j < len(value_row) and value_row[j] is None and j < len(bit_numbers):
                if bit_numbers[j] is not None:
                    bit_low = bit_numbers[j]
                j += 1

            result.append((cell_str, bit_high, bit_low))
            i = j

        return result

    # Detect format: check if first row has 16 columns (D15..D0) or 8 columns (D15..D8)
    if not table_rows:
        return fields

    header_row = table_rows[0]
    ncols = len(header_row)

    if ncols >= 16:
        # Single-row format: D15..D0
        if len(table_rows) >= 2:
            fields = _extract_fields_from_header_and_values(header_row, table_rows[1])
    elif ncols == 8:
        # Split format: D15..D8 header, values, [separator], D7..D0 header, values
        # Find value row for upper byte
        if len(table_rows) >= 2:
            fields = _extract_fields_from_header_and_values(header_row, table_rows[1])

        # Find the D7 header row
        for ri in range(2, len(table_rows)):
            row = table_rows[ri]
            row_strs = [str(c or '').strip() for c in row]
            if 'D7' in row_strs:
                if ri + 1 < len(table_rows):
                    # Check if the next row has actual values
                    next_vals = [str(c or '').strip() for c in table_rows[ri + 1]]
                    if any(v and v not in ('', 'None') for v in next_vals):
                        fields += _extract_fields_from_header_and_values(row, table_rows[ri + 1])
                    elif continuation_row is not None:
                        # Value row split to next page
                        fields += _extract_fields_from_header_and_values(row, continuation_row)
                elif continuation_row is not None:
                    # D7 header is last row, value row on next page
                    fields += _extract_fields_from_header_and_values(row, continuation_row)
                break

    return fields


def _extract_field_descriptions(lines, field_names):
    """Extract field descriptions from text lines following a register table.

    Looks for patterns like:
      FieldName: Description text
      FieldName—Description text

    Args:
        lines: list of text lines to search through
        field_names: list of field names to look for
    """
    descriptions = {}

    # Build a set of field names to look for (skip X/x/0/1/Reserved)
    skip = {'x', 'X', '0', '1', 'Reserved', ''}
    target_fields = {f for f in field_names if f not in skip}

    for field_name in target_fields:
        # Escape for regex
        escaped = re.escape(field_name)
        # Look for "FieldName: description" or "FieldName—description"
        pattern = rf'^{escaped}\s*[:—]\s*(.+)'
        for li, line in enumerate(lines):
            m = re.match(pattern, line.strip())
            if m:
                desc = m.group(1).strip()
                # Collect continuation lines
                for nli in range(li + 1, len(lines)):
                    next_line = lines[nli].strip()
                    # Stop at next field definition, table header, register header, or blank
                    if not next_line:
                        break
                    if re.match(r'^[A-Za-z]\w*\s*[:—]', next_line):
                        break
                    if re.match(r'^(Table\s+\d+|D15\s+D14|www\.)', next_line):
                        break
                    if re.match(r'^\w+\s+Register\s*\(', next_line):
                        break
                    desc += ' ' + next_line
                descriptions[field_name] = desc
                break

    return descriptions


def _clean_page_text_lines(text):
    """Return text lines with watermark/header/footer lines removed."""
    lines = text.split('\n')
    cleaned = []
    for line in lines:
        stripped = line.strip()
        # Skip common header/footer lines
        if re.match(r'^MAX17\d+\s', stripped):
            continue
        if re.match(r'^Fuel Gauge', stripped):
            continue
        if re.match(r'^www\.analog\.com', stripped):
            continue
        if re.match(r'^Analog Devices\s*\|', stripped):
            continue
        cleaned.append(line)
    return cleaned


def extract_maxim_register_details(pdf_path, output_md, register_pages):
    """Extract Maxim/ADI-style register details with D15..D0 bit layout tables.

    Uses pdfplumber to extract tables and text.  Field descriptions are
    collected across page boundaries so that continuation paragraphs on
    the next page are not lost.
    """
    if not register_pages:
        return 0

    with open(output_md, 'a') as f:
        f.write("## Register Bit Field Details\n\n")

    print(f"Found {len(register_pages)} Maxim/ADI-style registers with bit field details")

    # ------------------------------------------------------------------
    # Pass 1 – build a flat list of "events" across all pages.
    #
    # Each event is one of:
    #   ('reg_header', page, line_global, reg_name, addr_hex)
    #   ('bit_table',  page, line_global, table_rows)
    #   ('text',       page, line_global, line_text)
    #
    # line_global is a monotonically increasing counter so we can
    # compare positions across pages.
    # ------------------------------------------------------------------
    events = []          # list of event tuples
    all_lines = []       # flat list of (global_idx, line_text)
    global_idx = 0

    # Collect all pages that contain registers we care about, plus one
    # page after for description continuation.
    pages_of_interest = set()
    for _, (pg, _, _) in register_pages.items():
        pages_of_interest.add(pg)
        pages_of_interest.add(pg + 1)  # next page for continuations

    pending_continuation = None  # tracks incomplete split tables across pages

    with pdfplumber.open(pdf_path) as pdf:
        for page_num in sorted(pages_of_interest):
            if page_num < 1 or page_num > len(pdf.pages):
                continue

            page = pdf.pages[page_num - 1]
            text = page.extract_text() or ""
            tables = page.extract_tables()
            cleaned_lines = _clean_page_text_lines(text)

            # Identify D15 bit tables and orphan continuation rows
            bit_tables_on_page = []
            orphan_continuation_rows = []  # 1-row 8-col tables that may be D7 value continuations
            for table in tables:
                if not table or not table[0]:
                    continue
                header_strs = [str(c or '').strip() for c in table[0]]
                if 'D15' in header_strs:
                    bit_tables_on_page.append(table)
                elif (len(table) == 1 and len(table[0]) == 8
                      and 'D15' not in header_strs and 'D7' not in header_strs):
                    # Possible orphan value row from a cross-page split table
                    orphan_continuation_rows.append(table[0])

            # Check if the *last* D15 table on the previous page was
            # incomplete (has D7 header but no value row).  If so,
            # attach the first orphan row as its continuation.
            if orphan_continuation_rows and pending_continuation is not None:
                cont_table, cont_row = pending_continuation
                # Find the table in events and update it
                for ei in range(len(events) - 1, -1, -1):
                    if events[ei][0] == 'bit_table' and events[ei][3] is cont_table:
                        events[ei] = ('bit_table', events[ei][1], events[ei][2],
                                      cont_table, orphan_continuation_rows[0])
                        break
                pending_continuation = None

            # Detect incomplete split tables on this page for cross-page handling
            pending_continuation = None
            for table in bit_tables_on_page:
                if len(table[0]) == 8:
                    has_d7_header = False
                    has_d7_values = False
                    for ri, row in enumerate(table):
                        row_strs = [str(c or '').strip() for c in row]
                        if 'D7' in row_strs:
                            has_d7_header = True
                            if ri + 1 < len(table):
                                next_vals = [str(c or '').strip() for c in table[ri + 1]]
                                if any(v and v not in ('', 'None') for v in next_vals):
                                    has_d7_values = True
                    if has_d7_header and not has_d7_values:
                        pending_continuation = (table, None)

            bit_table_iter = iter(bit_tables_on_page)

            for line in cleaned_lines:
                stripped = line.strip()

                # Check for register header
                m = re.search(
                    r'(?:Table\s+\d+\.\s*)?([ns]?[A-Z][A-Za-z0-9_]*)\s+(?:Register\s*)?\((?:0x)?([0-9A-Fa-f]+)h\)',
                    stripped
                )
                if m:
                    events.append(('reg_header', page_num, global_idx, m.group(1), m.group(2)))

                # Check for D15 table start in text
                if re.match(r'^D15\s+D14', stripped):
                    tbl = next(bit_table_iter, None)
                    if tbl is not None:
                        # 5-tuple: type, page, gidx, table_rows, continuation_row
                        events.append(('bit_table', page_num, global_idx, tbl, None))

                all_lines.append((global_idx, stripped))
                events.append(('text', page_num, global_idx, stripped))
                global_idx += 1

    # ------------------------------------------------------------------
    # Pass 2 – walk events to pair each bit_table with its register
    # header and collect description text that follows (across pages).
    # ------------------------------------------------------------------
    register_records = []  # (reg_name, addr_hex, table_rows, desc_lines)

    last_reg_header = None   # (reg_name, addr_hex)
    i = 0
    while i < len(events):
        ev = events[i]
        if ev[0] == 'reg_header':
            last_reg_header = (ev[3], ev[4])  # name, addr
        elif ev[0] == 'bit_table' and last_reg_header is not None:
            table_rows = ev[3]
            continuation_row = ev[4] if len(ev) > 4 else None
            table_gidx = ev[2]

            # Collect description text lines after the table until
            # the next register header or next D15 table.
            desc_lines = []
            # Skip text events that are part of the table itself
            # (the D15 header line and the value line(s) right after)
            j = i + 1
            # Skip the value row(s) that immediately follow the D15 line
            skipping_table_rows = True
            while j < len(events):
                jev = events[j]
                if jev[0] == 'reg_header' or jev[0] == 'bit_table':
                    break
                if jev[0] == 'text':
                    line = jev[3]
                    # Skip the bit-value row (field names from the table)
                    if skipping_table_rows:
                        # The first non-empty text line after D15 is the
                        # value row; keep skipping until we see a line
                        # that looks like a description (has ':' or '—')
                        # or is clearly not a table row.
                        if re.match(r'^D[0-9]+\s', line):
                            j += 1
                            continue
                        # Value row: single line of field names without
                        # punctuation – skip it
                        if line and not re.search(r'[:—.]', line) and not re.match(r'^(Table\s+\d+|www\.)', line):
                            j += 1
                            skipping_table_rows = False
                            continue
                        skipping_table_rows = False
                    desc_lines.append(line)
                j += 1

            rname, raddr = last_reg_header
            register_records.append((rname, raddr, table_rows, desc_lines, continuation_row))
            # Don't reset last_reg_header – the same header might not
            # repeat before the next table on the same page.
        i += 1

    # ------------------------------------------------------------------
    # Pass 3 – deduplicate and write output.
    # ------------------------------------------------------------------
    seen = set()
    extracted_count = 0

    for rname, raddr, table_rows, desc_lines, continuation_row in register_records:
        reg_key = (rname, raddr)
        if reg_key in seen:
            continue
        seen.add(reg_key)

        fields = _parse_maxim_bit_table(table_rows, continuation_row=continuation_row)
        if not fields:
            continue

        field_names = [f[0] for f in fields]
        descriptions = _extract_field_descriptions(desc_lines, field_names)

        with open(output_md, 'a') as f:
            f.write(f"### {rname} (0x{raddr})\n\n")
            f.write("#### Bit Layout\n\n")

            f.write("| Bits | Field |\n")
            f.write("|------|-------|\n")
            for field_name, bit_high, bit_low in fields:
                if bit_high == bit_low:
                    bits_str = str(bit_high)
                else:
                    bits_str = f"{bit_high}:{bit_low}"
                f.write(f"| {bits_str} | {field_name} |\n")
            f.write("\n")

            if descriptions:
                f.write("#### Field Descriptions\n\n")
                for field_name, bit_high, bit_low in fields:
                    if field_name in descriptions:
                        if bit_high == bit_low:
                            bits_str = str(bit_high)
                        else:
                            bits_str = f"{bit_high}:{bit_low}"
                        f.write(f"- **{field_name}** (Bit {bits_str}): {descriptions[field_name]}\n")
                f.write("\n")

            f.write("---\n\n")

        extracted_count += 1

    print(f"  Extracted details for {extracted_count} registers")
    return extracted_count


def extract_adi_bitfield_format(pdf_path, output_md):
    """Extract ADI-style bit field descriptions (bit descriptions for REGISTER)."""
    with pdfplumber.open(pdf_path) as pdf:
        register_details = []  # (page_num_1indexed, register_name)

        for i, page in enumerate(pdf.pages, 1):
            text = page.extract_text() or ""
            if 'bit descriptions for' not in text.lower():
                continue
            lines = text.split('\n')
            for line in lines:
                if 'bit descriptions for' in line.lower():
                    raw = line.split('for')[-1].strip()
                    # Strip trailing "(Continued)" — the table continues
                    # from a previous page; skip it to avoid duplicates.
                    if re.search(r'\(Continued\)', raw, re.IGNORECASE):
                        continue
                    register_name = raw
                    register_details.append((i, register_name))

        if not register_details:
            return 0

        print(f"Found {len(register_details)} ADI-style bit field tables")

        extracted = 0
        with open(output_md, 'a') as f:
            for page_num, reg_name in register_details:
                print(f"  Page {page_num}: {reg_name}")

                try:
                    tables = camelot.read_pdf(pdf_path, pages=str(page_num),
                                              flavor='stream', edge_tol=50)
                except Exception:
                    continue

                for table in tables:
                    df = table.df
                    header_text = ' '.join(str(x) for x in df.iloc[0:2].values.flatten()).lower()
                    if 'bit' in header_text and ('description' in header_text or 'bit name' in header_text):
                        f.write(f"### {reg_name}\n\n")

                        if df.shape[0] > 1:
                            df.columns = df.iloc[1]
                            df = df[2:]

                        f.write(df.to_markdown(index=False))
                        f.write("\n\n")
                        extracted += 1
                        break

        return extracted


def extract_register_map(pdf_path, output_md='register_map.md'):
    """Main function to extract register map from PDF."""
    with open(output_md, 'w') as f:
        f.write(f"# Register Map\n\n")
        f.write(f"Extracted from: {pdf_path}\n\n")

    style = detect_pdf_register_style(pdf_path)
    print(f"Detected register style: {style}")

    bitfield_count = 0

    if style == 'tmc':
        register_pages = find_register_details_pages(pdf_path)
        print(f"  {len(register_pages)} unique TMC register definitions")
        extract_register_summary(pdf_path, output_md, register_pages)
        bitfield_count = extract_tmc_register_details(pdf_path, output_md, register_pages)

    elif style == 'maxim':
        maxim_register_pages = find_maxim_register_details_pages(pdf_path)
        print(f"  {len(maxim_register_pages)} unique Maxim register definitions")
        extract_register_summary(pdf_path, output_md, {})
        bitfield_count = extract_maxim_register_details(pdf_path, output_md, maxim_register_pages)

    # ADI is always the fallback
    if bitfield_count == 0:
        if style != 'adi':
            print(f"  {style} extraction found 0 registers, falling back to ADI style")
        extract_register_summary(pdf_path, output_md, {})
        bitfield_count = extract_adi_bitfield_format(pdf_path, output_md)

    if bitfield_count == 0:
        with open(output_md, 'a') as f:
            f.write("## Register Bit Field Details\n\n")
            f.write("No bit field tables found.\n")

    print(f"\nRegister map saved to: {output_md}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: ")
        print("  ./extract_register_map.py ./datasheet.pdf")
    else:
        pdf_path = sys.argv[1]
        output_path = sys.argv[2] if len(sys.argv) > 2 else "register_map.md"

        extract_register_map(pdf_path, output_path)
