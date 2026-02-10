#!/bin/python3
"""
Trys to extract register map from datasheet PDF
"""
import camelot
import pdfplumber
import sys
import re


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


def extract_adi_bitfield_format(pdf_path, output_md):
    """Extract ADI-style bit field descriptions (bit descriptions for REGISTER)."""
    with pdfplumber.open(pdf_path) as pdf:
        register_details = []

        for i, page in enumerate(pdf.pages, 1):
            text = page.extract_text() or ""
            if 'bit descriptions for' in text.lower():
                lines = text.split('\n')
                for line in lines:
                    if 'bit descriptions for' in line.lower():
                        register_name = line.split('for')[-1].strip()
                        register_details.append((i, register_name))
                        break

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

    register_pages = find_register_details_pages(pdf_path)
    print(f"Detected {len(register_pages)} unique register definitions")

    extract_register_summary(pdf_path, output_md, register_pages)
    tmc_count = extract_tmc_register_details(pdf_path, output_md, register_pages)

    if tmc_count == 0:
        adi_count = extract_adi_bitfield_format(pdf_path, output_md)
        if adi_count == 0:
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
