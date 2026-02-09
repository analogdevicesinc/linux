#!/bin/python3
"""
Trys to extract register map from datasheet PDF
"""
import camelot
import pdfplumber
import sys

def extract_register_map(pdf_path, output_md='register_map.md'):

    with open(output_md, 'w') as f:
        register_pages = []

        with pdfplumber.open(pdf_path) as pdf:
            for i, page in enumerate(pdf.pages, 1):
                text = page.extract_text() or ""
                lower_text = text.lower()

                if i <= 10:
                    continue

                score = 0
                reason = ""

                import re
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
                    register_pages.append((i, score, reason))

        if not register_pages:
            print("No register pages found")
        else:
            register_pages.sort(key=lambda x: (-x[1], x[0]))  # Sort by score desc, then page number
            start_page, start_score, start_reason = register_pages[0]
            print(f"Found {start_reason} on page {start_page} (score: {start_score})")

            register_pages.sort(key=lambda x: x[0])

            all_pages = [start_page]
            for page, score, reason in register_pages:
                if page <= start_page:
                    continue  # Skip pages before start
                if page <= all_pages[-1] + 2 and score >= 40:
                    all_pages.append(page)
                elif score >= 80:
                    all_pages.append(page)

            if len(all_pages) > 1:
                print(f"  Extracting from {len(all_pages)} pages: {all_pages[0]}-{all_pages[-1]}")

            page_range = ','.join(str(p) for p in all_pages)

            try:
                lattice_tables = camelot.read_pdf(pdf_path, pages=page_range, flavor='lattice')
            except:
                lattice_tables = []

            try:
                stream_tables = camelot.read_pdf(pdf_path, pages=page_range, flavor='stream')
            except:
                stream_tables = []

            lattice_rows = sum(t.df.shape[0] for t in lattice_tables if t.df.shape[0] >= 5)
            stream_rows = sum(t.df.shape[0] for t in stream_tables if t.df.shape[0] >= 5)

            if stream_rows > lattice_rows:
                tables = stream_tables
                mode = "stream"
            else:
                tables = lattice_tables
                mode = "lattice"

            extracted = False
            total_rows = 0
            if tables:
                for table in tables:
                    df = table.df
                    if df.shape[0] >= 5:
                        if not extracted:
                            f.write("## Register Summary\n\n")
                            extracted = True

                        header_row = -1
                        for i in range(min(5, df.shape[0])):
                            row_text = str(df.iloc[i].values).lower()
                            if ('addr' in row_text and 'name' in row_text) or \
                               ('address' in row_text and 'name' in row_text) or \
                               ('register' in row_text and 'bit' in row_text):
                                header_row = i
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

        f.write("## Register Bit Field Details\n\n")

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

            print(f"Found {len(register_details)} registers with bit field tables\n")

            for page_num, reg_name in register_details:
                print(f"  Page {page_num}: {reg_name}")

                tables = camelot.read_pdf(pdf_path, pages=str(page_num),
                                         flavor='stream', edge_tol=50)

                for table in tables:
                    df = table.df

                    header_text = ' '.join(str(x) for x in df.iloc[0:2].values.flatten()).lower()
                    if 'bit' in header_text and ('description' in header_text or 'bit name' in header_text):

                        f.write(f"### {reg_name}\n\n")

                        if df.shape[0] > 1:
                            df.columns = df.iloc[1]
                            df = df[2:]  # Skip title and header rows

                        f.write(df.to_markdown(index=False))
                        f.write("\n\n")
                        break

    print(f"Register map saved to: {output_md}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: ")
        print("  ./extract_register_map.py ./datasheet.pdf")
    else:
        pdf_path = sys.argv[1]
        output_path = sys.argv[2] if len(sys.argv) > 2 else "register_map.md"

        extract_register_map(pdf_path, output_path)
