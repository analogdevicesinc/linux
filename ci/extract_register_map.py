#!/bin/python3
"""
Trys to extract register map from datasheet PDF
"""
import camelot
import pdfplumber
import sys

def extract_register_map(pdf_path, output_md='register_map.md'):

    with open(output_md, 'w') as f:
        with pdfplumber.open(pdf_path) as pdf:
            for i, page in enumerate(pdf.pages, 1):
                text = page.extract_text() or ""
                if 'register summary' in text.lower() and i > 10:
                    print(f"Found register summary on page {i}")

                    tables = camelot.read_pdf(pdf_path, pages=str(i), flavor='lattice')
                    if tables:
                        f.write("## Register Summary\n\n")

                        for table in tables:
                            df = table.df
                            if df.shape[0] > 10:
                                print(f"  Extracted {df.shape[0]} registers")

                                if 'address' in str(df.iloc[0].values).lower():
                                    df.columns = df.iloc[0]
                                    df = df[1:]

                                f.write(df.to_markdown(index=False))
                                f.write("\n\n---\n\n")
                    break

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
