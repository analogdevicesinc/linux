from os import path

# -- Project information -----------------------------------------------------

repository = 'linux'
project = 'Linux Drivers'
copyright = '2025, Analog Devices, Inc.'
author = 'Analog Devices, Inc.'
version = '0.3'

locale_dirs = ['locales/']  # path is relative to the source directory
language = 'en'

# -- General configuration ---------------------------------------------------

extensions = [
    'adi_doctools',
]

needs_extensions = {
    'adi_doctools': '0.3'
}

exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']
source_suffix = '.rst'

# -- Options for HTML output --------------------------------------------------

html_theme = 'harmonic'

html_theme_options = {}

html_favicon = path.join("sources", "icon.svg")
