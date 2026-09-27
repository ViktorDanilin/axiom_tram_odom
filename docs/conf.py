project = "Резервная одометрия трамвая"
author = "Команда AXIOM"
language = "ru"

extensions = ["myst_parser"]
myst_enable_extensions = ["amsmath", "dollarmath"]
source_suffix = {".md": "markdown"}
root_doc = "index"

html_theme = "furo"
html_title = "AXIOM · Резервная одометрия"
templates_path = ["_templates"]
html_static_path = ["_static"]
html_css_files = ["custom.css"]
html_show_sphinx = False
html_show_copyright = False
html_theme_options = {
    "navigation_with_keys": True,
    "light_css_variables": {
        "color-brand-primary": "#116e70",
        "color-brand-content": "#116e70",
        "color-background-primary": "#f8faf9",
        "color-sidebar-background": "#f0f5f3",
    },
}

exclude_patterns = ["_build", ".DS_Store"]
