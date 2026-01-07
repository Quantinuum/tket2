# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information


project = "tket-py"
copyright = "2025, Quantinuum compiler team"
author = "Quantinuum compiler team"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx.ext.autosummary",
    "sphinx.ext.viewcode",
    "sphinx_copybutton",
    "myst_nb",
    "sphinx_autodoc_typehints",
]

autosummary_ignore_module_all = False  # Respect __all__ if specified
autosummary_generate = True

templates_path = ["_templates"]


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "furo"


exclude_patterns = ["jupyter_execute/**"]

suppress_warnings = [
    "misc.highlighting_failure",
]


intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "pytket": ("https://docs.quantinuum.com/tket/api-docs/", None),
    "hugr": ("https://quantinuum.github.io/hugr/", None),
}

nb_execution_mode = "off"


exclude_patterns = [
    "**/jupyter_execute",
    "jupyter_execute/*",
    ".jupyter_cache",
    "*.venv",
    "README.md",
    "**/README.md",
    ".jupyter_cache",
    "examples/1-Getting-Started.ipynb",
    "examples/2-Rewriting-Circuits.ipynb",
]
