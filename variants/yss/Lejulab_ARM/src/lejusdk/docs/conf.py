# Configuration file for the Sphinx documentation builder.

import os

project = 'LejuSDK'
copyright = '2025, Leju Robotics'
author = 'Leju Robotics'
version = os.environ.get('LEJUSDK_VERSION', '1.0.0')
release = version

extensions = [
    'breathe',
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
]

# Breathe configuration
breathe_projects = {
    "LejuSDK": "./output/xml"
}
breathe_default_project = "LejuSDK"

exclude_patterns = [
    '_build', 'Thumbs.db', '.DS_Store', 'output',
]

language = 'zh_CN'

html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    'navigation_depth': 4,
    'collapse_navigation': False,
    'titles_only': False,
}

# 隐藏 "查看页面源码" 链接，并禁止将 rst 源码拷贝到 _sources/
html_show_sourcelink = False
html_copy_source = False

# 抑制特定警告
suppress_warnings = [
    'misc.*',
]
