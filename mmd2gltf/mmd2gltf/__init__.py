# -*- coding: utf-8 -*-
"""mmd2gltf: convert MMD PMX/VMD files to glTF 2.0 (.glb)."""
from .convert import convert
from .pmx import parse_pmx
from .vmd import parse_vmd

__version__ = "1.0.0"
__all__ = ["convert", "parse_pmx", "parse_vmd"]
