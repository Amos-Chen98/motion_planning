#!/usr/bin/env python3
"""Run a unittest module with catkin-compatible XML, without a ROS master."""

import argparse
import importlib.util
from pathlib import Path
import sys
import unittest

from rosunit.xmlrunner import XMLTestRunner


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("test_file", type=Path)
    parser.add_argument("result_file", type=Path)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location(args.test_file.stem, args.test_file)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    suite = unittest.defaultTestLoader.loadTestsFromModule(module)
    args.result_file.parent.mkdir(parents=True, exist_ok=True)
    with args.result_file.open("w", encoding="utf-8") as stream:
        result = XMLTestRunner(stream).run(suite)
    return 0 if result.wasSuccessful() and result.testsRun else 1


if __name__ == "__main__":
    sys.exit(main())
