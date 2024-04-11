# Copyright (C) 2024 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import os
import sys
from xml.etree import ElementTree as ET

SYSTEM_OUT_TAG = "system-out"


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        description="Parse system output from junit file generated by CMake and inject them into junit files generated by gtest."
    )
    parser.add_argument(
        "cmake_junit_file",
        help="Junit file generated by 'cmake --output-junit=<cmake_junit_file>'",
    )
    parser.add_argument(
        "gtest_results_dir", help="Directory where gtest junit files were output"
    )
    args = parser.parse_args(argv)

    doc = ET.parse(args.cmake_junit_file)

    for testcase in doc.findall("testcase"):
        results_file_name = os.path.join(
            args.gtest_results_dir, testcase.attrib["name"] + ".xml"
        )

        # Ignore tests that start with 'check_'. Those are tests that simply run
        # the check_test_run.py script, so we don't need their output.
        if results_file_name.startswith("check_"):
            continue
        system_output = testcase.find(SYSTEM_OUT_TAG)
        assert system_output is not None
        try:
            results_doc = ET.parse(results_file_name)
            should_write_to_file = False
            for result_testsuite in results_doc.findall("testsuite"):
                if result_testsuite.find(SYSTEM_OUT_TAG) is None:
                    result_testsuite.append(system_output)
                    should_write_to_file = True

            # junit files created by gtest start with `<testsuites>` (plural).
            # If the root tag is `<testsuite>`, this indicates that the junit file was
            # created by our check_test_run.py script, which means the test crashed, in which
            # case, we will just write the output to the first testcase.
            if results_doc.getroot().tag == "testsuite":
                result_test_case = results_doc.find("testcase")
                assert result_test_case is not None
                if result_test_case.find(SYSTEM_OUT_TAG) is None:
                    result_test_case.append(system_output)
                    should_write_to_file = True

            if should_write_to_file:
                results_doc.write(results_file_name)

        except Exception as e:
            print(e)


if __name__ == "__main__":
    main()
