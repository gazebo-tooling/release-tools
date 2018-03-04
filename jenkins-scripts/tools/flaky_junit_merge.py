#!/usr/bin/env python
# This script merges junit files to identify flaky tests.
# Usage: flaky_junit_merge.py file1.xml file2.xml
# A junit testcase is considered flaky if it has no failures
# in one file but some in the other file.
# For the failing file, the <failure> tag is changed to <flakyFailure>,
# and the failure count is reduced.
# For the passing file, the <flakyFailure> node is appended
# to the relevant testcase.
# The merged output is printed to stdout.
from __future__ import print_function
from copy import deepcopy
from lxml import etree
import os
import sys

# Count the number of child <failure> tags
def count_failures(testcase):
    failures = 0
    for f in testcase.getchildren():
        if f.tag == 'failure':
            failures += 1
    return failures

# Subtract one from the 'failures' attribute
# if it has the expected tag
# (this is so we don't decrement twice when testsuite is the root tag)
def one_less_failure(element, expectedTag):
    if element.tag == expectedTag:
        element.attrib['failures'] = str(int(element.attrib['failures']) - 1)

# Try to read file as xml
# Create simple xml document if file is empty
def open_xml(fileName):
    f = open(fileName, 'r')
    try:
      xml = etree.fromstring(f.read().encode())
    except etree.XMLSyntaxError as err:
        print("error parsing %s as xml, writing generic test_ran failure" % (fileName), file=sys.stderr)
        testName = os.path.basename(fileName)
        d = {'test': testName, 'fileName': fileName , 'testNoXml': testName.replace('.xml', '')}
        xml = etree.fromstring(("""<?xml version="1.0" encoding="UTF-8"?>
<testsuite tests="1" failures="1" time="1" errors="0" name="%(test)s">
  <testcase name="test_ran" status="run" time="1" classname="%(testNoXml)s">
    <failure message="Unable to find test results for %(test)s, test did not run.\nExpected results in %(fileName)s" type=""/>
  </testcase>
</testsuite>"""%d).encode())
    f.close()
    return xml

if len(sys.argv) != 3:
    print('need to specify two files to merge', file=sys.stderr)
    exit()
fileName1 = sys.argv[1]
fileName2 = sys.argv[2]

xml1 = open_xml(fileName1)
xml2 = open_xml(fileName2)

# if root tags differ, make sure xml1 has testsuites
if xml1.tag == 'testsuite' and xml2.tag == 'testsuites':
    xml1,xml2 = xml2,xml1
    fileName1,fileName2 = fileName2,fileName1

# if root tags differ
if xml1.tag == 'testsuites' and xml2.tag == 'testsuite':
    # and xml2 has one testcase with name 'test_ran'
    if 1 == len(xml2.getchildren()) == len(xml2.findall("./testcase[@name='test_ran']")):
        # add a passing `test_ran` testcase to xml1
        testRanSuite = deepcopy(xml2)
        testRanSuite.attrib['failures'] = "0"
        testRanSuite.attrib['time'] = xml1.attrib['time']
        tc = testRanSuite.getchildren()[0]
        # with a matching classname
        tc.attrib['classname'] = xml1.findall(".//testcase")[0].attrib['classname']
        # and child <failure> tags removed
        for c in tc.getchildren():
            tc.remove(c)
        xml1.append(testRanSuite)

# we want to iterate over the testsuite elements
if xml1.tag == 'testsuites':
    testsuiteArray = xml1.getchildren()
elif xml1.tag == 'testsuite':
    # just add the whole doc to a list
    testsuiteArray = [xml1]
else:
    print("root tag of %s should be testsuite or testsuites" % (fileName1),
          file=sys.stderr)
    exit()

# we search for testsuite elements in the second file with XPath,
# which means testsuite can't be the root element
# so create root2 with root tag testsuites to contain the testsuite if necessary
if xml2.tag == 'testsuites':
    root2 = xml2
elif xml2.tag == 'testsuite':
    root2 = etree.Element('testsuites')
    root2.append(xml2)
else:
    print("root tag of %s should be testsuite or testsuites" % (fileName2),
          file=sys.stderr)
    exit()

# iterate over <testsuite> tags in xml1
for ts in testsuiteArray:
    if ts.tag != 'testsuite':
        print('expected testsuite tag', file=sys.stderr)
        continue
    # find <testsuite> tag with matching name attribute in root2
    try:
        ts2 = root2.findall(".//testsuite[@name='%s']" % (ts.attrib['name']))[0]
    except IndexError as err:
        print("testsuite named %s not found in %s" % (ts.attrib['name'], fileName2),
              file=sys.stderr)
        continue
    # iterate over <testcase> tags in <testsuite> from xml1
    for tc in ts.getchildren():
        if tc.tag != 'testcase':
            print('expected testcase tag', file=sys.stderr)
            continue
        # find matching <testcase> tag from root2
        try:
            tc2 = root2.findall(".//testsuite[@name='%s']/testcase[@name='%s']" %
                                (ts.attrib['name'], tc.attrib['name']))[0]
        except IndexError as err:
            print("testsuite/testcase named %s/%s not found in %s" %
                  (ts.attrib['name'], tc.attrib['name'], fileName2),
                  file=sys.stderr)
            continue
        failures1 = count_failures(tc)
        failures2 = count_failures(tc2)
        if failures1 > 0 and failures2 == 0:
            # flaky test
            for f in tc.getchildren():
                if f.tag == 'failure':
                    f.tag = 'flakyFailure'
                    tc2.append(deepcopy(f))
            one_less_failure(ts, 'testsuite')
            one_less_failure(xml1, 'testsuites')
        elif failures1 == 0 and failures2 > 0:
            # flaky test
            for f in tc2.getchildren():
                if f.tag == 'failure':
                    f.tag = 'flakyFailure'
                    tc.append(deepcopy(f))
            one_less_failure(ts2, 'testsuite')
            one_less_failure(xml2, 'testsuites')
        elif failures1 > 0 and failures2 > 0:
            # repeated failures
            # append the second failure as a rerunFailure
            for f in tc2.getchildren():
                if f.tag == 'failure':
                    f.tag = 'rerunFailure'
                    tc.append(deepcopy(f))

# This script modifies the content of both files, and either could be printed.
# This script prints the first one, but the second one could also be printed.
print(etree.tostring(xml1).decode())
# print(etree.tostring(xml2))
