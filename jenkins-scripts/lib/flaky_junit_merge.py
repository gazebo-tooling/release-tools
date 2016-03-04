#!/usr/bin/env python
from copy import deepcopy
from lxml import etree
import sys

if len(sys.argv) != 3:
    print('need to specify two files to merge', file=sys.stderr)
    exit()

f = open(sys.argv[1], 'r')
xml1 = etree.fromstring(f.read())
f.close()
f = open(sys.argv[2], 'r')
xml2 = etree.fromstring(f.read())
f.close()

if xml1.tag != 'testsuites':
    print('root tag should be testsuites', file=sys.stderr)
    exit
for ts in xml1.getchildren():
    if ts.tag != 'testsuite':
        print('child tags should be testsuite', file=sys.stderr)
        continue
    ts2 = xml2.findall(".//testsuite[@name='%s']" % (ts.attrib['name']))[0]
    for tc in ts.getchildren():
        tc2 = xml2.findall(".//testsuite[@name='%s']/testcase[@name='%s']" % (ts.attrib['name'], tc.attrib['name']))[0]
        failures1 = 0
        failures2 = 0
        for f in tc.getchildren():
            if f.tag == 'failure':
                failures1 += 1
        for f in tc2.getchildren():
            if f.tag == 'failure':
                failures2 += 1
        if failures1 == 1 and failures2 == 0:
            # flaky test
            f1 = ''
            for f in tc.getchildren():
                if f.tag == 'failure':
                    f.tag = 'flakyFailure'
                    f1 = f
            ts.attrib['failures'] = str(int(ts.attrib['failures']) - 1)
            xml1.attrib['failures'] = str(int(xml1.attrib['failures']) - 1)
            tc2.append(deepcopy(f1))
        elif failures1 == 0 and failures2 == 1:
            # flaky test
            f2 = ''
            for f in tc2.getchildren():
                if f.tag == 'failure':
                    f.tag = 'flakyFailure'
                    f2 = f
            ts2.attrib['failures'] = str(int(ts2.attrib['failures']) - 1)
            xml2.attrib['failures'] = str(int(xml2.attrib['failures']) - 1)
            tc.append(deepcopy(f2))

print etree.tostring(xml1)
# print etree.tostring(xml2)
