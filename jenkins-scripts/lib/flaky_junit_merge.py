#!/usr/bin/env python
from lxml import etree
from copy import deepcopy
f = open('test_results_original/UNIT_Rand_TEST.xml', 'r')
xml1 = etree.fromstring(f.read())
f.close()
f = open('test_results/UNIT_Rand_TEST.xml', 'r')
xml2 = etree.fromstring(f.read())
f.close()

print xml1.attrib['name']
print xml1.attrib['failures']
if xml1.tag != 'testsuites':
    print('root tag should be testsuites')
    exit
for ts in xml1.getchildren():
    if ts.tag != 'testsuite':
        print('child tags should be testsuite')
        continue
    print "  %s" %ts.attrib['name']
    print "  %s" %ts.attrib['failures']
    ts2 = xml2.findall(".//testsuite[@name='%s']" % (ts.attrib['name']))[0]
    print "  %s" %ts2.attrib['name']
    print "  %s" %ts2.attrib['failures']
    for tc in ts.getchildren():
        print "    %s" % tc.attrib['name']
        print "    %i" % len(tc.getchildren())
        tc2 = xml2.findall(".//testsuite[@name='%s']/testcase[@name='%s']" % (ts.attrib['name'], tc.attrib['name']))[0]
        print "    %s" % tc2.attrib['name']
        print "    %i" % len(tc2.getchildren())
        failures1 = 0
        failures2 = 0
        for f in tc.getchildren():
            print "      %s" % f.tag
            if f.tag == 'failure':
                failures1 += 1
        for f in tc2.getchildren():
            print "      %s" % f.tag
            if f.tag == 'failure':
                failures2 += 1
        print "%s %i %i" % (tc.attrib['name'], failures1, failures2)
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
print etree.tostring(xml2)
