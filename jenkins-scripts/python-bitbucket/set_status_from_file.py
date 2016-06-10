#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, getopt

from pprint import pprint

from pybitbucket.auth import BasicAuthenticator
from pybitbucket.bitbucket import Client
from pybitbucket.build import BuildStatus, BuildStatusStates

from osrfbitbucket.client import OSRFBitbucketClient, StatusData

def main(argv):
    try:
        opts, args = getopt.getopt(argv,"l:u:p:f:d:s", ["user=",
                                                        "pass=",
                                                        "load_from_file=",
                                                        "desc=",
                                                        "status="])
    except getopt.GetoptError:
      print """
            osrf_pr_from_file --user <bitbucket_user>
                              --pass <bitbucket_pass>
                              --load_from_file <path>
                              --desc <build_description>
                              --status [ok|failed|progress]
            """
      sys.exit(1)

    # Optional parameters
    jenkins_build_url         = None
    jenkins_build_name        = None
    jenkins_build_description = None

    for opt, arg in opts:
        if opt in ('-l', '--load_from_file'):
            config_file = arg
        elif opt in ('-u', '--user'):
            user = arg
        elif opt in ('-p', '--pass'):
            password = arg
        elif opt in ('-d', '--desc'):
            jenkins_build_description = arg
        elif opt in ('-s', '--status'):
            status = arg
       
    client = OSRFBitbucketClient(user, password)
    client.send_build_status(client.build_data_from_file(config_file), 
                             StatusData(
                                client.get_build_status_from_str(status),
                                jenkins_build_description))

if __name__ == "__main__":
   main(sys.argv[1:])
