#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, getopt

from pprint import pprint

from pybitbucket.auth import BasicAuthenticator
from pybitbucket.bitbucket import Client
from pybitbucket.build import BuildStatus, BuildStatusStates

from osrfbitbucket.client import OSRFBitbucketClient

def main(argv):
    try:
        opts, args = getopt.getopt(argv,"l:u:p:f:s", ["user=",
                                                      "pass=",
                                                      "load_from_file=",
                                                      "status="])
    except getopt.GetoptError:
      print """
            osrf_pr_from_file --user <bitbucket_user>
                              --pass <bitbucket_pass>
                              --load_from_file <path>
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
        if opt == '--user':
            user = arg
        elif opt == '--pass':
            password = arg
        elif opt == '--status':
            status = arg
        
    client = OSRFBitbucketClient(user, password)
    client.send_build_status(client.build_data_from_file(config_file), 
                             client.get_build_status_from_str(status))

if __name__ == "__main__":
   main(sys.argv[1:])
