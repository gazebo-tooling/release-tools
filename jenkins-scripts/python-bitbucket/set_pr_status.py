#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, getopt

from pprint import pprint

from pybitbucket.auth import BasicAuthenticator
from pybitbucket.bitbucket import Client
from pybitbucket.build import BuildStatus, BuildStatusStates

from osrfbitbucket.client import *

def main(argv):
    try:
        opts, args = getopt.getopt(argv,"u:p:n:s:t:j:r:b:d",
            ["user=","pass=","repository_name=","sha=", "status=", "jenkins_job_name=","jenkins_build_url=","jenkins_build_name=","jenkins_build_description="])
 
    except getopt.GetoptError:
      print """
            osrf_pr_status.py --user <bitbucket_user>
                              --pass <bitbucket_pass>
                              --repository_name <repo_name> 
                              --sha <commit> 
                              --status [ok|failed|progress]
                              --jenkins_job_name <name>
                              [--jenkins_build_url<url>]
                              [--jenkins_build_name <name>]
                              [--jenkins_build_description <desc>]
            """
      sys.exit(1)

    # Optional parameters
    jenkins_build_url         = None
    jenkins_build_name        = None
    jenkins_build_description = None

    for opt, arg in opts:
        if opt == '--user':
            user = arg
        elif opt == '--pass':
            password = arg
        elif opt == '--status':
            status = arg
        elif opt == '--repository_name':
            repository_name = arg
        elif opt == '--sha':
            sha = arg
        elif opt == '--jenkins_job_name':
            jenkins_job_name = arg
        elif opt == '--jenkins_build_url':
            jenkins_build_url = arg
        elif opt == '--jenkins_build_name':
            jenkins_build_name = arg
        elif opt == '--jenkins_build_description':
            jenkins_build_description = arg
        
    client = OSRFBitbucketClient(user, password)

    print("Sending status: " + status + " to " + 
          repository_name + " commit " + sha) 

    client.send_build_status(BuildData(
                               BuildSourceData(repository_name,
                                               sha),
                               BuildJenkinsData(jenkins_job_name,
                                                jenkins_build_url,
                                                jenkins_build_name,
                                                jenkins_build_description)),
                             client.get_build_status_from_str(status))

if __name__ == "__main__":
   main(sys.argv[1:])
