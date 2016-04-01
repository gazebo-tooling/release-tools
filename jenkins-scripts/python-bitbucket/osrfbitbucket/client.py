from pprint import pprint

import yaml
import sys

from pybitbucket.auth import BasicAuthenticator
from pybitbucket.bitbucket import Client
from pybitbucket.build import BuildStatus, BuildStatusStates

class BuildJenkinsData:
    def __init__(self, jenkins_job_name, 
                       jenkins_build_url = None,
                       jenkins_build_name = None,
                       jenkins_build_description = None):
        self.job_name  = jenkins_job_name
        self.build_url = jenkins_build_url        
 
        if jenkins_build_name == None:
            self.build_name = jenkins_job_name
        else:
            self.build_name = jenkins_build_name

        if jenkins_build_description == None:
            self.build_description = jenkins_job_name
        else:
            self.build_description = jenkins_build_description

class BuildSourceData:
    def __init__(self, repository_name, sha):
        self.owner             = repository_name.partition("/")[0]
        self.repo_short_name   = repository_name.partition("/")[2]
        self.sha               = sha

class BuildData:
    def __init__(self, build_source_data, build_jenkins_data):
        self.source_data  = build_source_data
        self.jenkins_data = build_jenkins_data

class OSRFBitbucketClient:
    def __init__(self, user, password):
        self.client = Client(
            BasicAuthenticator(
                user,
                password,
                'jrivero@osrfoundation.org'))

        self.status_str = ''

    @staticmethod
    def get_build_status_from_str(status):
     if status == 'ok':
        return BuildStatusStates.SUCCESSFUL
     elif status == 'failed':
        return BuildStatusStates.FAILED
     elif status == 'inprogress':
        return BuildStatusStates.INPROGRESS
     else:
         print("Invalid status: " + status)
         sys.exit(1)

    @staticmethod
    def build_data_from_file(config_file):
        stream           = file(config_file, 'r')
        config           = yaml.load(stream)
        bitbucket_origin = config['bitbucket_origin']
        src_data         = config['jenkins_job']
        
        return BuildData(
                    BuildSourceData(bitbucket_origin["repository_name"], 
                                    bitbucket_origin["sha"]),
                    BuildJenkinsData(src_data["name"],
                                     src_data["url"]))

    def send_build_status(self, build_data, status):

        # Define key as the first 40 chars (bitbucket limit) of the 
        # job-name just after -ci-pr_any-. This should leave the
        # testing platform and architecture
        try:
            key = build_data.jenkins_data.job_name.split("-ci-pr_any-")[1][0:39]
        except: # fallback to use 40 first chars of job_name
            key = build_data.jenkins_data.job_name[0:39]

        build_status = BuildStatus.create_buildstatus(
            owner           = build_data.source_data.owner,
            repository_name = build_data.source_data.repo_short_name,
            revision        = build_data.source_data.sha,
            key             = key,
            name            = build_data.jenkins_data.build_name,
            url             = build_data.jenkins_data.build_url,
            description     = build_data.jenkins_data.build_description,
            state           = status,
            client          = self.client)

        pprint(build_status)

        return build_status
