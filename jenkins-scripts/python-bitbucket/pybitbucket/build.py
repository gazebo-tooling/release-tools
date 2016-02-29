# -*- coding: utf-8 -*-
"""
Defines the BuildStatus resource and registers the type with the Client.

Classes:
- BuildStatusStates: enumerates the possible states of a build status
- BuildStatus: represents the result of a build
"""
from uritemplate import expand

from pybitbucket.bitbucket import Bitbucket, BitbucketBase, Client, enum


BuildStatusStates = enum(
    'BuildStatusStates',
    INPROGRESS='INPROGRESS',
    SUCCESSFUL='SUCCESSFUL',
    FAILED='FAILED')


class BuildStatus(BitbucketBase):
    id_attribute = 'key'
    resource_type = 'build'

    @staticmethod
    def is_type(data):
        return (BuildStatus.has_v2_self_url(data))

    @staticmethod
    def make_payload(
            key,
            state,
            url,
            name=None,
            description=None):
        BuildStatusStates.expect_valid_value(state)
        payload = {
            'key': key,
            'state': state,
            'url': url,
        }
        # Since server defaults may change, method defaults are None.
        # If the parameters are not provided, then don't send them
        # so the server can decide what defaults to use.
        if name is not None:
            payload.update({'name': name})
        if description is not None:
            payload.update({'description': description})
        return payload

    @staticmethod
    def create_buildstatus(
            owner,
            repository_name,
            revision,
            key,
            state,
            url,
            name=None,
            description=None,
            client=Client()):
        template = (
            '{+bitbucket_url}' +
            '/2.0/repositories{/owner,repository_name}' +
            '/commit{/revision}/statuses/build')
        # owner, repository_name, and revision are required
        api_url = expand(
            template, {
                'bitbucket_url': client.get_bitbucket_url(),
                'owner': owner,
                'repository_name': repository_name,
                'revision': revision
            })
        payload = BuildStatus.make_payload(
            key=key,
            state=state,
            url=url,
            name=name,
            description=description)
        return BuildStatus.post(api_url, json=payload, client=client)

    """
    A convenience method for changing the current build status.
    """
    def modify(
            self,
            key=None,
            state=None,
            url=None,
            name=None,
            description=None):
        if (state is None):
            state = self.state
        if (key is None):
            key = self.key
        if (url is None):
            url = self.url
        if (name is None):
            name = self.name
        if (description is None):
            description = self.description
        payload = self.make_payload(
            state=state,
            key=key,
            name=name,
            url=url,
            description=description)
        return self.put(json=payload)

    """
    A convenience method for finding a specific build status.
    In contrast to the pure hypermedia driven method on the Bitbucket
    class, this method returns a BuildStatus object, instead of the
    generator.
    """
    @staticmethod
    def find_buildstatus_for_repository_commit_by_key(
            repository_name,
            revision,
            key,
            owner=None,
            client=Client()):
        if (owner is None):
            owner = client.get_username()
        return next(
            Bitbucket(client=client).repositoryCommitBuildStatusByKey(
                owner=owner,
                repository_name=repository_name,
                revision=revision,
                key=key))

    """
    A convenience method for finding build statuses
    for a repository's commit.
    The method is a generator BuildStatus objects.
    """
    @staticmethod
    def find_buildstatuses_for_repository_commit(
            repository_name,
            revision,
            owner=None,
            client=Client()):
        if (owner is None):
            owner = client.get_username()
        return Bitbucket(client=client).repositoryCommitBuildStatuses(
            owner=owner,
            repository_name=repository_name,
            revision=revision)


Client.bitbucket_types.add(BuildStatus)
