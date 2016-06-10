# -*- coding: utf-8 -*-
"""
Defines the PullRequest resource and registers the type with the Client.

Classes:
- PullRequestState: enumerates the possible states of a pull request
- PullRequest: represents a pull request for code review
"""
from functools import partial
from uritemplate import expand

from pybitbucket.bitbucket import Bitbucket, BitbucketBase, Client, enum


PullRequestState = enum(
    'PullRequestState',
    OPEN='open',
    MERGED='merged',
    DECLINED='declined')


class PullRequest(BitbucketBase):
    id_attribute = 'id'
    resource_type = 'pullrequests'

    @staticmethod
    def is_type(data):
        return (PullRequest.has_v2_self_url(data))

    def __init__(self, data, client=Client()):
        super(PullRequest, self).__init__(data, client=client)
        if data.get('source', {}).get('commit', {}):
            self.source_commit = client.convert_to_object(
                data['source']['commit'])
        if data.get('source', {}).get('repository', {}):
            self.source_repository = client.convert_to_object(
                data['source']['repository'])
        if data.get('destination', {}).get('commit', {}):
            self.destination_commit = client.convert_to_object(
                data['destination']['commit'])
        if data.get('destination', {}).get('repository', {}):
            self.destination_repository = client.convert_to_object(
                data['destination']['repository'])
        # Special treatment for approve, decline, merge, and diff
        if data.get('links', {}).get('approve', {}).get('href', {}):
            url = data['links']['approve']['href']
            # Approve is a POST on the approve link
            setattr(self, 'approve', partial(
                self.post_approval, template=url))
            # Unapprove is a DELETE on the approve link
            setattr(self, 'unapprove', partial(
                self.delete_approval, template=url))
        if data.get('links', {}).get('decline', {}).get('href', {}):
            url = data['links']['decline']['href']
            # Decline is a POST
            setattr(self, 'decline', partial(
                self.post, client=client, url=url, json=None))
        if data.get('links', {}).get('merge', {}).get('href', {}):
            url = data['links']['merge']['href']
            # Merge is a POST
            setattr(self, 'merge', partial(
                self.post, client=client, url=url, json=None))
        if data.get('links', {}).get('diff', {}).get('href', {}):
            url = data['links']['diff']['href']
            # Diff returns plain text
            setattr(self, 'diff', partial(
                self.content, url=url))

    def content(self, url):
        response = self.client.session.get(url)
        Client.expect_ok(response)
        return response.content

    @staticmethod
    def make_new_pullrequest_payload(
            title,
            source_branch_name,
            source_repository_full_name,
            destination_branch_name,
            close_source_branch=None,
            description=None,
            reviewers=None,
            source_commit=None,
            destination_commit=None):
        payload = {
            'title': title,
            'source': {
                'branch': {
                    'name': source_branch_name
                },
                'repository': {
                    'full_name': source_repository_full_name
                }
            },
            'destination': {
                'branch': {
                    'name': destination_branch_name
                }
            }
        }
        # Since server defaults may change, method defaults are None.
        # If the parameters are not provided, then don't send them
        # so the server can decide what defaults to use.
        if close_source_branch is not None:
            PullRequest.expect_bool('close_source_branch', close_source_branch)
            payload.update({'close_source_branch': close_source_branch})
        if description is not None:
            payload.update({'description': description})
        if reviewers is not None:
            PullRequest.expect_list('reviewers', reviewers)
            payload.update(
                {'reviewers': [{'username': u} for u in reviewers]})
        if source_commit is not None:
            payload.get('source', {}).update(
                {'commit': {'hash': source_commit}})
        if destination_commit is not None:
            payload.get('destination', {}).update(
                {'commit': {'hash': destination_commit}})
        return payload

    @staticmethod
    def create_pullrequest(
            username,
            repository_name,
            title,
            source_branch_name,
            destination_branch_name,
            close_source_branch=None,
            description=None,
            reviewers=None,
            client=Client()):
        template = (
            '{+bitbucket_url}' +
            '/2.0/repositories{/username,repository_name}/pullrequests')
        url = expand(
            template,
            {
                'bitbucket_url': client.get_bitbucket_url(),
                'username': username,
                'repository_name': repository_name
            })
        payload = PullRequest.make_new_pullrequest_payload(
            title,
            source_branch_name,
            (username + '/' + repository_name),
            destination_branch_name,
            close_source_branch,
            description,
            reviewers)
        return PullRequest.post(url, json=payload, client=client)

    """
    A convenience method for finding a specific pull request.
    In contrast to the pure hypermedia driven method on the Bitbucket
    class, this method returns a PullRequest object, instead of the
    generator.
    """
    @staticmethod
    def find_pullrequest_in_repository_by_id(
            owner,
            repository_name,
            pullrequest_id,
            client=Client()):
        return next(
            Bitbucket(client=client).repositoryPullRequestByPullRequestId(
                owner=owner,
                repository_name=repository_name,
                pullrequest_id=pullrequest_id))

    """
    A convenience method for finding pull requests for a repository.
    The method is a generator PullRequest objects.
    If no owner is provided, this method assumes the client can provide one.
    If no state is provided, the server will assume open pull requests.
    """
    @staticmethod
    def find_pullrequests_for_repository_by_state(
            repository_name,
            owner=None,
            state=None,
            client=Client()):
        if (state is not None):
            PullRequestState.expect_state(state)
        if (owner is None):
            owner = client.get_username()
        return Bitbucket(client=client).repositoryPullRequestsInState(
            owner=owner,
            repository_name=repository_name,
            state=state)


Client.bitbucket_types.add(PullRequest)
