# -*- coding: utf-8 -*-
"""
Defines the BranchRestriction resource and registers the type with the Client.

Classes:
- BranchRestriction: represents a restriction on a branch for a repository.
"""
from uritemplate import expand
from pybitbucket.bitbucket import Bitbucket, BitbucketBase, Client, enum


BranchRestrictionKind = enum(
    'BranchRestrictionKind',
    PUSH='push',
    DELETE='delete',
    FORCE='force')


class BranchRestriction(BitbucketBase):
    id_attribute = 'id'
    resource_type = 'branch-restrictions'

    @staticmethod
    def is_type(data):
        return (BranchRestriction.has_v2_self_url(data))

    @staticmethod
    def payload(
            kind=None,
            pattern=None,
            groups=None,
            users=None):
        payload = {}
        # Since server defaults may change, method defaults are None.
        # If the parameters are not provided, then don't send them
        # so the server can decide what defaults to use.
        if kind is not None:
            BranchRestrictionKind.expect_valid_value(kind)
            payload.update({'kind': kind})
        if pattern is not None:
            payload.update({'pattern': pattern})
        if groups is not None:
            BranchRestriction.expect_list('groups', groups)
            payload.update({'groups': groups})
        if users is not None:
            BranchRestriction.expect_list('users', users)
            payload.update({'users': [{'username': u} for u in users]})
        return payload

    @staticmethod
    def create(
            owner,
            repository_name,
            kind,
            pattern=None,
            groups=None,
            users=None,
            client=Client()):
        template = (
            '{+bitbucket_url}' +
            '/2.0/repositories{/owner,repository_name}' +
            '/branch-restrictions')
        url = expand(
            template, {
                'bitbucket_url': client.get_bitbucket_url(),
                'owner': owner,
                'repository_name': repository_name,
            })
        payload = BranchRestriction.payload(
            kind=kind,
            pattern=pattern,
            groups=groups,
            users=users)
        return BranchRestriction.post(url, json=payload, client=client)

    """
    A convenience method for changing the current branch-restriction.
    The parameters make it easier to know what can be changed.
    """
    def update(
            self,
            kind=None,
            pattern=None,
            groups=None,
            users=None):
        payload = self.payload(
            kind=kind,
            pattern=pattern,
            groups=groups,
            users=users)
        return self.put(payload)

    """
    A convenience method for finding branch-restrictions for a repository.
    The method is a generator BranchRestriction objects.
    """
    @staticmethod
    def find_branchrestrictions_for_repository(
            repository_name,
            owner=None,
            client=Client()):
        if (owner is None):
            owner = client.get_username()
        return Bitbucket(client=client).repositoryBranchRestrictions(
            owner=owner,
            repository_name=repository_name)

    """
    A convenience method for finding a specific branch-restriction.
    In contrast to the pure hypermedia driven method on the Bitbucket
    class, this method returns a BranchRestriction object, instead of the
    generator.
    """
    @staticmethod
    def find_branchrestriction_for_repository_by_id(
            repository_name,
            restriction_id,
            owner=None,
            client=Client()):
        if (owner is None):
            owner = client.get_username()
        return next(
            Bitbucket(
                client=client).repositoryBranchRestrictionByRestrictionId(
                    owner=owner,
                    repository_name=repository_name,
                    restriction_id=restriction_id))


Client.bitbucket_types.add(BranchRestriction)
