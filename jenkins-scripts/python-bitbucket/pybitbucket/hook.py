# -*- coding: utf-8 -*-
"""
Defines the Hook resource and registers the type with the Client.

Classes:
- Hook: represents a web hook for a repository
"""
from uritemplate import expand

from pybitbucket.bitbucket import Bitbucket, BitbucketBase, Client


class Hook(BitbucketBase):
    id_attribute = 'uuid'
    resource_type = 'hooks'

    @staticmethod
    def is_type(data):
        return (Hook.has_v2_self_url(data))

    @staticmethod
    def make_payload(
            description,
            callback_url,
            active=True,
            events=('repo:push',)
    ):
        payload = {
            'description': description,
            'url': callback_url,
        }
        Hook.expect_bool('active', active)
        payload.update({'active': active})
        Hook.expect_list('events', events)
        payload.update({'events': events})
        return payload

    @staticmethod
    def create_hook(
            repository_name,
            description,
            callback_url,
            active=None,
            events=None,
            username=None,
            client=Client()):
        template = (
            '{+bitbucket_url}' +
            '/2.0/repositories/{username}/{repository_name}/hooks')
        if username is None:
            username = client.get_username()
        url = expand(
            template, {
                'bitbucket_url': client.get_bitbucket_url(),
                'username': username,
                'repository_name': repository_name
            })
        payload = Hook.make_payload(description, callback_url, active, events)
        return Hook.post(url, json=payload, client=client)

    @staticmethod
    def find_hook_in_repository_by_uuid(
            owner,
            repository_name,
            uuid,
            client=Client()):
        """
        A convenience method for finding a hook by uuid and repo name.
        The method returns a Hook object.
        """
        return next(Bitbucket(client=client).repositoryHookById(
            owner=owner, repository_name=repository_name, uuid=uuid))

    @staticmethod
    def find_hooks_in_repository(
            owner,
            repository_name,
            client=Client()):
        """
        A convenience method for finding hooks by repo name.
        The method is a generator for Hook objects
        """
        return Bitbucket(client=client).repositoryHooks(
            owner=owner, repository_name=repository_name)

    def modify(
            self,
            description=None,
            callback_url=None,
            active=None,
            events=None):
        """
        A convenience method for changing the current hook.
        The parameters make it easier to know what can be changed.
        """
        if (description is None):
            description = self.description
        if (callback_url is None):
            callback_url = self.callback_url
        if (active is None):
            active = self.active
        if (events is None):
            events = self.events
        payload = self.make_payload(
            description=description,
            callback_url=callback_url,
            active=active,
            events=events)
        return self.put(json=payload)

    def delete(self):
        """
        A convenience method for deleting the current hook.
        """
        return super(Hook, self).delete()


Client.bitbucket_types.add(Hook)
