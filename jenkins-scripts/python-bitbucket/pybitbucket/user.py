"""
Provides a class for manipulating User resources on Bitbucket.
"""
import json

from pybitbucket.bitbucket import Bitbucket, BitbucketBase, Client


class User(BitbucketBase):
    id_attribute = 'username'
    resource_type = 'users'

    @staticmethod
    def is_type(data):
        return (User.has_v2_self_url(data))

    """
    A convenience method for finding the current user.
    In contrast to the pure hypermedia driven method on the Bitbucket
    class, this method returns a User object, instead of the
    generator.
    """
    @staticmethod
    def find_current_user(client=Client()):
        return next(Bitbucket(client=client).userForMyself())

    """
    A convenience method for finding a specific user.
    In contrast to the pure hypermedia driven method on the Bitbucket
    class, this method returns a User object, instead of the
    generator.
    """
    @staticmethod
    def find_user_by_username(username, client=Client()):
        return next(Bitbucket(client=client).userByUsername(
            username=username))


class UserV1(BitbucketBase):
    id_attribute = 'username'
    links_json = """
{
  "_links": {
    "plan": {
      "href": "https://api.bitbucket.org/1.0/users{/username}/plan"
    },
    "followers": {
      "href": "https://api.bitbucket.org/1.0/users{/username}/followers"
    },
    "events": {
      "href": "https://api.bitbucket.org/1.0/users{/username}/events"
    }
}
"""

    @staticmethod
    def is_type(data):
        return (
            # Make sure there is a user structure
            (data.get('user') is not None) and
            # Categorize as 1.0 structure
            (data['user'].get('resource_uri') is not None) and
            # Categorize as user-like (user or team)
            (data['user'].get('username') is not None) and
            # Categorize as user, not team
            (data['user'].get('is_team') is False))

    def self(self):
        return User.find_user_by_username(
            self.username,
            client=self.client)

    def __init__(self, data, client=Client()):
        # This completely override the base constructor
        # because the user data is a child of the root object.
        self.data = data
        self.client = client
        if data.get('user'):
            self.__dict__.update(data['user'])
        if data.get('repositories'):
            self.repositories = [
                client.convert_to_object(r)
                for r
                in data['repositories']]
        self.add_remote_relationship_methods(
            json.loads(UserV1.links_json))


Client.bitbucket_types.add(User)
Client.bitbucket_types.add(UserV1)
