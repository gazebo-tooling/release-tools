"""
Provides a class for manipulating Team resources on Bitbucket.
"""
from pybitbucket.bitbucket import Bitbucket, BitbucketBase, Client, enum


TeamRole = enum(
    'TeamRole',
    ADMIN='admin',
    CONTRIBUTOR='contributor',
    MEMBER='member')


class Team(BitbucketBase):
    id_attribute = 'username'
    resource_type = 'teams'

    @staticmethod
    def is_type(data):
        return (Team.has_v2_self_url(data))

    """
    A convenience method for finding teams by the user's role.
    The method is a generator Team objects.
    """
    @staticmethod
    def find_teams_for_role(role=TeamRole.ADMIN, client=Client()):
        TeamRole.expect_valid_value(role)
        return Bitbucket(client=client).teamsForRole(role=role)

    """
    A convenience method for finding a specific team.
    In contrast to the pure hypermedia driven method on the Bitbucket
    class, this method returns a User object, instead of the
    generator.
    """
    @staticmethod
    def find_team_by_username(username, client=Client()):
        return next(Bitbucket(client=client).teamByUsername(
            username=username))


Client.bitbucket_types.add(Team)
