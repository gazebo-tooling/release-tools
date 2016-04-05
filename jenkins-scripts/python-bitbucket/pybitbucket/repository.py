"""
Provides a class for manipulating Repository resources on Bitbucket.
"""
import json
from uritemplate import expand

from pybitbucket.bitbucket import Bitbucket, BitbucketBase, Client, enum


RepositoryRole = enum(
    'RepositoryRole',
    OWNER='owner',
    ADMIN='admin',
    CONTRIBUTOR='contributor',
    MEMBER='member')


RepositoryForkPolicy = enum(
    'RepositoryForkPolicy',
    ALLOW_FORKS='allow_forks',
    NO_PUBLIC_FORKS='no_public_forks',
    NO_FORKS='no_forks')


RepositoryType = enum(
    'RepositoryType',
    GIT='git',
    HG='hg')


class Repository(BitbucketBase):
    id_attribute = 'full_name'
    resource_type = 'repositories'

    @staticmethod
    def is_type(data):
        return (Repository.has_v2_self_url(data))

    def __init__(self, data, client=Client()):
        super(Repository, self).__init__(data, client=client)
        if data.get('links', {}).get('clone'):
            self.clone = {
                clone_method['name']: clone_method['href']
                for clone_method
                in data['links']['clone']}

    @staticmethod
    def make_new_repository_payload(
            fork_policy,
            is_private,
            scm=None,
            name=None,
            description=None,
            language=None,
            has_issues=None,
            has_wiki=None):
        # Since server defaults may change, method defaults are None.
        # If the parameters are not provided, then don't send them
        # so the server can decide what defaults to use.
        payload = {}
        RepositoryForkPolicy.expect_valid_value(fork_policy)
        payload.update({'fork_policy': fork_policy})
        Repository.expect_bool('is_private', is_private)
        payload.update({'is_private': is_private})
        if scm is not None:
            RepositoryType.expect_valid_value(scm)
            payload.update({'scm': scm})
        if name is not None:
            payload.update({'name': name})
        if description is not None:
            payload.update({'description': description})
        if language is not None:
            payload.update({'language': language})
        if has_issues is not None:
            Repository.expect_bool('has_issues', has_issues)
            payload.update({'has_issues': has_issues})
        if has_wiki is not None:
            Repository.expect_bool('has_wiki', has_wiki)
            payload.update({'has_wiki': has_wiki})
        return payload

    @staticmethod
    def create_repository(
            username,
            repository_name,
            fork_policy,
            is_private,
            scm=None,
            name=None,
            description=None,
            language=None,
            has_issues=None,
            has_wiki=None,
            client=Client()):
        template = (
            '{+bitbucket_url}' +
            '/2.0/repositories/{username}/{repository_name}')
        url = expand(
            template,
            {
                'bitbucket_url': client.get_bitbucket_url(),
                'username': username,
                'repository_name': repository_name
            })
        payload = Repository.make_new_repository_payload(
            fork_policy,
            is_private,
            scm,
            name,
            description,
            language,
            has_issues,
            has_wiki)
        response = client.session.post(url, data=payload)
        Client.expect_ok(response)
        return client.convert_to_object(response.json())

    """
    A convenience method for finding a specific repository.
    In contrast to the pure hypermedia driven method on the Bitbucket
    class, this method returns a Repository object, instead of the
    generator.
    """
    @staticmethod
    def find_repository_by_owner_and_name(
            owner,
            repository_name,
            client=Client()):
        return next(
            Bitbucket(client=client).repositoryByOwnerAndRepositoryName(
                owner=owner,
                repository_name=repository_name))

    """
    A convenience method for finding a specific repository.
    In contrast to the pure hypermedia driven method on the Bitbucket
    class, this method returns a Repository object, instead of the
    generator.
    """
    @staticmethod
    def find_repository_by_full_name(
            repository_full_name,
            client=Client()):
        if '/' not in repository_full_name:
            raise NameError(
                "Repository full name must be in the form: username/name")
        owner, repository_name = repository_full_name.split('/')
        return Repository.find_repository_by_owner_and_name(
            owner,
            repository_name,
            client=client)

    """
    A convenience method for finding public repositories.
    The method is a generator Repository objects.
    """
    @staticmethod
    def find_public_repositories(client=Client()):
        return Bitbucket(client=client).repositoriesThatArePublic()

    """
    A convenience method for finding a user's repositories.
    The method is a generator Repository objects.
    """
    @staticmethod
    def find_repositories_by_owner_and_role(
            owner,
            role=RepositoryRole.OWNER,
            client=Client()):
        RepositoryRole.expect_valid_value(role)
        return Bitbucket(client=client).repositoriesByOwnerAndRole(
            owner=owner,
            role=role)

    """
    A convenience method for finding current user's repositories.
    The method is a generator Repository objects.
    """
    @staticmethod
    def find_my_repositories_by_role(
            role=RepositoryRole.OWNER,
            client=Client()):
        RepositoryRole.expect_valid_value(role)
        return Bitbucket(client=client).repositoriesByOwnerAndRole(
            owner=client.get_username(),
            role=role)


class RepositoryV1(BitbucketBase):
    id_attribute = 'name'
    links_json = """
{
  "_links": {
    "changesets": {
      "href": "https://api.bitbucket.org/1.0/repositories{/owner,slug}/changesets{?limit,start}"
    },
    "deploy_keys": {
      "href": "https://api.bitbucket.org/1.0/repositories{/owner,slug}/deploy-keys"
    },
    "events": {
      "href": "https://api.bitbucket.org/1.0/repositories{/owner,slug}/events"
    },
    "followers": {
      "href": "https://api.bitbucket.org/1.0/repositories{/owner,slug}/followers"
    },
    "issues": {
      "href": "https://api.bitbucket.org/1.0/repositories{/owner,slug}/issues"
    },
    "integration_links": {
      "href": "https://api.bitbucket.org/1.0/repositories{/owner,slug}/links"
    },
    "services": {
      "href": "https://api.bitbucket.org/1.0/repositories{/owner,slug}/services"
    },
    "src": {
      "href": "https://api.bitbucket.org/1.0/repositories{/owner,slug}/src{/revision,path}"
    },
    "wiki": {
      "href": "https://api.bitbucket.org/1.0/repositories{/owner,slug}/wiki{/page}"
    }
  }
}
"""  # noqa

    @staticmethod
    def is_type(data):
        return (
            # Categorize as 1.0 structure
            (data.get('resource_uri') is not None) and
            # Categorize as repo-like (repo or snippet)
            (data.get('scm') is not None) and
            # Categorize as repo, not snippet
            (data.get('slug') is not None))

    def self(self):
        return Repository.find_repository_by_owner_and_name(
            self.owner,
            self.slug,
            client=self.client)

    def __init__(self, data, client=Client()):
        super(RepositoryV1, self).__init__(data, client)
        self.add_remote_relationship_methods(
            json.loads(RepositoryV1.links_json))


Client.bitbucket_types.add(Repository)
Client.bitbucket_types.add(RepositoryV1)
