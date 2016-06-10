"""
Provides classes for manipulating Comment resources.
"""
from uritemplate import expand

from pybitbucket.bitbucket import Bitbucket, BitbucketBase, Client


class Comment(BitbucketBase):
    id_attribute = 'id'
    resource_type = 'comments'

    @staticmethod
    def is_type(data):
        return (Comment.has_v2_self_url(data))

    @staticmethod
    def make_payload(content):
        return {'content': {'raw': content}}

    @staticmethod
    def create_comment(
            content,
            snippet_id,
            username=None,
            client=Client()):
        if username is None:
            username = client.get_username()
        template = (
            '{+bitbucket_url}' +
            '/2.0/snippets/{username}/{snippet_id}' +
            '/comments')
        url = expand(
            template, {
                'bitbucket_url': client.get_bitbucket_url(),
                'username': username,
                'snippet_id': snippet_id,
            })
        payload = Comment.make_payload(content)
        response = client.session.post(url, data=payload)
        Client.expect_ok(response)
        return Comment(response.json(), client=client)

    """
    A convenience method for finding a specific comment on a snippet.
    In contrast to the pure hypermedia driven method on the Bitbucket
    class, this method returns a Comment object, instead of the
    generator.
    """
    @staticmethod
    def find_comment_for_snippet_by_id(
            snippet_id,
            comment_id,
            username=None,
            client=Client()):
        if username is None:
            username = client.get_username()
        return next(Bitbucket(client=client).snippetCommentByCommentId(
            username=username,
            snippet_id=snippet_id,
            comment_id=comment_id))

    """
    A convenience method for finding a specific comment on a commit.
    In contrast to the pure hypermedia driven method on the Bitbucket
    class, this method returns a Comment object, instead of the
    generator.
    """
    @staticmethod
    def find_comment_for_repository_commit_by_id(
            owner,
            repository_name,
            revision,
            comment_id,
            client=Client()):
        return next(
            Bitbucket(client=client).repositoryCommitCommentByCommentId(
                owner=owner,
                repository_name=repository_name,
                revision=revision,
                comment_id=comment_id))

    """
    A convenience method for finding a specific comment on a pull request.
    In contrast to the pure hypermedia driven method on the Bitbucket
    class, this method returns a Comment object, instead of the
    generator.
    """
    @staticmethod
    def find_comment_for_repository_pullrequest_by_id(
            owner,
            repository_name,
            pullrequest_id,
            comment_id,
            client=Client()):
        return next(
            Bitbucket(client=client).repositoryPullRequestCommentsByCommentId(
                owner=owner,
                repository_name=repository_name,
                pullrequest_id=pullrequest_id,
                comment_id=comment_id))


Client.bitbucket_types.add(Comment)
