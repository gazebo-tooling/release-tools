# -*- coding: utf-8 -*-

entrypoints_json = """
{
  "_links": {
    "self": {
      "href": "https://api.bitbucket.org/"
    },
    "userForMyself": {
      "href": "https://api.bitbucket.org/2.0/user"
    },
    "userByUsername": {
      "href": "https://api.bitbucket.org/2.0/users{/username}"
    },
    "teamsForRole": {
      "href": "https://api.bitbucket.org/2.0/teams{?role}"
    },
    "teamByUsername": {
      "href": "https://api.bitbucket.org/2.0/teams{/username}"
    },
    "repositoriesThatArePublic": {
      "href": "https://api.bitbucket.org/2.0/repositories"
    },
    "repositoriesByOwnerAndRole": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner}{?role}"
    },
    "repositoryByOwnerAndRepositoryName": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}"
    },
    "repositoryWatchers": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}/watchers"
    },
    "repositoryForks": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}/forks"
    },
    "repositoryBranchRestrictions": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}/branch-restrictions"
    },
    "repositoryBranchRestrictionByRestrictionId": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}/branch-restrictions{/restriction_id}"
    },
    "repositoryCommits": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}/commits{/branch}{?include*,exclude*}"
    },
    "repositoryCommitByRevision": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}/commit{/revision}"
    },
    "repositoryCommitComments": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}/commit{/revision}/comments"
    },
    "repositoryCommitCommentByCommentId": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}/commit{/revision}/comments{/comment_id}"
    },
    "repositoryCommitBuildStatuses": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}/commit{/revision}/statuses/build"
    },
    "repositoryCommitBuildStatusByKey": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}/commit{/revision}/statuses/build{/key}"
    },
    "repositoryPullRequestsInState": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}/pullrequests{?state}"
    },
    "repositoryPullRequestActivitiesForWholeRepository": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}/pullrequests/activity"
    },
    "repositoryPullRequestByPullRequestId": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}/pullrequests{/pullrequest_id}"
    },
    "repositoryPullRequestActivitiesForPullRequest": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}/pullrequests{/pullrequest_id}/activity"
    },
    "repositoryPullRequestCommits": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}/pullrequests{/pullrequest_id}/commits"
    },
    "repositoryPullRequestDiff": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}/pullrequests{/pullrequest_id}/diff"
    },
    "repositoryPullRequestComments": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}/pullrequests{/pullrequest_id}/comments"
    },
    "repositoryPullRequestCommentsByCommentId": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}/pullrequests{/pullrequest_id}/comments{/comment_id}"
    },
    "repositoryHooks": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}/hooks"
    },
    "repositoryHookById": {
      "href": "https://api.bitbucket.org/2.0/repositories{/owner,repository_name}/hooks{/uuid}"
    },
    "snippetsThatArePublic": {
      "href": "https://api.bitbucket.org/2.0/snippets"
    },
    "snippetsForRole": {
      "href": "https://api.bitbucket.org/2.0/snippets{?role}"
    },
    "snippetByOwner": {
      "href": "https://api.bitbucket.org/2.0/snippets{/owner}"
    },
    "snippetByOwnerAndSnippetId": {
      "href": "https://api.bitbucket.org/2.0/snippets{/owner,snippet_id}"
    },
    "snippetCommentByCommentId": {
      "href": "https://api.bitbucket.org/2.0/snippets{/owner,snippet_id}/comments{/comment_id}"
    },
    "privilegesForOwner": {
      "href": "https://api.bitbucket.org/1.0/privileges{/owner}"
    },
    "privilegesForRepository": {
      "href": "https://api.bitbucket.org/1.0/privileges{/owner,repository_name}"
    },
    "privilegesForRepositoryAndUsername": {
      "href": "https://api.bitbucket.org/1.0/privileges{/owner,repository_name,username}"
    },
    "groupsForFilter": {
      "href": "https://api.bitbucket.org/1.0/groups{?filter*}"
    },
    "groupsForOwner": {
      "href": "https://api.bitbucket.org/1.0/groups{/owner}"
    },
    "groupForOwnerAndGroupId": {
      "href": "https://api.bitbucket.org/1.0/groups{/owner,group_id}"
    },
    "groupMembers": {
      "href": "https://api.bitbucket.org/1.0/groups{/owner,group_id}/members"
    },
    "groupPrivilegesForRepositoryOwner": {
      "href": "https://api.bitbucket.org/1.0/group-privileges{/repository_owner}"
    },
    "groupPrivilegesForRepository": {
      "href": "https://api.bitbucket.org/1.0/group-privileges{/repository_owner,repository_id}"
    },
    "groupPrivilegesForRepositoryPrivilegesGroup": {
      "href": "https://api.bitbucket.org/1.0/group-privileges{/repository_owner,group_owner,group_id}"
    },
    "groupForRepositoryAndGroup": {
      "href": "https://api.bitbucket.org/1.0/group-privileges{/repository_owner,repository_id,group_owner,group_id}"
    }
  }
}
"""
