# -*- coding: utf-8 -*-
"""
Classes for abstracting over different forms of Bitbucket authentication.

"""
from six.moves import input

from requests.utils import default_user_agent
from requests import Session
from requests.auth import HTTPBasicAuth
from requests_oauthlib import OAuth2Session, OAuth1Session

from pybitbucket import metadata


class Authenticator(object):

    @staticmethod
    def user_agent_header():
        return u'%s/%s %s' % (
            metadata.package,
            metadata.version,
            default_user_agent())

    @staticmethod
    def headers(email=None, user_agent=None):
        user_agent = user_agent or Authenticator.user_agent_header()
        headers = {
            'Accept': 'application/json',
            'User-Agent': user_agent,
        }
        if email:
            headers.update({'From': email})
        return headers

    def start_http_session(self):
        session = Session()
        session.headers.update(self.headers())
        return session

    def get_username(self):
        return ""


class Anonymous(Authenticator):

    def __init__(self, server_base_uri=None):
        self.server_base_uri = server_base_uri or 'https://api.bitbucket.org'
        self.session = self.start_http_session()


class BasicAuthenticator(Authenticator):

    def start_http_session(self):
        session = Session()
        session.headers.update(self.headers(email=self.client_email))
        session.auth = HTTPBasicAuth(
            self.username,
            self.password)
        return session

    def get_username(self):
        return self.username

    def __init__(
            self,
            username,
            password,
            client_email,
            server_base_uri=None):
        self.server_base_uri = server_base_uri or 'https://api.bitbucket.org'
        self.username = username
        self.password = password
        self.client_email = client_email
        self.session = self.start_http_session()


class OAuth1Authenticator(Authenticator):
    def __init__(
            self,
            client_key,
            client_secret,
            access_token=None,
            access_token_secret=None,
            server_base_uri=None):

        self.server_base_uri = server_base_uri or 'https://api.bitbucket.org'
        self.client_key = client_key
        self.client_secret = client_secret
        self.access_token = access_token
        self.access_token_secret = access_token_secret
        self.username = None
        self.session = self.start_http_session()

    def get_username(self):
        if not self.username:
            self.username = self._fetch_username()
        return self.username

    def _fetch_username(self):
        response = self.session.get('https://api.bitbucket.org/2.0/user')
        response.raise_for_status()
        return response.json()['username']

    def start_http_session(self):
        return OAuth1Session(
            self.client_key,
            client_secret=self.client_secret,
            resource_owner_key=self.access_token,
            resource_owner_secret=self.access_token_secret)


class OAuth2Authenticator(Authenticator):

    def obtain_authorization(self):
        authorization_url = self.session.authorization_url(self.auth_uri)
        print('Please go here and authorize,', authorization_url)
        self.redirect_response = input('Paste the full redirect URL here:')

    def start_http_session(self):
        session = OAuth2Session(self.client_id)
        session.headers.update(self.headers())
        if not session.authorized:
            self.obtain_authorization()
        session.fetch_token(
            self.token_uri,
            authorization_response=self.redirect_response)
        return session

    def get_username(self):
        # TODO: Get user resource to find current username
        return ""

    def __init__(
            self,
            client_id,
            client_secret,
            client_email,
            redirect_uris,
            server_base_uri=None,
            redirect_response=None,
            client_name=None,
            client_description=None,
            auth_uri=None,
            token_uri=None):
        self.server_base_uri = server_base_uri or 'https://api.bitbucket.org'
        # TODO: construct URIs by appending to server_base_uri
        self.auth_uri = (
            auth_uri or
            'https://bitbucket.org/site/oauth2/authorize')
        self.token_uri = (
            token_uri or
            'https://bitbucket.org/site/oauth2/access_token')
        self.client_id = client_id
        self.client_secret = client_secret
        self.redirect_uris = redirect_uris
        self.client_email = client_email
        self.client_name = client_name
        self.client_description = client_description
        self.redirect_response = redirect_response
        self.session = self.start_http_session()
