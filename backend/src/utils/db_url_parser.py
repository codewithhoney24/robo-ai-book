"""
Utility functions to properly parse and handle database URLs with special parameters
that are not compatible with certain database drivers.
"""
from urllib.parse import urlparse, parse_qs, urlencode, urlunparse
import logging

logger = logging.getLogger(__name__)

def clean_database_url_for_psycopg2(database_url: str) -> str:
    """
    Clean a database URL by removing parameters that psycopg2 doesn't support directly in the URL.
    These parameters need to be passed separately to the connect function.

    Args:
        database_url: The original database URL string

    Returns:
        str: A cleaned database URL without problematic parameters
    """
    # Parse the database URL to handle special parameters
    parsed = urlparse(database_url)

    # Extract query parameters
    query_params = parse_qs(parsed.query)

    # Remove parameters that psycopg2 doesn't support directly in the URL
    # These need to be handled separately
    params_to_remove = [
        'sslmode', 'channel_binding', 'sslcert', 'sslkey', 'sslrootcert',
        'sslmode', 'sslcompression', 'sslfactory', 'sslcert', 'sslkey',
        'sslmode', 'gssencmode', 'channel_binding'
    ]

    for param in params_to_remove:
        if param in query_params:
            del query_params[param]

    # Rebuild query string without problematic parameters
    new_query = urlencode(query_params, doseq=True)
    new_parsed = parsed._replace(query=new_query)
    clean_database_url = urlunparse(new_parsed)

    return clean_database_url


def extract_connection_params_from_url(database_url: str) -> tuple:
    """
    Extract connection parameters from a database URL that need to be passed separately
    to database drivers like psycopg2.

    Args:
        database_url: The original database URL string

    Returns:
        tuple: (cleaned_url, params_dict) where params_dict contains the extracted parameters
    """
    # Parse the database URL to handle special parameters
    parsed = urlparse(database_url)

    # Extract query parameters
    query_params = parse_qs(parsed.query)

    # Extract parameters that need to be passed separately to psycopg2
    connection_params = {}
    params_to_extract = [
        'sslmode', 'channel_binding', 'sslcert', 'sslkey', 'sslrootcert',
        'sslcompression', 'sslfactory', 'gssencmode'
    ]

    for param in params_to_extract:
        if param in query_params:
            # parse_qs returns lists, so we take the first value
            connection_params[param] = query_params[param][0]
            # Remove from query params since they'll be passed separately
            del query_params[param]

    # Rebuild query string without extracted parameters
    new_query = urlencode(query_params, doseq=True)
    new_parsed = parsed._replace(query=new_query)
    clean_database_url = urlunparse(new_parsed)

    return clean_database_url, connection_params


def clean_database_url_for_asyncpg(database_url: str) -> str:
    """
    Clean a database URL by removing parameters that asyncpg doesn't support directly in the URL.

    Args:
        database_url: The original database URL string

    Returns:
        str: A cleaned database URL without problematic parameters
    """
    # Parse the database URL to handle special parameters
    parsed = urlparse(database_url)

    # Extract query parameters
    query_params = parse_qs(parsed.query)

    # Remove parameters that asyncpg doesn't support directly in the URL
    params_to_remove = [
        'sslmode',  # asyncpg doesn't support sslmode in URL
        'channel_binding',  # asyncpg doesn't support channel_binding
    ]

    for param in params_to_remove:
        if param in query_params:
            del query_params[param]

    # Rebuild query string without problematic parameters
    new_query = urlencode(query_params, doseq=True)
    new_parsed = parsed._replace(query=new_query)
    clean_database_url = urlunparse(new_parsed)

    return clean_database_url