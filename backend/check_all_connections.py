import os
import requests
import psycopg2
from dotenv import load_dotenv
from qdrant_client import QdrantClient
import openai
from urllib.parse import urlparse, parse_qs, urlencode, urlunparse

load_dotenv()

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
        'sslcompression', 'sslfactory', 'sslcert', 'sslkey',
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
            del query_params[param]

    # Rebuild query string without extracted parameters
    new_query = urlencode(query_params, doseq=True)
    new_parsed = parsed._replace(query=new_query)
    clean_database_url = urlunparse(new_parsed)

    return clean_database_url, connection_params

def test_status(name, success, error=""):
    status = "[OK]" if success else f"[FAILED] ({error})"
    print(f"{name:<25} : {status}")

print("\n--- Starting System Connection Audit ---\n")

# 1. Frontend to Backend (FastAPI Health)
try:
    # Maan lete hain aapka server port 8000 par chal raha hai
    response = requests.get("http://localhost:8000/health", timeout=5)
    test_status("1. FastAPI Server", response.status_code == 200)
except Exception as e:
    test_status("1. FastAPI Server", False, "Server not running")

# 2. Vector DB (Qdrant Cloud)
try:
    endpoint = os.getenv("QDRANT_ENDPOINT")
    api_key = os.getenv("QDRANT_API_KEY")

    if not endpoint or not api_key:
        test_status("2. Qdrant Cloud", False, "Missing endpoint or API key in .env")
    else:
        client = QdrantClient(
            url=endpoint,
            api_key=api_key,
            timeout=10
        )
        # Check connection by getting collections
        client.get_collections()
        test_status("2. Qdrant Cloud", True)
except Exception as e:
    error_msg = str(e)
    if "404" in error_msg:
        error_msg = "Invalid endpoint or API key - check .env configuration. Visit https://qdrant.tech/ to set up a free instance."
    elif "forbidden" in error_msg.lower():
        error_msg = "Forbidden access - check API key validity"
    test_status("2. Qdrant Cloud", False, error_msg)

# 3. SQL DB (Neon Postgres)
try:
    # Extract connection parameters that need to be passed separately to psycopg2
    database_url = os.getenv("DATABASE_URL")
    if database_url:
        clean_url, connection_params = extract_connection_params_from_url(database_url)

        # Extract sslmode separately to avoid the keyword argument issue
        sslmode = connection_params.pop('sslmode', None)

        # Prepare connection arguments
        conn_args = {
            'dsn': clean_url
        }

        # Add sslmode as a connection parameter if it exists
        if sslmode:
            conn_args['sslmode'] = sslmode

        # Add other connection parameters
        conn_args.update(connection_params)

        conn = psycopg2.connect(**conn_args)
        conn.close()
        test_status("3. Neon Postgres", True)
    else:
        test_status("3. Neon Postgres", False, "DATABASE_URL not set in .env")
except Exception as e:
    test_status("3. Neon Postgres", False, str(e))

# 4. AI API (OpenAI/Claude)
try:
    # Sirf check karne ke liye ke key valid hai
    openai.api_key = os.getenv("OPENAI_API_KEY")
    # Yahan ek choti si request bhi ki ja sakti hai
    test_status("4. AI API (OpenAI)", bool(os.getenv("OPENAI_API_KEY")))
except Exception as e:
    test_status("4. AI API (OpenAI)", False, str(e))

# 5. Deployment (GitHub Remote)
try:
    import subprocess
    result = subprocess.run(['git', 'remote', '-v'], capture_output=True, text=True)
    test_status("5. GitHub Remote", "origin" in result.stdout)
except:
    test_status("5. GitHub Remote", False, "Git not initialized")

print("\n--- Audit Complete ---\n")