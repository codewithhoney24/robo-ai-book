import os
import psycopg2
from urllib.parse import urlparse

# Get the database URL
database_url = os.getenv("DATABASE_URL", "postgresql://localhost:5432/auth_service_db?sslmode=prefer")

# Parse the database URL to get connection details
parsed_url = urlparse(database_url)
host = parsed_url.hostname or 'localhost'
port = parsed_url.port or 5432
username = parsed_url.username or 'postgres'
password = parsed_url.password
database_name = parsed_url.path[1:]  # Remove the leading '/'

print(f"Attempting to connect to PostgreSQL server at {host}:{port}")

try:
    # Connect to PostgreSQL server (to the default postgres database)
    conn = psycopg2.connect(
        host=host,
        port=port,
        user=username,
        password=password,
        database='postgres'  # Connect to default postgres db first
    )

    conn.autocommit = True  # Required for CREATE DATABASE
    cur = conn.cursor()

    # Check if database exists
    cur.execute("SELECT 1 FROM pg_catalog.pg_database WHERE datname = %s", (database_name,))
    exists = cur.fetchone()

    if not exists:
        cur.execute(f"CREATE DATABASE {database_name};")
        print(f"Database '{database_name}' created successfully!")
    else:
        print(f"Database '{database_name}' already exists.")

    cur.close()
    conn.close()

    print(f"Database '{database_name}' is ready for use!")

except psycopg2.OperationalError as e:
    print(f"Error connecting to PostgreSQL: {e}")
    print("Make sure PostgreSQL server is running and credentials are correct.")
    print("If you don't have PostgreSQL installed, visit: https://www.postgresql.org/download/")
except psycopg2.Error as e:
    print(f"Error with PostgreSQL: {e}")