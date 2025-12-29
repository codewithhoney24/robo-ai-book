import asyncio
import asyncpg
from urllib.parse import urlparse, parse_qs, urlencode, urlunparse

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

# Test database connection directly with asyncpg
async def db_test():
    # Database URL from the .env in backend
    database_url = "postgresql://neondb_owner:npg_gPuCILYeV73E@ep-square-glitter-a854b5sd-pooler.eastus2.azure.neon.tech/neondb"

    try:
        # Clean the database URL for asyncpg
        clean_url = clean_database_url_for_asyncpg(database_url)
        conn = await asyncpg.connect(clean_url)

        # Run the sanity test query (without created_at since it's not in the table by default)
        rows = await conn.fetch("""
            SELECT email, software_background, hardware_background
            FROM "user"
            LIMIT 5;
        """)

        if rows:
            print('DB SANITY TEST: Found', len(rows), 'users in the database')
            print('Sample users:')
            for i, row in enumerate(rows, 1):
                print(f'  {i}. Email: {row["email"]}, Software: {row["software_background"]}, Hardware: {row["hardware_background"]}')
        else:
            print('DB SANITY TEST: No users found in the database')
            print('This means either no signups have occurred or there might be an issue with the signup API')

        await conn.close()

    except Exception as e:
        print(f'DB SANITY TEST: Error connecting or querying: {e}')

if __name__ == '__main__':
    asyncio.run(db_test())